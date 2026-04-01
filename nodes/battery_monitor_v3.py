import json
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav2_msgs.srv import ManageLifecycleNodes
from nav2_simple_commander.robot_navigator import BasicNavigator

from action_msgs.msg import GoalStatusArray
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist, PoseStamped, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


# ── Configuration ──────────────────────────────────────────────────────────
INITIAL_CHARGE_PERCENTAGE  = 100.0
LOW_BATTERY_THRESHOLD      = 5.0        # % — triggers low-battery chain
IDLE_DRAIN_PER_SECOND      = 0.05
MOVEMENT_DRAIN_PER_SECOND  = 0.5

SETTLE_DURATION_SEC        = 5.0         # real seconds to wait in SETTLING
CONFIRM_DURATION_SEC       = 5.0         # real seconds to poll in CONFIRM_STOP
BATTERY_OK_DURATION_SEC    = 1.0         # real seconds to stay in BATTERY_OK
WAIT_PICKUP_INTERVAL_SEC   = 10.0        # real seconds between position signals
MAX_ATTEMPTS = 3

# How long to sleep at the end of each loop iteration (seconds).
# Keeps CPU usage sane without blocking ROS2 callbacks.
LOOP_SLEEP_SEC             = 0.1        # 10 Hz — plenty of resolution


from enum import Enum, auto

class RobotState(Enum):
    """
    Merged state machine.

    Model layer
    ───────────
    MONITORING   healthy polling; branches on battery level every loop
    BATTERY_OK   battery confirmed OK; stays BATTERY_OK_DURATION_SEC, then
                 → MONITORING
    BATTERY_LOW  threshold crossed; raises energy_low; relay → CANCEL_TASK

    Shutdown chain
    ──────────────
    CANCEL_TASK   calls cancelTask(), instantly → SETTLING
    SETTLING      publishes zero-vel for SETTLE_DURATION_SEC, then
                  → CONFIRM_STOP
    CONFIRM_STOP  polls isTaskComplete() for up to CONFIRM_DURATION_SEC, then
                  → SHUTDOWN_NAV
    SHUTDOWN_NAV  async shutdown of each lifecycle manager, then → WAIT_PICKUP
    WAIT_PICKUP   raises Wifi·send_position every WAIT_PICKUP_INTERVAL_SEC
                  (self-loop: state stays, timer resets)
    """
    MONITORING    = auto()
    BATTERY_OK    = auto()
    BATTERY_LOW   = auto()
    CANCEL_TASK   = auto()
    SETTLING      = auto()
    CONFIRM_STOP  = auto()
    SHUTDOWN_NAV  = auto()
    WAIT_PICKUP   = auto()


class BatteryMonitor(Node):

    def __init__(self, with_exploration_node: bool = False):
        super().__init__('battery_monitor')

        self._with_exploration_node = with_exploration_node
        self._abort_attempts: int = 0

        # ── Lifecycle manager discovery ────────────────────────────────────
        self._available_managers: list[str] = []
        self._detect_lifecycle_managers()

        # ── Battery simulation ─────────────────────────────────────────────
        self.current_charge   = INITIAL_CHARGE_PERCENTAGE
        self.is_moving        = False
        self._last_drain_time = self.get_clock().now()

        # ── FSM ───────────────────────────────────────────────────────────
        self._state              = RobotState.MONITORING
        # Timestamp of when we entered the current state.
        # _transition_to() always updates this.
        self._state_entered_time: Time = self.get_clock().now()

        # Outstanding async future (SHUTDOWN_NAV)
        self._pending_future = None
        self._pending_client = None

        # ── Robot pose ────────────────────────────────────────────────────
        self.current_pose = Pose()
        self.current_pose.orientation.w = 1.0

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel',
                                 self._cmd_vel_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose',
                                 self._pose_callback, 10)
        if not self._with_exploration_node:
            self.create_subscription(
                GoalStatusArray,
                '/navigate_to_pose/_action/status',
                self._nav_status_callback,
                10
            )
        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_vel_pub            = self.create_publisher(Twist, '/cmd_vel', 1)
        self.emergency_position_pub = self.create_publisher(
            PoseStamped, '/Wifi/send_position', 10)
        self.emergency_status_pub   = self.create_publisher(String, '/status', 10)

        # ── Navigator ─────────────────────────────────────────────────────
        self.navigator = BasicNavigator()

        self.get_logger().info(
            f'🔋 BatteryMonitor ready (while-True loop, no timers)\n'
            f'   charge={self.current_charge:.1f}%  '
            f'threshold={LOW_BATTERY_THRESHOLD:.1f}%\n'
            f'   loop rate ≈ {1/LOOP_SLEEP_SEC:.0f} Hz  |  '
            f'position signal every {WAIT_PICKUP_INTERVAL_SEC:.0f}s\n'
            f'   with_exploration_node={self._with_exploration_node}'
        )

    # ──────────────────────────────────────────────────────────────────────
    #  Main loop  (called from main())
    # ──────────────────────────────────────────────────────────────────────

    def run(self) -> None:
        """
        Owns the thread.  Runs until rclpy shuts down or KeyboardInterrupt.

        Sequence each iteration:
          1. Drain ROS2 callback queue (subscriptions, service responses).
          2. Update battery charge.
          3. Advance the FSM.
          4. Short sleep to avoid burning 100% CPU.
        """
        self.get_logger().info('▶  Main loop started.')
        while rclpy.ok():
            # 1 ── ROS2 callbacks ──────────────────────────────────────────
            # timeout_sec=0 → non-blocking; returns immediately if nothing queued.
            rclpy.spin_once(self, timeout_sec=0.0)

            # 2 ── Battery drain ───────────────────────────────────────────
            self._drain_battery()

            # 3 ── FSM ─────────────────────────────────────────────────────
            self._tick_fsm()

            # 4 ── Yield ───────────────────────────────────────────────────
            time.sleep(LOOP_SLEEP_SEC)

    # ──────────────────────────────────────────────────────────────────────
    #  Battery drain  (real-time delta, not tick-based)
    # ──────────────────────────────────────────────────────────────────────

    def _drain_battery(self) -> None:
        now  = self.get_clock().now()
        dt   = (now - self._last_drain_time).nanoseconds / 1e9
        self._last_drain_time = now

        rate = MOVEMENT_DRAIN_PER_SECOND if self.is_moving else IDLE_DRAIN_PER_SECOND
        self.current_charge = max(0.0, self.current_charge - rate * dt)

    # ──────────────────────────────────────────────────────────────────────
    #  State-transition helper
    # ──────────────────────────────────────────────────────────────────────

    def _transition_to(self, new_state: RobotState) -> None:
        """
        Change state and record the entry timestamp.
        Every timed guard computes elapsed = now - _state_entered_time.
        """
        self.get_logger().info(
             f'[FSM] {self._state.name} → {new_state.name}'
        )
        self._state              = new_state
        self._state_entered_time = self.get_clock().now()

    def _time_in_state(self) -> float:
        """Seconds elapsed since we entered the current state."""
        return (
            self.get_clock().now() - self._state_entered_time
        ).nanoseconds / 1e9

    # ──────────────────────────────────────────────────────────────────────
    #  FSM dispatcher
    # ──────────────────────────────────────────────────────────────────────

    def _tick_fsm(self) -> None:
        s = self._state

        if   s == RobotState.MONITORING:    self._state_monitoring()
        elif s == RobotState.BATTERY_OK:    self._state_battery_ok()
        elif s == RobotState.BATTERY_LOW:   self._state_battery_low()
        elif s == RobotState.CANCEL_TASK:   self._state_cancel_task()
        elif s == RobotState.SETTLING:      self._state_settling()
        elif s == RobotState.CONFIRM_STOP:  self._state_confirm_stop()
        elif s == RobotState.SHUTDOWN_NAV:  self._state_shutdown_nav()
        elif s == RobotState.WAIT_PICKUP:   self._state_wait_pickup()

    # ──────────────────────────────────────────────────────────────────────
    #  State handlers
    # ──────────────────────────────────────────────────────────────────────

    def _state_monitoring(self) -> None:
        """
        Healthy polling.
        Transitions are instant — no time guard needed here.
        """
        if self.current_charge > LOW_BATTERY_THRESHOLD:
            self.get_logger().info(
                f'🔋 Battery: {self.current_charge:.1f}% | Moving: {self.is_moving}'
            )
            self._transition_to(RobotState.BATTERY_OK)
        else:
            self.get_logger().warn(
                f'⚠️  LOW BATTERY ({self.current_charge:.1f}%) '
                f'— raising energy_low'
            )
            self._raise_energy_low()
            self._transition_to(RobotState.BATTERY_LOW)

    def _state_battery_ok(self) -> None:
        """
        1-second confirmation that the battery reading is healthy.
        Guard: stay here for BATTERY_OK_DURATION_SEC real seconds,
        then return to MONITORING.
        """
        if self._time_in_state() >= BATTERY_OK_DURATION_SEC:
            self._transition_to(RobotState.MONITORING)

    def _state_battery_low(self) -> None:
        """
        Relay state.
        • with_exploration_node=False (default) → relay instantly to CANCEL_TASK.
        We own the shutdown chain.
        • with_exploration_node=True → do nothing and stay here.
        The exploration node is responsible for stopping navigation.
        """
        if self._with_exploration_node:
            return
        self._transition_to(RobotState.CANCEL_TASK)

    def _state_cancel_task(self) -> None:
        """Instant: issue cancelTask(), publish zero-vel, → SETTLING."""
        self.get_logger().info('[FSM] CANCEL_TASK — calling cancelTask()')
        try:
            self.navigator.cancelTask()
        except Exception as exc:
            self.get_logger().info(f'[FSM] cancelTask() note: {exc}')
        self._publish_zero_velocity()
        self._transition_to(RobotState.SETTLING)

    def _state_settling(self) -> None:
        """
        Timed: keep publishing zero-vel for SETTLE_DURATION_SEC seconds,
        then → CONFIRM_STOP.
        """
        self._publish_zero_velocity()
        elapsed = self._time_in_state()
        self.get_logger().debug(
            f'[FSM] SETTLING {elapsed:.2f}/{SETTLE_DURATION_SEC}s'
        )
        if elapsed >= SETTLE_DURATION_SEC:
            self._transition_to(RobotState.CONFIRM_STOP)

    def _state_confirm_stop(self) -> None:
        """
        Timed + conditional:
          • If isTaskComplete() → proceed now.
          • If CONFIRM_DURATION_SEC elapsed without confirmation → proceed anyway.
        """
        self._publish_zero_velocity()
        if self.navigator.isTaskComplete():
            self.get_logger().info('[FSM] CONFIRM_STOP — task complete confirmed')
            self._enter_shutdown_nav()
            return

        elapsed = self._time_in_state()
        self.get_logger().debug(
            f'[FSM] CONFIRM_STOP {elapsed:.2f}/{CONFIRM_DURATION_SEC}s'
        )
        if elapsed >= CONFIRM_DURATION_SEC:
            self.get_logger().warn('[FSM] CONFIRM_STOP — timed out, continuing')
            self._enter_shutdown_nav()

    def _state_shutdown_nav(self) -> None:
        """
        Poll the outstanding async future.
        When done, move to the next manager or → WAIT_PICKUP.
        Runs every loop iteration — fast polling, no blocking.
        """
        if self._pending_future is None:
            # Safety guard.
            self._enter_wait_pickup()
            return

        if not self._pending_future.done():
            self.get_logger().debug('[FSM] SHUTDOWN_NAV — awaiting response…')
            return

        # Future resolved.
        service_name = self._available_managers.pop(0)
        self._pending_client.destroy()
        self._pending_client = None

        if self._pending_future.result() is not None:
            self.get_logger().info(f'[FSM] ✅  {service_name} shut down')
        else:
            self.get_logger().error(f'[FSM] ❌  {service_name} — no result')

        self._pending_future = None

        if self._available_managers:
            self._issue_next_shutdown()
        else:
            self._enter_wait_pickup()

    def _state_wait_pickup(self) -> None:
        """
        WAIT_PICKUP self-loop (model spec):
        after WAIT_PICKUP_INTERVAL_SEC seconds → raise Wifi·send_position
                                                → transition back to WAIT_PICKUP
        """
        if self._time_in_state() >= WAIT_PICKUP_INTERVAL_SEC:
            self._raise_wifi_send_position()
            self._transition_to(RobotState.WAIT_PICKUP)

    # ──────────────────────────────────────────────────────────────────────
    #  SHUTDOWN_NAV helpers
    # ──────────────────────────────────────────────────────────────────────

    def _enter_shutdown_nav(self) -> None:
        if not self._available_managers:
            self.get_logger().info('[FSM] No lifecycle managers — skip SHUTDOWN_NAV')
            self._enter_wait_pickup()
            return
        self._issue_next_shutdown()
        self._transition_to(RobotState.SHUTDOWN_NAV)    

    def _issue_next_shutdown(self) -> None:
        if not self._available_managers:
            self._enter_wait_pickup()
            return

        service_name = self._available_managers[0]
        self.get_logger().info(f'[FSM] → shutdown request: {service_name}')

        client = self.create_client(ManageLifecycleNodes, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'[FSM] {service_name} unavailable — skipping')
            client.destroy()
            self._available_managers.pop(0)
            self._issue_next_shutdown()
            return

        req         = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.SHUTDOWN
        self._pending_future = client.call_async(req)
        self._pending_client = client

    def _enter_wait_pickup(self) -> None:
        self._publish_zero_velocity()
        self._transition_to(RobotState.WAIT_PICKUP)
        # Arm the interval timer: first signal fires WAIT_PICKUP_INTERVAL_SEC
        # after entry, consistent with the model's self-loop.
        self._last_position_signal_time = self.get_clock().now()
        self.get_logger().warn(
            '🛑 WAIT_PICKUP — nav stack DOWN.\n'
            f'  First position signal in {WAIT_PICKUP_INTERVAL_SEC:.0f}s.'
        )

    # ──────────────────────────────────────────────────────────────────────
    #  Signals
    # ──────────────────────────────────────────────────────────────────────

    def _raise_energy_low(self) -> None:
        msg      = String()
        msg.data = json.dumps({
            'signal':             'energy_low',
            'battery_percentage': round(self.current_charge, 2),
            'timestamp':          self.get_clock().now().nanoseconds / 1e9,
        })
        self.emergency_status_pub.publish(msg)
        self.get_logger().warn(
            f'📡 energy_low raised (battery={self.current_charge:.1f}%)'
        )

    def _raise_wifi_send_position(self) -> None:
        ps                 = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose            = self.current_pose
        self.emergency_position_pub.publish(ps)

        msg      = String()
        msg.data = json.dumps({
            'signal':             'Wifi_send_position',
            'timestamp':          self.get_clock().now().nanoseconds / 1e9,
            'battery_percentage': round(self.current_charge, 2),
            'position': {
                'x': round(self.current_pose.position.x, 3),
                'y': round(self.current_pose.position.y, 3),
                'z': round(self.current_pose.position.z, 3),
            },
            'orientation': {
                'x': round(self.current_pose.orientation.x, 3),
                'y': round(self.current_pose.orientation.y, 3),
                'z': round(self.current_pose.orientation.z, 3),
                'w': round(self.current_pose.orientation.w, 3),
            },
        })
        self.emergency_status_pub.publish(msg)

        self.get_logger().info(
            f'📍 Wifi·send_position raised | '
            f'({self.current_pose.position.x:.2f}, '
            f'{self.current_pose.position.y:.2f}) | '
            f'battery={self.current_charge:.1f}%'
        )

    # ──────────────────────────────────────────────────────────────────────
    #  Helpers
    # ──────────────────────────────────────────────────────────────────────

    def _publish_zero_velocity(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    # ──────────────────────────────────────────────────────────────────────
    #  Subscriptions
    # ──────────────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.is_moving = (msg.linear.x != 0.0 or msg.angular.z != 0.0)

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.current_pose = msg.pose.pose
    
    def _nav_status_callback(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status

        
        if (latest_status == GoalStatus.STATUS_ABORTED):

            if self._state not in (RobotState.CANCEL_TASK, RobotState.SETTLING,
                                RobotState.CONFIRM_STOP, RobotState.SHUTDOWN_NAV,
                                RobotState.WAIT_PICKUP):
                self._abort_attempts += 1
                self.get_logger().warn(
                    f'[FSM] Task aborted — attempt {self._abort_attempts}/{MAX_ATTEMPTS}'
                )
                if self._abort_attempts >= MAX_ATTEMPTS:
                    self.get_logger().warn('[FSM] Max aborts reached → CANCEL_TASK')
                    self._abort_attempts = 0
                    self._transition_to(RobotState.CANCEL_TASK)

        
        if (latest_status == GoalStatus.STATUS_SUCCEEDED):
            self.get_logger().info('[FSM] Goal execution succeeded — resetting abort counter')
            self._abort_attempts = 0


    # ──────────────────────────────────────────────────────────────────────
    #  Lifecycle-manager discovery
    # ──────────────────────────────────────────────────────────────────────

    def _detect_lifecycle_managers(self) -> None:
        candidates = [
            '/lifecycle_manager_navigation/manage_nodes',
            '/lifecycle_manager_localization/manage_nodes',
            '/lifecycle_manager/manage_nodes',
        ]
        self.get_logger().info('[DETECT] Probing lifecycle managers…')
        for name in candidates:
            client = self.create_client(ManageLifecycleNodes, name)
            found  = client.wait_for_service(timeout_sec=2.0)
            client.destroy()
            if found:
                self._available_managers.append(name)
                self.get_logger().info(f'[DETECT] ✅  {name}')
            else:
                self.get_logger().info(f'[DETECT] ➖  {name}')
        if self._available_managers:
            self.get_logger().info(
                f'[DETECT] {len(self._available_managers)} manager(s) found.')
        else:
            self.get_logger().warn('[DETECT] No lifecycle managers found.')


# ──────────────────────────────────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    parser = argparse.ArgumentParser(description='Battery Monitor Node')
    parser.add_argument(
        '--with_exploration_node',
        action='store_true',          # present → True, absent → False
        default=False,
        help='If set, BATTERY_LOW will NOT relay to CANCEL_TASK '
             '(exploration node handles shutdown itself).'
    )

    our_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = BatteryMonitor(with_exploration_node=our_args.with_exploration_node)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()