#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import Twist, PoseStamped, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json
import time
from enum import Enum, auto


# --- Configuration ---
INITIAL_CHARGE_PERCENTAGE  = 100.0
LOW_BATTERY_THRESHOLD      = 20.0
IDLE_DRAIN_PER_SECOND      = 0.05
MOVEMENT_DRAIN_PER_SECOND  = 0.5
POSITION_PUBLISH_INTERVAL  = 10.0

# How many 1-Hz ticks to wait for the robot to settle after cancelTask()
SETTLE_TICKS               = 5
# How many 1-Hz ticks to wait for isTaskComplete() confirmation
CONFIRM_TICKS              = 5


class ShutdownStage(Enum):
    """
    Stages of the low-battery shutdown sequence.
    The battery timer advances through these on each 1-Hz tick,
    keeping everything on the single executor thread.
    """
    IDLE            = auto()   # Normal operation
    CANCEL_TASK     = auto()   # cancelTask() issued, begin settle countdown
    SETTLING        = auto()   # Waiting SETTLE_TICKS ticks
    CONFIRM_STOP    = auto()   # Polling isTaskComplete() for up to CONFIRM_TICKS ticks
    SHUTDOWN_NAV    = auto()   # Sending shutdown to each lifecycle manager in turn
    DONE            = auto()   # Shutdown complete; position timer is active


class BatteryMonitor(Node):
    """
    Battery Monitor Node — single-threaded version.

    All logic runs inside the SingleThreadedExecutor via the 1-Hz battery
    timer and the regular ROS2 subscriber callbacks.  No background threads,
    no time.sleep(), no spin_until_future_complete().

    The low-battery shutdown sequence is driven by a small state machine
    (ShutdownStage) that advances one step per timer tick, so the executor
    is never blocked and all subscriptions remain live throughout.
    """

    def __init__(self):
        super().__init__('battery_monitor')

        # ── Auto-detect lifecycle managers ───────────────────────────────
        self._available_managers: list[str] = []
        self._detect_lifecycle_managers()

        # ── Battery simulation ────────────────────────────────────────────
        self.current_charge   = INITIAL_CHARGE_PERCENTAGE
        self.is_moving        = False
        self.last_update_time = self.get_clock().now()

        # ── Shutdown state machine ────────────────────────────────────────
        self._stage               = ShutdownStage.IDLE
        self._settle_ticks_left   = 0
        self._confirm_ticks_left  = 0
        self._pending_future      = None              # outstanding call_async future
        self._pending_client      = None              # client for the pending future

        # ── Robot position (from AMCL / SLAM) ────────────────────────────
        self.current_pose = Pose()
        self.current_pose.orientation.w = 1.0

        # ── Subscribers ───────────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self._pose_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.emergency_position_pub = self.create_publisher(
            PoseStamped, '/Wifi/send_position', 10
        )
        self.emergency_status_pub = self.create_publisher(String, '/status', 10)

        # ── Navigator ─────────────────────────────────────────────────────
        self.navigator = BasicNavigator()

        # ── Timers ────────────────────────────────────────────────────────
        # Single 1-Hz timer drives both battery drain and the shutdown FSM.
        self.battery_timer = self.create_timer(1.0, self._update_callback)

        # Position publishing timer; created only after shutdown completes.
        self.position_timer = None

        self.get_logger().info(
            f'🔋 Battery Monitor started (single-threaded)\n'
            f'   Initial charge: {self.current_charge:.1f}%\n'
            f'   Low threshold:  {LOW_BATTERY_THRESHOLD:.1f}%\n'
            f'   Position publish interval: {POSITION_PUBLISH_INTERVAL}s'
        )

    # ------------------------------------------------------------------ #
    #  Lifecycle-manager detection                                         #
    # ------------------------------------------------------------------ #
    def _detect_lifecycle_managers(self):
        candidates = [
            '/lifecycle_manager_navigation/manage_nodes',
            '/lifecycle_manager_localization/manage_nodes',
            '/lifecycle_manager/manage_nodes',
        ]
        self.get_logger().info('[DETECT] Probing for lifecycle managers...')
        for service_name in candidates:
            client = self.create_client(ManageLifecycleNodes, service_name)
            found  = client.wait_for_service(timeout_sec=2.0)
            client.destroy()
            if found:
                self._available_managers.append(service_name)
                self.get_logger().info(f'[DETECT] ✅  Found: {service_name}')
            else:
                self.get_logger().info(f'[DETECT] ➖  Not found: {service_name}')

        if not self._available_managers:
            self.get_logger().warn('[DETECT] No lifecycle managers found.')
        else:
            self.get_logger().info(
                f'[DETECT] Will shut down {len(self._available_managers)} manager(s).'
            )

    # ------------------------------------------------------------------ #
    #  Subscribers                                                         #
    # ------------------------------------------------------------------ #
    def _cmd_vel_callback(self, msg: Twist):
        self.is_moving = (msg.linear.x != 0.0 or msg.angular.z != 0.0)

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    # ------------------------------------------------------------------ #
    #  1-Hz timer — battery drain + FSM tick                              #
    # ------------------------------------------------------------------ #
    def _update_callback(self):

        now = self.get_clock().now()
        dt  = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        drain = (
            MOVEMENT_DRAIN_PER_SECOND if self.is_moving else IDLE_DRAIN_PER_SECOND
        ) * dt
        self.current_charge = max(0.0, self.current_charge - drain)

        self.get_logger().info(
            f'🔋 Battery: {self.current_charge:.1f}% | Moving: {self.is_moving} '
            f'| Stage: {self._stage.name}'
        )

        # ── Trigger the shutdown FSM when threshold is crossed ─────────────
        if (
            self.current_charge <= LOW_BATTERY_THRESHOLD
            and self._stage == ShutdownStage.IDLE
        ):
            self.get_logger().warn(
                f'⚠️  LOW BATTERY: {self.current_charge:.1f}% — starting shutdown FSM'
            )
            self._stage = ShutdownStage.CANCEL_TASK

        # ── Advance the FSM ───────────────────────────────────────────────
        self._tick_fsm()

    # ------------------------------------------------------------------ #
    #  Shutdown FSM                                                        #
    # ------------------------------------------------------------------ #
    def _tick_fsm(self):
        stage = self._stage

        # ── CANCEL_TASK ───────────────────────────────────────────────────
        if stage == ShutdownStage.CANCEL_TASK:
            self.get_logger().info('[FSM] CANCEL_TASK — cancelling active navigation task')
            try:
                self.navigator.cancelTask()
            except Exception as exc:
                self.get_logger().info(f'[FSM] cancelTask() note: {exc}')
            self._publish_zero_velocity()
            self._settle_ticks_left = SETTLE_TICKS
            self._stage = ShutdownStage.SETTLING

        # ── SETTLING ──────────────────────────────────────────────────────
        elif stage == ShutdownStage.SETTLING:
            self._publish_zero_velocity()
            self._settle_ticks_left -= 1
            self.get_logger().info(
                f'[FSM] SETTLING — {self._settle_ticks_left} tick(s) remaining'
            )
            if self._settle_ticks_left <= 0:
                self._confirm_ticks_left = CONFIRM_TICKS
                self._stage = ShutdownStage.CONFIRM_STOP

        # ── CONFIRM_STOP ──────────────────────────────────────────────────
        elif stage == ShutdownStage.CONFIRM_STOP:
            self._publish_zero_velocity()
            if self.navigator.isTaskComplete():
                self.get_logger().info('[FSM] CONFIRM_STOP — task confirmed complete')
                self._begin_nav_shutdown()
                return

            self._confirm_ticks_left -= 1
            self.get_logger().info(
                f'[FSM] CONFIRM_STOP — waiting for task complete '
                f'({self._confirm_ticks_left} tick(s) left)'
            )
            if self._confirm_ticks_left <= 0:
                self.get_logger().warn(
                    '[FSM] CONFIRM_STOP — timed out, proceeding anyway'
                )
                self._begin_nav_shutdown()

        # ── SHUTDOWN_NAV ──────────────────────────────────────────────────
        elif stage == ShutdownStage.SHUTDOWN_NAV:
            self._tick_nav_shutdown()

        # ── DONE — nothing to do, position timer handles publishing ───────
        # (ShutdownStage.IDLE is also a no-op here)

    def _begin_nav_shutdown(self):
        if not self._available_managers:
            self.get_logger().info('[FSM] No lifecycle managers — skipping SHUTDOWN_NAV')
            self._finish_shutdown()
            return

        self._stage = ShutdownStage.SHUTDOWN_NAV
        self.get_logger().info(
            f'[FSM] SHUTDOWN_NAV — {len(self._available_managers)} manager(s) to shut down'
        )
        # Kick off the first manager immediately (don't wait a full tick).
        self._issue_next_shutdown()

    def _issue_next_shutdown(self):
        """
        Send an async shutdown request to the next lifecycle manager in the queue.
        The future is stored in self._pending_future; _tick_nav_shutdown() polls it.
        """
        if not self._available_managers:
            self._finish_shutdown()
            return

        service_name = self._available_managers[0]   # peek — pop after success
        self.get_logger().info(f'[FSM] Sending shutdown to {service_name}')

        client = self.create_client(ManageLifecycleNodes, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'[FSM] {service_name} not available — skipping')
            client.destroy()
            self._available_managers.pop(0)
            self._issue_next_shutdown()   # try next immediately
            return

        req         = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.SHUTDOWN

        self._pending_future = client.call_async(req)
        self._pending_client = client

    def _tick_nav_shutdown(self):


        if not self._pending_future.done():
            # Future not ready yet — check again next tick.
            self.get_logger().info('[FSM] SHUTDOWN_NAV — waiting for response...')
            return

        # Future is done — inspect result
        service_name = self._available_managers.pop(0)
        self._pending_client.destroy()
        self._pending_client = None

        if self._pending_future.result() is not None:
            self.get_logger().info(f'[FSM] ✅  {service_name} shutdown complete')
        else:
            self.get_logger().error(f'[FSM] ❌  {service_name} shutdown returned no result')

        self._pending_future = None

        if self._available_managers:
            self._issue_next_shutdown()
        else:
            self._finish_shutdown()

    def _finish_shutdown(self):
        self._publish_zero_velocity()
        self._stage = ShutdownStage.DONE
        self.get_logger().warn(
            '🛑 [FSM] SHUTDOWN COMPLETE.\n'
            '   • Nav2 stack is DOWN\n'
            f'  • Publishing position every {POSITION_PUBLISH_INTERVAL:.0f}s for rescue.'
        )
        # Publish once immediately, then arm the recurring timer.
        self._publish_position_callback()
        self.position_timer = self.create_timer(
            POSITION_PUBLISH_INTERVAL, self._publish_position_callback
        )

    # ------------------------------------------------------------------ #
    #  Position publishing                                                 #
    # ------------------------------------------------------------------ #
    def _publish_position_callback(self):
        pose_stamped                 = PoseStamped()
        pose_stamped.header.stamp    = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose            = self.current_pose
        self.emergency_position_pub.publish(pose_stamped)

        status_json = {
            'timestamp':          self.get_clock().now().nanoseconds / 1e9,
            'status':             'BATTERY_LOW',
            'battery_percentage': round(self.current_charge, 2),
            'position': {
                'x': round(self.current_pose.position.x, 3),
                'y': round(self.current_pose.position.y, 3),
                'z': round(self.current_pose.position.z, 3),
            },
            'orientation': {
                'x': round(self.current_pose.orientation.x, 3),
                'y': round(self.current_pose.orientation.y, 3),
                'z': round(self.current_pose.orientation.w, 3),
                'w': round(self.current_pose.orientation.w, 3),
            },
        }
        status_msg      = String()
        status_msg.data = json.dumps(status_json)
        self.emergency_status_pub.publish(status_msg)

        self.get_logger().info(
            f'📍 Position: ({self.current_pose.position.x:.2f}, '
            f'{self.current_pose.position.y:.2f}) | '
            f'🔋 Battery: {self.current_charge:.1f}%'
        )

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #
    def _publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())


# --------------------------------------------------------------------------- #
#  Entry point                                                                 #
# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
