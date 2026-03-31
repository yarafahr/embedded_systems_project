#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import Twist, PoseStamped, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json
import time
import threading

# --- Configuration ---
INITIAL_CHARGE_PERCENTAGE  = 100.0
LOW_BATTERY_THRESHOLD      = 20.0
IDLE_DRAIN_PER_SECOND      = 0.05
MOVEMENT_DRAIN_PER_SECOND  = 0.5
POSITION_PUBLISH_INTERVAL  = 10.0


class BatteryMonitor(Node):
    """
    Battery Monitor Node
    Simulates battery drain, stops Nav2 when low, and publishes location for rescue.
    """
    def __init__(self):
        super().__init__('battery_monitor')

        # ── Callback groups ───────────────────────────────────────────────
        self._default_cbg  = MutuallyExclusiveCallbackGroup()
        self._sequence_cbg = MutuallyExclusiveCallbackGroup()

        # ── Auto-detect lifecycle managers ───────────────────────────────
        self._available_managers = []
        self._detect_lifecycle_managers()

        # --- Battery Simulation ---
        self.current_charge        = INITIAL_CHARGE_PERCENTAGE
        self.is_moving             = False
        self.last_update_time      = self.get_clock().now()
        self.low_battery_triggered = False

        # --- Robot Position (from AMCL / SLAM) ---
        self.current_pose                  = Pose()
        self.current_pose.position.x       = 0.0
        self.current_pose.position.y       = 0.0
        self.current_pose.position.z       = 0.0
        self.current_pose.orientation.w    = 1.0

        # --- Subscribers ---
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
            callback_group=self._default_cbg,
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self._pose_callback,
            10,
            callback_group=self._default_cbg,
        )

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.emergency_position_pub = self.create_publisher(
            PoseStamped, '/Wifi/send_position', 10
        )

        self.emergency_status_pub = self.create_publisher(
            String, '/status', 10
        )

        # --- Navigator ---
        self.navigator = BasicNavigator()

        # --- Timers ---
        self.battery_timer = self.create_timer(
            1.0,
            self._update_callback,
            callback_group=self._default_cbg,
        )

        self.position_timer = None  # created after low-battery sequence

        self.get_logger().info(
            f'🔋 Battery Monitor started\n'
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
                self.get_logger().info(f'[DETECT] ✅ Found: {service_name}')
            else:
                self.get_logger().info(f'[DETECT] ➖ Not found: {service_name}')

        if not self._available_managers:
            self.get_logger().warn(
                '[DETECT] No lifecycle managers found! '
                'Will rely on cancelTask() only.'
            )
        else:
            self.get_logger().info(
                f'[DETECT] Will shut down {len(self._available_managers)} '
                f'manager(s) on low battery.'
            )

    # ------------------------------------------------------------------ #
    #  Subscribers                                                         #
    # ------------------------------------------------------------------ #
    def _cmd_vel_callback(self, msg: Twist):
        self.is_moving = (msg.linear.x != 0.0 or msg.angular.z != 0.0)

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    # ------------------------------------------------------------------ #
    #  Battery update (1 Hz timer)                                         #
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
            f'🔋 Battery: {self.current_charge:.1f}% | Moving: {self.is_moving}'
        )

        if self.current_charge <= LOW_BATTERY_THRESHOLD and not self.low_battery_triggered:
            self.low_battery_triggered = True
            self.get_logger().warn(
                f'⚠️  LOW BATTERY TRIGGERED: {self.current_charge:.1f}%'
            )
            threading.Thread(
                target=self._low_battery_sequence,
                daemon=True,
            ).start()

    # ------------------------------------------------------------------ #
    #  Position publishing                                                 #
    # ------------------------------------------------------------------ #
    def _start_position_timer(self):
        if self.position_timer is None:
            self.position_timer = self.create_timer(
                POSITION_PUBLISH_INTERVAL,
                self._publish_position_callback,
                callback_group=self._default_cbg,
            )
            self.get_logger().info(
                f'[BATTERY] 📡 Position timer started '
                f'(every {POSITION_PUBLISH_INTERVAL:.0f}s).'
            )

    def _publish_position_callback(self):
        self.get_logger().info('[BATTERY] 📡 Position callback called')

        pose_stamped                  = PoseStamped()
        pose_stamped.header.stamp     = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id  = 'map'
        pose_stamped.pose             = self.current_pose

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
                'z': round(self.current_pose.orientation.z, 3),
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
    #  Low-battery sequence  (background thread)                           #
    # ------------------------------------------------------------------ #
    def _low_battery_sequence(self):
        # ── Step 1: Cancel active navigation task ─────────────────────────
        self.get_logger().info('[BATTERY] Step 1/3 — Cancelling active task...')
        try:
            self.navigator.cancelTask()
            self.get_logger().info('[BATTERY] cancelTask() sent.')
            self._publish_zero_velocity()
        except Exception as exc:
            self.get_logger().info(f'[BATTERY] cancelTask() note: {exc}')

        # ── Settle delay ───────────────────────────────────────────────────
        self.get_logger().info('[BATTERY] Waiting 5 s for robot to settle...')
        time.sleep(5.0)
        self.get_logger().info('[BATTERY] Settle delay complete.')

        # ── Step 2: Confirm task is complete ──────────────────────────────
        self._publish_zero_velocity()
        self.get_logger().info('[BATTERY] Step 2/3 — Confirming task stopped...')
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._publish_zero_velocity()
            if self.navigator.isTaskComplete():
                self.get_logger().info('[BATTERY] Task confirmed complete.')
                break
            time.sleep(0.2)
        else:
            self.get_logger().warn(
                '[BATTERY] Task did not confirm completion within 5 s — continuing anyway.'
            )

        # ── Step 3: Shut down available lifecycle managers ─────────────────
        self.get_logger().info(
            f'[BATTERY] Step 3/3 — Shutting down '
            f'{len(self._available_managers)} lifecycle manager(s)...'
        )
        for manager_service in self._available_managers:
            self._shutdown_lifecycle_manager(manager_service)

        self._publish_zero_velocity()
        self.get_logger().warn(
            '🛑 [BATTERY] SHUTDOWN COMPLETE.\n'
            '   • Nav2 stack is DOWN\n'
            f'   • Publishing position every {POSITION_PUBLISH_INTERVAL:.0f}s for rescue.'
        )

        # ── Publish once immediately, then start the recurring timer ───────
        self._publish_position_callback()
        self._start_position_timer()

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #
    def _publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def _shutdown_lifecycle_manager(self, service_name: str):
        self.get_logger().info(f'[BATTERY] Shutting down {service_name}...')

        client = self.create_client(ManageLifecycleNodes, service_name)

        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                f'[BATTERY] {service_name} not available — skipping.'
            )
            client.destroy()
            return

        req         = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.SHUTDOWN

        future = client.call_async(req)

        # ── Poll until done — DO NOT use rclpy.spin_until_future_complete. ──
        # spin_until_future_complete() hijacks the node's executor from this  
        # background thread, corrupting its internal state and preventing any  
        # timers created afterwards from ever firing.                          
        # The MultiThreadedExecutor on the main thread drives the future;      
        # we just sleep-poll until it is resolved.                             
        deadline = time.time() + 5.0
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error(
                    f'[BATTERY] ❌ {service_name} shutdown timed out.'
                )
                client.destroy()
                return
            time.sleep(0.05)

        client.destroy()

        if future.result() is not None:
            self.get_logger().info(
                f'[BATTERY] ✅ {service_name} shutdown complete.'
            )
        else:
            self.get_logger().error(
                f'[BATTERY] ❌ {service_name} shutdown returned no result.'
            )


# --------------------------------------------------------------------------- #
#  Entry point                                                                 #
# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    executor = MultiThreadedExecutor()
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