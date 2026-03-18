import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32
from forklift_msgs.msg import ForkliftDirectCommand

from forklift_config import load_controls_config


class ForkliftSafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")

        cfg = load_controls_config()
        self._safety_cfg = cfg.safety
        topics = self._safety_cfg.ros_topics

        self.heartbeat_timeout_sec = self._safety_cfg.heartbeat_timeout_sec
        self.command_timeout_sec = self._safety_cfg.command_timeout_sec
        self.AUTO_TIMEOUT_SEC = self._safety_cfg.auto_timeout_sec
        self.TELEOP_PRIORITY_TIMEOUT = self._safety_cfg.teleop_priority_timeout_sec
        self.ACTIVITY_THRESHOLD = self._safety_cfg.teleop_activity_threshold

        self.last_heartbeat_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        self.latest_teleop_cmd = ForkliftDirectCommand()
        self.latest_auto_effort = 0.0
        self.last_auto_msg_time = self.get_clock().now()

        self.auto_fork_canceled = False
        self.last_teleop_activity_time = self.get_clock().now()

        self.current_status_code = 3
        self.was_safe = False

        self.create_subscription(
            ForkliftDirectCommand, topics.teleop_raw_command, self.teleop_cb, 1
        )
        self.create_subscription(UInt8, topics.safety, self.heartbeat_cb, 1)
        self.create_subscription(
            Float32, topics.auto_lift_effort, self.auto_effort_cb, 10
        )
        self.create_subscription(
            Float32, topics.target_fork_height, self.auto_target_cb, 1
        )

        self.safe_pub = self.create_publisher(
            ForkliftDirectCommand, topics.safe_raw_command, 1
        )

        self.create_timer(self._safety_cfg.watchdog_period_sec, self.watchdog_loop)

        self.get_logger().info("Safety Multiplexer Initialized. Awaiting Heartbeat...")

    def heartbeat_cb(self, msg: UInt8):
        self.current_status_code = msg.data
        self.last_heartbeat_time = self.get_clock().now()

    def auto_effort_cb(self, msg: Float32):
        self.latest_auto_effort = msg.data
        self.last_auto_msg_time = self.get_clock().now()

    def auto_target_cb(self, msg: Float32):
        if self.auto_fork_canceled:
            self.get_logger().info(
                "New Auto Target Received: Resuming Auto Fork control."
            )
            self.auto_fork_canceled = False

    def teleop_cb(self, msg: ForkliftDirectCommand):
        self.latest_teleop_cmd = msg
        self.last_cmd_time = self.get_clock().now()

        th = self.ACTIVITY_THRESHOLD
        is_driving = (
            abs(msg.drive_speed) > th or abs(msg.steering_angle) > th
        )
        is_forking = (
            abs(msg.lift_speed) > th
            or abs(msg.tilt_speed) > th
            or abs(msg.side_shift_speed) > th
            or abs(msg.fork_spread_speed) > th
        )

        if is_driving or is_forking:
            self.last_teleop_activity_time = self.get_clock().now()
            if is_forking and not self.auto_fork_canceled:
                self.get_logger().warn(
                    "Teleop Fork Intervention: Canceling Auto Fork until new target."
                )
                self.auto_fork_canceled = True

    def check_teleop_safety(self) -> tuple[bool, str]:
        now = self.get_clock().now()
        time_since_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

        is_safe = True
        reason = ""

        if self.current_status_code in self._safety_cfg.unsafe_status_codes:
            is_safe = False
            reason = (
                f"Status Code {self.current_status_code} (Unfocused/Disconnected)"
            )
        elif time_since_heartbeat > self.heartbeat_timeout_sec:
            is_safe = False
            reason = f"Heartbeat Stale ({time_since_heartbeat:.2f}s)"
        elif time_since_cmd > self.command_timeout_sec:
            is_safe = False
            reason = f"Command Stale ({time_since_cmd:.2f}s)"

        return is_safe, reason

    def check_automation_safety(self) -> tuple[bool, str]:
        now = self.get_clock().now()
        time_since_auto = (now - self.last_auto_msg_time).nanoseconds / 1e9

        is_safe = True
        reason = ""

        if time_since_auto > self.AUTO_TIMEOUT_SEC:
            is_safe = False
            reason = f"Auto Stale ({time_since_auto:.2f}s)"

        return is_safe, reason

    def watchdog_loop(self):
        now = self.get_clock().now()
        time_since_activity = (
            now - self.last_teleop_activity_time
        ).nanoseconds / 1e9

        teleop_safe, teleop_reason = self.check_teleop_safety()
        auto_safe, auto_reason = self.check_automation_safety()

        mux_cmd = ForkliftDirectCommand()
        is_teleop_mode = time_since_activity < self.TELEOP_PRIORITY_TIMEOUT

        is_global_safe = True
        fault_reason = ""

        if is_teleop_mode:
            if teleop_safe:
                mux_cmd = self.latest_teleop_cmd
            else:
                is_global_safe = False
                fault_reason = f"TELEOP SAFETY TRIP: {teleop_reason}"
        else:
            if auto_safe:
                if not self.auto_fork_canceled:
                    mux_cmd.lift_speed = self.latest_auto_effort
                else:
                    mux_cmd.lift_speed = 0.0
            else:
                is_global_safe = False
                fault_reason = f"AUTO SAFETY TRIP: {auto_reason}"

        if is_global_safe:
            if not self.was_safe:
                self.get_logger().info("SYSTEM SAFE. Passing commands to hardware.")
                self.was_safe = True
            self.safe_pub.publish(mux_cmd)
        else:
            if self.was_safe:
                self.get_logger().warn(
                    f"SAFETY TRIP! Reason: {fault_reason}. Clamping to 0.0."
                )
                self.was_safe = False
            stop_cmd = ForkliftDirectCommand()
            self.safe_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
