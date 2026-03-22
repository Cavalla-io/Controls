import json

import rclpy
from rclpy.node import Node
from forklift_msgs.msg import ForkliftDirectCommand
from std_msgs.msg import Float32, String

# Must match operator_relay / widget JSON (space in key).
SELECTED_TARGET_HEIGHT_KEY = "selected target height"


class OperatorForkHeightBridge(Node):
    def __init__(self):
        super().__init__("operator_fork_height_bridge")

        self.declare_parameter("master_topic", "/master_remop_message")
        self.declare_parameter("target_topic", "/forklift/target_fork_height")
        self.declare_parameter("republish_rate_hz", 20.0)
        self.declare_parameter("teleop_topic", "/teleop/raw_command")
        self.declare_parameter("fork_activity_threshold", 0.01)

        master_topic = self.get_parameter("master_topic").get_parameter_value().string_value
        target_topic = self.get_parameter("target_topic").get_parameter_value().string_value
        rate_hz = self.get_parameter("republish_rate_hz").get_parameter_value().double_value
        if rate_hz <= 0.0:
            rate_hz = 20.0

        teleop_topic = self.get_parameter("teleop_topic").get_parameter_value().string_value
        self._fork_threshold = self.get_parameter(
            "fork_activity_threshold"
        ).get_parameter_value().double_value
        if self._fork_threshold <= 0.0:
            self._fork_threshold = 0.01

        self._target_m: float | None = None
        self._target_pub = self.create_publisher(Float32, target_topic, 10)
        self.create_subscription(String, master_topic, self._on_master, 10)
        self.create_subscription(
            ForkliftDirectCommand, teleop_topic, self._on_teleop, 10
        )

        period = 1.0 / rate_hz
        self.create_timer(period, self._republish_target)

        self.get_logger().info(
            f"operator_fork_height_bridge: {master_topic} -> {target_topic} at {rate_hz} Hz (mm -> m); "
            f"clear target on fork motion ({teleop_topic})"
        )

    def _is_fork_teleop_active(self, msg: ForkliftDirectCommand) -> bool:
        t = self._fork_threshold
        return (
            abs(msg.lift_speed) > t
            or abs(msg.tilt_speed) > t
            or abs(msg.side_shift_speed) > t
            or abs(msg.fork_spread_speed) > t
        )

    def _on_teleop(self, msg: ForkliftDirectCommand):
        if not self._is_fork_teleop_active(msg):
            return
        if self._target_m is None:
            return
        self.get_logger().info(
            "Teleop fork motion: clearing latched height target (manual override)."
        )
        self._target_m = None

    def _on_master(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Invalid JSON on master message")
            return

        if SELECTED_TARGET_HEIGHT_KEY not in data:
            return

        raw = data[SELECTED_TARGET_HEIGHT_KEY]
        try:
            mm = float(raw)
        except (TypeError, ValueError):
            self.get_logger().warning(
                f'Non-numeric "{SELECTED_TARGET_HEIGHT_KEY}": {raw!r}'
            )
            return

        if mm <= 0.0:
            return

        self._target_m = mm * 0.001
        self.get_logger().info(
            f"Target {mm:.3f} mm -> {self._target_m:.6f} m"
        )

    def _republish_target(self):
        if self._target_m is None or self._target_m <= 0.0:
            return
        msg = Float32()
        msg.data = float(self._target_m)
        self._target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorForkHeightBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
