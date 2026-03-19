import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

# Must match operator_relay / widget JSON (space in key).
SELECTED_TARGET_HEIGHT_KEY = "selected target height"


class OperatorForkHeightBridge(Node):
    def __init__(self):
        super().__init__("operator_fork_height_bridge")

        self.declare_parameter("master_topic", "/master_remop_message")
        self.declare_parameter("target_topic", "/forklift/target_fork_height")
        self.declare_parameter("republish_rate_hz", 20.0)

        master_topic = self.get_parameter("master_topic").get_parameter_value().string_value
        target_topic = self.get_parameter("target_topic").get_parameter_value().string_value
        rate_hz = self.get_parameter("republish_rate_hz").get_parameter_value().double_value
        if rate_hz <= 0.0:
            rate_hz = 20.0

        self._target_m: float | None = None
        self._target_pub = self.create_publisher(Float32, target_topic, 10)
        self.create_subscription(String, master_topic, self._on_master, 10)

        period = 1.0 / rate_hz
        self.create_timer(period, self._republish_target)

        self.get_logger().info(
            f"operator_fork_height_bridge: {master_topic} -> {target_topic} at {rate_hz} Hz (mm -> m)"
        )

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
