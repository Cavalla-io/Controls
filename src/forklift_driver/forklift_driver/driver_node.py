import rclpy
from rclpy.node import Node
from forklift_msgs.msg import ForkliftDirectCommand
from .can_interface.mbv15_iface import MBV15Interface

DEFAULT_ACCEL_S = 1.0
DEFAULT_DECEL_S = 0.2


class ForkliftDriverNode(Node):
    def __init__(self):
        super().__init__('forklift_driver')

        self.curtis = MBV15Interface(channel='can0', bitrate=250000)
        if self.curtis.connected:
            self.get_logger().info("Connected to MBV15/Curtis Controller on can0")
        else:
            self.get_logger().warn("CAN0 not found. Running in MOCK mode.")

        self.create_subscription(
            ForkliftDirectCommand, '/safe/raw_command', self.command_callback, 1
        )

    def command_callback(self, msg: ForkliftDirectCommand):
        accel = msg.accel_time_s if msg.accel_time_s > 0.0 else DEFAULT_ACCEL_S
        decel = msg.decel_time_s if msg.decel_time_s > 0.0 else DEFAULT_DECEL_S

        self.curtis.send_commands(
            msg.drive_speed,
            msg.steering_angle,
            msg.lift_speed,
            msg.tilt_speed,
            msg.side_shift_speed,
            accel_s=accel,
            decel_s=decel,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down driver. Commanding STOP.")
        node.curtis.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
