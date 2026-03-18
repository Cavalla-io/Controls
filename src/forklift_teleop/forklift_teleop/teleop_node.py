import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from forklift_msgs.msg import ForkliftDirectCommand
from std_msgs.msg import String
import json
from rclpy.qos import qos_profile_sensor_data

from forklift_config import load_controls_config


class ForkliftTeleop(Node):
    def __init__(self):
        super().__init__('forklift_teleop')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(ForkliftDirectCommand, '/teleop/raw_command', 1)
        self.preset_pub = self.create_publisher(String, '/teleop/preset', 1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
        self.master_sub = self.create_subscription(String, '/master_remop_message', self.master_callback, 10)
        
        # State tracking for the Forward/Reverse toggle
        self.is_forward_gear = True
        self.last_a_button_state = 0

        cfg = load_controls_config()
        self._teleop_cfg = cfg.teleop
        self._joy = self._teleop_cfg.joy_mapping
        self.lift_deadband = self._teleop_cfg.lift_deadband

        self.get_logger().info(
            "Forklift Teleop Node Initialized. Default Gear: FORWARD. "
            "Waiting for /joy data..."
        )

    def get_axis(self, joy_msg, index, default=0.0):
        return (
            float(joy_msg.axes[index])
            if index < len(joy_msg.axes)
            else float(default)
        )

    def get_button(self, joy_msg, index, default=0):
        return (
            int(joy_msg.buttons[index])
            if index < len(joy_msg.buttons)
            else int(default)
        )

    def master_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if 'layout' in data:
                layout_msg = String()
                layout_msg.data = data['layout']
                self.preset_pub.publish(layout_msg)
                self.get_logger().info(f"Published layout preset: {layout_msg.data}")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON from /master_remop_message")
        except Exception as e:
            self.get_logger().error(f"Error processing master message: {e}")

    def joy_callback(self, joy_msg: Joy):
        self.get_logger().debug(
            f"CALLBACK FIRED! Received {len(joy_msg.axes)} axes and "
            f"{len(joy_msg.buttons)} buttons."
        )

        cmd = ForkliftDirectCommand()
        j = self._joy

        current_a_button = self.get_button(joy_msg, j.gear_button)
        if current_a_button == 1 and self.last_a_button_state == 0:
            self.is_forward_gear = not self.is_forward_gear
            gear_str = "FORWARD" if self.is_forward_gear else "REVERSE"
            self.get_logger().info(f"Gear Shifted: {gear_str}")
        self.last_a_button_state = current_a_button

        raw_rt = self.get_axis(joy_msg, j.throttle_axis, default=0.0)
        throttle_magnitude = max(0.0, float(raw_rt))
        direction_multiplier = 1.0 if self.is_forward_gear else -1.0
        cmd.drive_speed = float(throttle_magnitude * direction_multiplier)

        raw_steer = float(self.get_axis(joy_msg, j.steer_axis))
        if self.is_forward_gear:
            cmd.steering_angle = -raw_steer
        else:
            cmd.steering_angle = raw_steer

        raw_lift = -float(self.get_axis(joy_msg, j.lift_axis))
        if abs(raw_lift) < self.lift_deadband:
            cmd.lift_speed = 0.0
        else:
            cmd.lift_speed = raw_lift

        dpad_up = self.get_button(joy_msg, j.tilt_up_button)
        dpad_down = self.get_button(joy_msg, j.tilt_down_button)
        dpad_left = self.get_button(joy_msg, j.shift_left_button)
        dpad_right = self.get_button(joy_msg, j.shift_right_button)

        if dpad_up == 1:
            cmd.tilt_speed = 1.0
        elif dpad_down == 1:
            cmd.tilt_speed = -1.0
        else:
            cmd.tilt_speed = 0.0

        if dpad_right == 1:
            cmd.side_shift_speed = 1.0
        elif dpad_left == 1:
            cmd.side_shift_speed = -1.0
        else:
            cmd.side_shift_speed = 0.0

        self.cmd_pub.publish(cmd)
        self.get_logger().debug("Command published to /teleop/raw_command")


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
