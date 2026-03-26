import os
import yaml
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32
from forklift_msgs.msg import ForkliftDirectCommand


class ForkliftTeleop(Node):
    def __init__(self):
        super().__init__('forklift_teleop')

        # --- Load presets ---
        try:
            pkg_share = get_package_share_directory('forklift_teleop')
            config_path = os.path.join(pkg_share, 'config', 'presets.yaml')
        except Exception as e:
            self.get_logger().error(f"Could not find package share directory: {e}")
            config_path = ""

        self.presets = self._load_presets(config_path)

        self.active_preset_name = 'default'
        self.active_config = self.presets.get('default', self._failsafe_config()).copy()
        self.get_logger().info("Loaded preset: DEFAULT")

        # --- State ---
        self.is_forward_gear = True
        self.last_a_button_state = 0
        self.current_height_mm = 0.0

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(ForkliftDirectCommand, '/teleop/raw_command', 1)
        self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/forklift/set_preset', self._preset_callback, 1)
        self.create_subscription(Float32, '/forklift/fork_height', self._height_callback, 1)

        self.get_logger().info("Forklift Teleop Node Initialized. Default Gear: FORWARD. Waiting for /joy data...")

    # ------------------------------------------------------------------ #
    #  Preset helpers
    # ------------------------------------------------------------------ #

    def _load_presets(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('presets', {})
        except FileNotFoundError:
            self.get_logger().error(f"Presets file not found at {path}. Using failsafe defaults.")
            return {}

    @staticmethod
    def _failsafe_config():
        return {
            'drive_scale': 0.2,
            'steer_scale': 0.5,
            'lift_scale': 0.2,
            'lower_scale': 0.2,
            'allow_fork_movement': False,
            'max_height_mm': 0,
            'min_height_mm': 0,
            'accel_time_s': 1.0,
            'decel_time_s': 1.0,
        }

    # ------------------------------------------------------------------ #
    #  Callbacks
    # ------------------------------------------------------------------ #

    def _height_callback(self, msg: Float32):
        self.current_height_mm = msg.data

    def _preset_callback(self, msg: String):
        text = msg.data.strip()

        # JSON blob override
        if text.startswith('{'):
            try:
                overrides = json.loads(text)
                for key in self._failsafe_config():
                    if key in overrides:
                        self.active_config[key] = float(overrides[key])
                        self.get_logger().info(f"OVERRIDE: {key} set to {self.active_config[key]}")
                return
            except (json.JSONDecodeError, ValueError):
                return

        # Single key:value override
        if ':' in text:
            key, val = text.split(':', 1)
            key = key.strip().lower()
            try:
                val = float(val.strip())
                alias_map = {
                    'max_height': 'max_height_mm', 'max_height_mm': 'max_height_mm',
                    'min_height': 'min_height_mm', 'min_height_mm': 'min_height_mm',
                    'accel': 'accel_time_s', 'accel_time': 'accel_time_s', 'accel_time_s': 'accel_time_s',
                    'decel': 'decel_time_s', 'decel_time': 'decel_time_s', 'decel_time_s': 'decel_time_s',
                    'lift': 'lift_scale', 'lift_scale': 'lift_scale',
                    'lower': 'lower_scale', 'lower_scale': 'lower_scale',
                }
                key = alias_map.get(key, key)

                if key in self.active_config:
                    self.active_config[key] = val
                    self.get_logger().info(f"OVERRIDE: {key} set to {val}")
                return
            except ValueError:
                return

        # Named preset switch
        requested = text.lower()
        if requested in self.presets:
            self.active_preset_name = requested
            self.active_config = self.presets[requested].copy()
            self.get_logger().info(f"Preset changed to: {requested.upper()}")

    # ------------------------------------------------------------------ #
    #  Joy helpers
    # ------------------------------------------------------------------ #

    def get_axis(self, joy_msg, index, default=0.0):
        """Safely gets an axis value without crashing if the array is too short."""
        return float(joy_msg.axes[index]) if index < len(joy_msg.axes) else float(default)

    def get_button(self, joy_msg, index, default=0):
        """Safely gets a button value without crashing if the array is too short."""
        return int(joy_msg.buttons[index]) if index < len(joy_msg.buttons) else int(default)

    # ------------------------------------------------------------------ #
    #  Main joy callback  (raw mapping → preset scaling → publish)
    # ------------------------------------------------------------------ #

    def joy_callback(self, joy_msg: Joy):
        self.get_logger().debug(
            f"CALLBACK FIRED! Received {len(joy_msg.axes)} axes and {len(joy_msg.buttons)} buttons."
        )

        config = self.active_config
        cmd = ForkliftDirectCommand()

        # --- 1. GEAR TOGGLE (Button 0) ---
        current_a_button = self.get_button(joy_msg, 0)
        if current_a_button == 1 and self.last_a_button_state == 0:
            self.is_forward_gear = not self.is_forward_gear
            gear_str = "FORWARD" if self.is_forward_gear else "REVERSE"
            self.get_logger().info(f"Gear Shifted: {gear_str}")
        self.last_a_button_state = current_a_button

        # --- 2. THROTTLE ---
        raw_rt = self.get_axis(joy_msg, 5, default=0.0)
        throttle_magnitude = max(0.0, float(raw_rt))
        direction_multiplier = 1.0 if self.is_forward_gear else -1.0
        raw_drive = float(throttle_magnitude * direction_multiplier)

        # --- 3. STEERING ---
        raw_steer = float(self.get_axis(joy_msg, 0))
        if self.is_forward_gear:
            raw_steer = -raw_steer

        # --- 4. LIFT (Right Stick Y, Axis 3) ---
        raw_lift = -float(self.get_axis(joy_msg, 3))

        # --- 5. TILT & SIDE SHIFT (D-Pad) ---
        dpad_up    = self.get_button(joy_msg, 13)
        dpad_down  = self.get_button(joy_msg, 12)
        dpad_left  = self.get_button(joy_msg, 14)
        dpad_right = self.get_button(joy_msg, 15)

        raw_tilt = 1.0 if dpad_up else (-1.0 if dpad_down else 0.0)
        raw_shift = 1.0 if dpad_right else (-1.0 if dpad_left else 0.0)

        # ====== APPLY PRESET SCALING ====== #

        cmd.drive_speed = float(raw_drive * config.get('drive_scale', 0.0))
        cmd.steering_angle = float(raw_steer * config.get('steer_scale', 0.0))

        if raw_lift > 0:
            cmd.lift_speed = float(raw_lift * config.get('lift_scale', 0.0))
        elif raw_lift < 0:
            cmd.lift_speed = float(raw_lift * config.get('lower_scale', 0.0))
        else:
            cmd.lift_speed = 0.0

        cmd.tilt_speed = raw_tilt
        cmd.side_shift_speed = raw_shift

        # ====== FORK MOVEMENT GATES ====== #

        if not config.get('allow_fork_movement', False):
            cmd.lift_speed = 0.0
            cmd.tilt_speed = 0.0
            cmd.side_shift_speed = 0.0
        else:
            max_h = config.get('max_height_mm', 0)
            min_h = config.get('min_height_mm', 0)
            if cmd.lift_speed > 0 and self.current_height_mm >= max_h:
                cmd.lift_speed = 0.0
            elif cmd.lift_speed < 0 and self.current_height_mm <= min_h:
                cmd.lift_speed = 0.0

        # ====== DRIVE CURVE PARAMETERS ====== #

        cmd.accel_time_s = float(config.get('accel_time_s', 1.0))
        cmd.decel_time_s = float(config.get('decel_time_s', 0.2))

        # ====== PUBLISH ====== #

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


if __name__ == '__main__':
    main()
