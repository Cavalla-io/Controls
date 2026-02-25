import os
import yaml
import json
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Standard ROS messages
from std_msgs.msg import String, Float32

# Custom ROS message
from forklift_msgs.msg import ForkliftDirectCommand

# Hardware Interface
from .can_interface.mbv15_iface import MBV15Interface

class ForkliftDriverNode(Node):
    def __init__(self):
        super().__init__('forklift_driver')
        
        self.curtis = MBV15Interface(channel='can0', bitrate=250000)
        if self.curtis.connected:
            self.get_logger().info("Connected to MBV15/Curtis Controller on can0")
        else:
            self.get_logger().warn("CAN0 not found. Running in MOCK mode.")

        self.current_height_mm = 0.0

        try:
            package_share_directory = get_package_share_directory('forklift_driver')
            config_path = os.path.join(package_share_directory, 'config', 'presets.yaml')
        except Exception as e:
            self.get_logger().error(f"Could not find package share directory: {e}")
            config_path = ""

        self.presets = self.load_presets(config_path)
        
        self.active_preset_name = 'default'
        self.active_config = self.presets.get('default', self.get_failsafe_config()).copy()
        self.get_logger().info("Loaded preset: DEFAULT")

        self.create_subscription(ForkliftDirectCommand, '/safe/raw_command', self.teleop_callback, 1)
        self.create_subscription(String, '/forklift/set_preset', self.preset_callback, 1)
        self.create_subscription(Float32, '/forklift/fork_height', self.height_callback, 1)

    def load_presets(self, path):
        try:
            with open(path, 'r') as file:
                data = yaml.safe_load(file)
                return data.get('presets', {})
        except FileNotFoundError:
            self.get_logger().error(f"Presets file not found at {path}. Using failsafe defaults.")
            return {}

    def get_failsafe_config(self):
        return {
            'drive_scale': 0.2,   
            'steer_scale': 0.5,
            'lift_scale': 0.2,
            'lower_scale': 0.2,
            'allow_fork_movement': False, 
            'max_height_mm': 0,
            'min_height_mm': 0,
            'accel_time_s': 1.0,
            'decel_time_s': 1.0
        }

    def height_callback(self, msg: Float32):
        self.current_height_mm = msg.data

    def preset_callback(self, msg: String):
        text = msg.data.strip()

        if text.startswith('{'):
            try:
                overrides = json.loads(text)
                for key in ['max_height_mm', 'min_height_mm', 'accel_time_s', 'decel_time_s', 'lift_scale', 'lower_scale', 'drive_scale', 'steer_scale']:
                    if key in overrides:
                        self.active_config[key] = float(overrides[key])
                        self.get_logger().info(f"OVERRIDE: {key} set to {self.active_config[key]}")
                return
            except (json.JSONDecodeError, ValueError):
                return

        if ':' in text:
            key, val = text.split(':', 1)
            key = key.strip().lower()
            try:
                val = float(val.strip())
                if key in ['max_height', 'max_height_mm']: key = 'max_height_mm'
                elif key in ['min_height', 'min_height_mm']: key = 'min_height_mm'
                elif key in ['accel', 'accel_time', 'accel_time_s']: key = 'accel_time_s'
                elif key in ['decel', 'decel_time', 'decel_time_s']: key = 'decel_time_s'
                elif key in ['lift', 'lift_scale']: key = 'lift_scale'
                elif key in ['lower', 'lower_scale']: key = 'lower_scale'

                if key in self.active_config:
                    self.active_config[key] = val
                    self.get_logger().info(f"OVERRIDE: {key} set to {val}")
                return
            except ValueError:
                return

        requested = text.lower()
        if requested in self.presets:
            self.active_preset_name = requested
            self.active_config = self.presets[requested].copy() 
            self.get_logger().info(f"Preset changed to: {requested.upper()}")

    def teleop_callback(self, msg: ForkliftDirectCommand):
        config = self.active_config

        # 1. Apply Drive & Steer Scaling
        safe_drive = msg.drive_speed * config.get('drive_scale', 0.0)
        safe_steer = msg.steering_angle * config.get('steer_scale', 0.0)
        
        # 2. Split Lift/Lower Scaling
        if msg.lift_speed > 0:
            safe_lift = msg.lift_speed * config.get('lift_scale', 0.0)
        elif msg.lift_speed < 0:
            safe_lift = msg.lift_speed * config.get('lower_scale', 0.0)
        else:
            safe_lift = 0.0

        safe_tilt  = msg.tilt_speed 
        safe_shift = msg.side_shift_speed

        # 3. Apply Fork Logic Gates
        if not config.get('allow_fork_movement', False):
            safe_lift = 0.0
            safe_tilt = 0.0
            safe_shift = 0.0
        else:
            max_h = config.get('max_height_mm', 0)
            min_h = config.get('min_height_mm', 0)

            if safe_lift > 0 and self.current_height_mm >= max_h:
                safe_lift = 0.0
            elif safe_lift < 0 and self.current_height_mm <= min_h:
                safe_lift = 0.0

        # 4. Get Curve Parameters
        accel_s = config.get('accel_time_s', 1.0)
        decel_s = config.get('decel_time_s', 1.0)

        # 5. Dispatch to Hardware Layer
        self.curtis.send_commands(
            safe_drive, 
            safe_steer, 
            safe_lift, 
            safe_tilt, 
            safe_shift, 
            accel_s=accel_s, 
            decel_s=decel_s
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