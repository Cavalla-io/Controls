import os
import yaml
import json
import rclpy
from rclpy.node import Node

# Standard ROS messages
from std_msgs.msg import String, Float32

# Custom ROS message
from forklift_msgs.msg import ForkliftDirectCommand

# Hardware Interface (Pure Python)
from .can_interface.mbv15_iface import MBV15Interface

class ForkliftDriverNode(Node):
    def __init__(self):
        super().__init__('forklift_driver')
        
        # --- 1. HARDWARE INIT ---
        self.curtis = MBV15Interface(channel='can0', bitrate=250000)
        if self.curtis.connected:
            self.get_logger().info("Connected to MBV15/Curtis Controller on can0")
        else:
            self.get_logger().warn("CAN0 not found. Running in MOCK mode.")

        # --- 2. STATE VARIABLES ---
        self.current_height_mm = 0.0  # Continuously updated by the SICK encoder node

        # --- 3. PRESET MANAGEMENT ---
        # Note: Ensure you create this file at controls/forklift_driver/config/presets.yaml
        config_path = os.path.join(os.getcwd(), 'src', 'controls', 'forklift_driver', 'config', 'presets.yaml')
        self.presets = self.load_presets(config_path)
        
        self.active_preset_name = 'default'
        # Use .copy() so dynamic overrides don't permanently corrupt the base preset
        self.active_config = self.presets.get('default', self.get_failsafe_config()).copy()
        self.get_logger().info(f"Loaded preset: DEFAULT")

        # --- 4. SUBSCRIBERS ---
        # The joystick / teleop server commands
        self.create_subscription(
            ForkliftDirectCommand, 
            '/teleop/raw_command', 
            self.teleop_callback, 
            10
        )
        
        # The preset switcher and override topic
        self.create_subscription(
            String, 
            '/forklift/set_preset', 
            self.preset_callback, 
            10
        )
        
        # The external SICK draw wire encoder node
        self.create_subscription(
            Float32, 
            '/forklift/fork_height', 
            self.height_callback, 
            10
        )

    def load_presets(self, path):
        """Loads the YAML config file. Returns an empty dict if not found."""
        try:
            with open(path, 'r') as file:
                data = yaml.safe_load(file)
                return data.get('presets', {})
        except FileNotFoundError:
            self.get_logger().error(f"Presets file not found at {path}. Using failsafe defaults.")
            return {}

    def get_failsafe_config(self):
        """Hardcoded safety defaults in case the YAML file goes missing."""
        return {
            'drive_scale': 0.2,   # Very slow
            'steer_scale': 0.5,
            'lift_scale': 0.2,
            'allow_fork_movement': False, # Lockout forks entirely
            'max_height_mm': 0,
            'min_height_mm': 0
        }

    def height_callback(self, msg: Float32):
        """Updates internal height state from the SICK encoder node."""
        self.current_height_mm = msg.data

    def preset_callback(self, msg: String):
        """Hot-swaps the software limits or overrides specific parameters."""
        text = msg.data.strip()

        # --- A. JSON Override Method ---
        # e.g., '{"max_height_mm": 4000, "min_height_mm": 100}'
        if text.startswith('{'):
            try:
                overrides = json.loads(text)
                if 'max_height_mm' in overrides:
                    self.active_config['max_height_mm'] = float(overrides['max_height_mm'])
                    self.get_logger().info(f"OVERRIDE: max_height_mm set to {self.active_config['max_height_mm']}")
                if 'min_height_mm' in overrides:
                    self.active_config['min_height_mm'] = float(overrides['min_height_mm'])
                    self.get_logger().info(f"OVERRIDE: min_height_mm set to {self.active_config['min_height_mm']}")
                return
            except (json.JSONDecodeError, ValueError):
                self.get_logger().warn("Failed to parse JSON override.")
                return

        # --- B. Simple String Override Method ---
        # e.g., 'max_height:4000'
        if ':' in text:
            key, val = text.split(':', 1)
            key = key.strip().lower()
            try:
                val = float(val.strip())
                if key in ['max_height', 'max_height_mm']:
                    self.active_config['max_height_mm'] = val
                    self.get_logger().info(f"OVERRIDE: max_height_mm set to {val}")
                elif key in ['min_height', 'min_height_mm']:
                    self.active_config['min_height_mm'] = val
                    self.get_logger().info(f"OVERRIDE: min_height_mm set to {val}")
                return
            except ValueError:
                self.get_logger().warn(f"Invalid numeric value for override: {val}")
                return

        # --- C. Standard Preset Switcher ---
        # e.g., 'turtle'
        requested = text.lower()
        if requested in self.presets:
            self.active_preset_name = requested
            # CRITICAL: Use .copy() so overrides don't permanently corrupt the base preset
            self.active_config = self.presets[requested].copy() 
            self.get_logger().info(f"Preset changed to: {requested.upper()}")
        else:
            self.get_logger().warn(f"Unknown command or preset: '{requested}'. Ignoring.")

    def teleop_callback(self, msg: ForkliftDirectCommand):
        """
        The Interceptor: Applies scaling and height limits before hardware execution.
        """
        config = self.active_config

        # 1. Apply Scaling
        safe_drive = msg.drive_speed * config.get('drive_scale', 0.0)
        safe_steer = msg.steering_angle * config.get('steer_scale', 0.0)
        safe_lift  = msg.lift_speed * config.get('lift_scale', 0.0)
        safe_tilt  = msg.tilt_speed 
        safe_shift = msg.side_shift_speed

        # 2. Apply Fork Logic Gates
        if not config.get('allow_fork_movement', False):
            safe_lift = 0.0
            safe_tilt = 0.0
            safe_shift = 0.0
        else:
            # Software Limits: Prevent moving UP if too high, DOWN if too low
            max_h = config.get('max_height_mm', 0)
            min_h = config.get('min_height_mm', 0)

            if safe_lift > 0 and self.current_height_mm >= max_h:
                safe_lift = 0.0
            elif safe_lift < 0 and self.current_height_mm <= min_h:
                safe_lift = 0.0

        # 3. Dispatch to Hardware Layer
        self.curtis.send_motion(safe_drive, safe_steer, safe_lift)
        self.curtis.send_hydraulics(safe_lift, safe_tilt, safe_shift)

def main(args=None):
    rclpy.init(args=args)
    node = ForkliftDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Failsafe: Command zero velocity before the node shuts down
        node.get_logger().info("Shutting down driver. Commanding STOP.")
        node.curtis.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()