import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from forklift_msgs.msg import ForkliftDirectCommand
from rclpy.qos import qos_profile_sensor_data

class ForkliftTeleop(Node):
    def __init__(self):
        super().__init__('forklift_teleop')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(ForkliftDirectCommand, '/teleop/raw_command', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
        
        # State tracking for the Forward/Reverse toggle
        self.is_forward_gear = True
        self.last_a_button_state = 0
        
        self.get_logger().info("Forklift Teleop Node Initialized. Default Gear: FORWARD. Waiting for /joy data...")

    def get_axis(self, joy_msg, index, default=0.0):
        """Safely gets an axis value without crashing if the array is too short."""
        return float(joy_msg.axes[index]) if index < len(joy_msg.axes) else float(default)

    def get_button(self, joy_msg, index, default=0):
        """Safely gets a button value without crashing if the array is too short."""
        return int(joy_msg.buttons[index]) if index < len(joy_msg.buttons) else int(default)

    def joy_callback(self, joy_msg: Joy):
        self.get_logger().debug(f"CALLBACK FIRED! Received {len(joy_msg.axes)} axes and {len(joy_msg.buttons)} buttons.")
        
        cmd = ForkliftDirectCommand()
        
        # --- 1. HANDLE GEAR TOGGLE ---
        # Usually Button 0
        current_a_button = self.get_button(joy_msg, 0)
        if current_a_button == 1 and self.last_a_button_state == 0:
            self.is_forward_gear = not self.is_forward_gear
            gear_str = "FORWARD" if self.is_forward_gear else "REVERSE"
            self.get_logger().info(f"Gear Shifted: {gear_str}")
            
        self.last_a_button_state = current_a_button
        
        # --- 2. HANDLE THROTTLE ---
        # Web teleop servers typically idle at 0.0 and go to 1.0 when fully pressed.
        raw_rt = self.get_axis(joy_msg, 5, default=0.0)
        
        # Clamp it to ensure we never get a negative throttle magnitude 
        throttle_magnitude = max(0.0, float(raw_rt))
        
        # Apply gear direction to throttle
        direction_multiplier = 1.0 if self.is_forward_gear else -1.0
        cmd.drive_speed = float(throttle_magnitude * direction_multiplier)
        
        # --- 3. HANDLE STEERING & LIFT ---
        raw_steer = float(self.get_axis(joy_msg, 0))
        
        # INVERT STEERING IN FORWARD GEAR ONLY
        if self.is_forward_gear:
            cmd.steering_angle = -raw_steer
        else:
            cmd.steering_angle = raw_steer
            
        # Right Stick Y (Usually Axis 3)
        cmd.lift_speed = -float(self.get_axis(joy_msg, 3))
        
        # --- 4. HANDLE TILT & SIDE SHIFT ---
        # In 6-axis setups, the D-Pad is usually buttons 11 through 14
        dpad_up = self.get_button(joy_msg, 11)   
        dpad_down = self.get_button(joy_msg, 12) 
        dpad_left = self.get_button(joy_msg, 13) 
        dpad_right = self.get_button(joy_msg, 14)

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
            
        # Publish the command
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