import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from forklift_msgs.msg import ForkliftDirectCommand # Your custom message

class ForkliftTeleop(Node):
    def __init__(self):
        super().__init__('forklift_teleop')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(ForkliftDirectCommand, '/teleop/raw_command', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # State tracking for the Forward/Reverse toggle
        self.is_forward_gear = True
        self.last_a_button_state = 0
        
        self.get_logger().info("Forklift Teleop Node Initialized. Default Gear: FORWARD")

    def joy_callback(self, joy_msg: Joy):
        cmd = ForkliftDirectCommand()
        
        # --- 1. HANDLE GEAR TOGGLE (A Button / Cross) ---
        # Index 0 is standard for A/Cross. We only toggle on the "press down" (transition from 0 to 1)
        current_a_button = joy_msg.buttons[0]
        if current_a_button == 1 and self.last_a_button_state == 0:
            self.is_forward_gear = not self.is_forward_gear
            gear_str = "FORWARD" if self.is_forward_gear else "REVERSE"
            self.get_logger().info(f"Gear Shifted: {gear_str}")
            
        self.last_a_button_state = current_a_button
        
        # --- 2. HANDLE THROTTLE (Right Trigger) ---
        # Axis 5 is RT. Usually 1.0 (unpressed) to -1.0 (pressed).
        # Math: (1.0 - raw_val) / 2.0 converts it to a 0.0 to 1.0 scale.
        raw_rt = joy_msg.axes[5]
        throttle_magnitude = (1.0 - raw_rt) / 2.0
        
        # Apply gear direction to throttle
        direction_multiplier = 1.0 if self.is_forward_gear else -1.0
        cmd.drive_speed = throttle_magnitude * direction_multiplier
        
        # --- 3. HANDLE STEERING & FORKS ---
        # Note: Axis indices can vary slightly by controller/driver. 
        # These are the standard Linux indices.
        cmd.steering_angle = joy_msg.axes[0]     # Left Stick X (Usually Left=1.0, Right=-1.0)
        cmd.lift_speed = joy_msg.axes[4]         # Right Stick Y (Usually Up=1.0, Down=-1.0)
        
        cmd.side_shift_speed = joy_msg.axes[6]   # D-Pad Left/Right
        cmd.tilt_speed = joy_msg.axes[7]         # D-Pad Up/Down
        
        # Publish the God Packet
        self.cmd_pub.publish(cmd)

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