import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from forklift_msgs.msg import ForkliftDirectCommand

class ForkliftSafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # --- Parameters & State ---
        self.heartbeat_timeout_sec = 0.5  # 500ms from your C++ code
        self.command_timeout_sec = 0.5    # How old a teleop command can be before it's considered stale
        
        self.last_heartbeat_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        
        self.latest_cmd = ForkliftDirectCommand()
        self.current_status_code = 3  # Default to 3 (Disconnected/Unsafe)
        self.was_safe = False         # Edge-detection for cleaner logging

        # --- Subscribers ---
        # 1. The Raw Teleop Command
        self.create_subscription(ForkliftDirectCommand, '/teleop/raw_command', self.teleop_cb, 1)
        # 2. The Adamo Web Heartbeat (0=Safe, 1=Unfocused, 2=High Latency, 3=Disconnected)
        self.create_subscription(UInt8, '/safety', self.heartbeat_cb, 1)

        # --- Publishers ---
        # The Verified, Safe Command going to the Hardware Driver
        self.safe_pub = self.create_publisher(ForkliftDirectCommand, '/safe/raw_command', 1)

        # --- Watchdog Timer (Runs at 10Hz / every 100ms) ---
        self.create_timer(0.1, self.watchdog_loop)

        self.get_logger().info("Safety Multiplexer Initialized. Awaiting Heartbeat...")

    def heartbeat_cb(self, msg: UInt8):
        self.current_status_code = msg.data
        self.last_heartbeat_time = self.get_clock().now()

    def teleop_cb(self, msg: ForkliftDirectCommand):
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def watchdog_loop(self):
        now = self.get_clock().now()
        
        time_since_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

        # --- SAFETY LOGIC GATES ---
        is_safe = True
        fault_reason = ""

        # 1. Check Heartbeat Status Code
        # C++ logic: 0 = safe, 1 = unfocused (unsafe), 2 = high latency (safe for now), 3 = disconnected (unsafe)
        if self.current_status_code in [1, 3]:
            is_safe = False
            fault_reason = f"Status Code {self.current_status_code} (Unfocused/Disconnected)"
        
        # 2. Check Heartbeat Timeout (Network Drop)
        elif time_since_heartbeat > self.heartbeat_timeout_sec:
            is_safe = False
            fault_reason = f"Heartbeat Stale ({time_since_heartbeat:.2f}s)"

        # 3. Check Command Timeout (Teleop Node Crashed)
        elif time_since_cmd > self.command_timeout_sec:
            is_safe = False
            fault_reason = f"Command Stale ({time_since_cmd:.2f}s)"

        # --- ACTION ---
        if is_safe:
            if not self.was_safe:
                self.get_logger().info("SYSTEM SAFE. Passing commands to hardware.")
                self.was_safe = True
                
            # Forward the active command
            self.safe_pub.publish(self.latest_cmd)
            
        else:
            if self.was_safe:
                self.get_logger().warn(f"SAFETY TRIP! Reason: {fault_reason}. Clamping to 0.0.")
                self.was_safe = False
                
            # Publish a completely zeroed-out command to instantly stop the forklift
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

if __name__ == '__main__':
    main()