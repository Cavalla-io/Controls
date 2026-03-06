import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32
from forklift_msgs.msg import ForkliftDirectCommand

class ForkliftSafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # --- Parameters & State ---
        self.heartbeat_timeout_sec = 0.5  # 500ms from your C++ code
        self.command_timeout_sec = 0.5    # How old a teleop command can be before it's considered stale
        
        self.last_heartbeat_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        
        # --- MUX STATE ---
        self.latest_teleop_cmd = ForkliftDirectCommand()
        self.latest_auto_effort = 0.0
        self.last_auto_msg_time = self.get_clock().now()  # Track when we last heard from auto
        self.AUTO_TIMEOUT_SEC = 0.5  # If auto stops talking, we stop listening
        
        self.auto_fork_canceled = False
        self.last_teleop_activity_time = self.get_clock().now()
        
        self.TELEOP_PRIORITY_TIMEOUT = 3.0  # Pause auto for 3s after teleop
        self.ACTIVITY_THRESHOLD = 0.01      # Threshold to consider teleop "active"

        self.current_status_code = 3  # Default to 3 (Disconnected/Unsafe)
        self.was_safe = False         # Edge-detection for cleaner logging

        # --- Subscribers ---
        # 1. The Raw Teleop Command
        self.create_subscription(ForkliftDirectCommand, '/teleop/raw_command', self.teleop_cb, 1)
        # 2. The Adamo Web Heartbeat (0=Safe, 1=Unfocused, 2=High Latency, 3=Disconnected)
        self.create_subscription(UInt8, '/safety', self.heartbeat_cb, 1)

        # 3. Auto Inputs (For Muxing)
        self.create_subscription(Float32, '/forklift/auto_lift_effort', self.auto_effort_cb, 10)
        # Watch for new targets to un-cancel the auto fork
        self.create_subscription(Float32, '/forklift/target_fork_height', self.auto_target_cb, 1)

        # --- Publishers ---
        # The Verified, Safe Command going to the Hardware Driver
        self.safe_pub = self.create_publisher(ForkliftDirectCommand, '/safe/raw_command', 1)

        # --- Watchdog Timer (Runs at 10Hz / every 100ms) ---
        self.create_timer(0.1, self.watchdog_loop)

        self.get_logger().info("Safety Multiplexer Initialized. Awaiting Heartbeat...")

    def heartbeat_cb(self, msg: UInt8):
        self.current_status_code = msg.data
        self.last_heartbeat_time = self.get_clock().now()

    def auto_effort_cb(self, msg: Float32):
        self.latest_auto_effort = msg.data
        self.last_auto_msg_time = self.get_clock().now()

    def auto_target_cb(self, msg: Float32):
        # If we receive a new target, we assume the operator wants auto to resume
        if self.auto_fork_canceled:
            self.get_logger().info("New Auto Target Received: Resuming Auto Fork control.")
            self.auto_fork_canceled = False

    def teleop_cb(self, msg: ForkliftDirectCommand):
        self.latest_teleop_cmd = msg
        self.last_cmd_time = self.get_clock().now()

        # Check for activity on any axis
        is_driving = (abs(msg.drive_speed) > self.ACTIVITY_THRESHOLD or 
                      abs(msg.steering_angle) > self.ACTIVITY_THRESHOLD)
        
        is_forking = (abs(msg.lift_speed) > self.ACTIVITY_THRESHOLD or 
                      abs(msg.tilt_speed) > self.ACTIVITY_THRESHOLD or 
                      abs(msg.side_shift_speed) > self.ACTIVITY_THRESHOLD or 
                      abs(msg.fork_spread_speed) > self.ACTIVITY_THRESHOLD)

        if is_driving or is_forking:
            self.last_teleop_activity_time = self.get_clock().now()
            
            # If the operator specifically moved the forks, cancel auto until next target
            if is_forking and not self.auto_fork_canceled:
                self.get_logger().warn("Teleop Fork Intervention: Canceling Auto Fork until new target.")
                self.auto_fork_canceled = True

    def watchdog_loop(self):
        now = self.get_clock().now()
        
        time_since_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        time_since_activity = (now - self.last_teleop_activity_time).nanoseconds / 1e9
        time_since_auto = (now - self.last_auto_msg_time).nanoseconds / 1e9

        # --- MUX LOGIC: Determine Base Command ---
        mux_cmd = ForkliftDirectCommand()

        if time_since_activity < self.TELEOP_PRIORITY_TIMEOUT:
            # Case 1: Teleop is active (or was recently). Priority given to Teleop.
            mux_cmd = self.latest_teleop_cmd
        else:
            # Case 2: Teleop is idle. Switch to Auto.
            
            # --- SAFETY CHECK: AUTO TIMEOUT ---
            # If we haven't heard from the auto controller recently, assume it died
            # and clamp to 0.0. This prevents "latching" old values.
            if time_since_auto > self.AUTO_TIMEOUT_SEC:
                # Silently clamp to 0.0 (or log if you prefer)
                self.latest_auto_effort = 0.0

            # Only apply lift effort if not canceled
            if not self.auto_fork_canceled:
                mux_cmd.lift_speed = self.latest_auto_effort
            else:
                mux_cmd.lift_speed = 0.0
            
            # (Optional) You could merge steer/drive from other auto nodes here

        # --- SAFETY LOGIC GATES ---
        is_safe = True
        fault_reason = ""

        # 1. Check Heartbeat Status Code
        # C++ logic: 0 = safe, 1 = unfocused (unsafe), 2 = high latency (safe for now), 3 = disconnected (unsafe)
        # if self.current_status_code in [1, 3]:
        #     is_safe = False
        #     fault_reason = f"Status Code {self.current_status_code} (Unfocused/Disconnected)"
        
        # 2. Check Heartbeat Timeout (Network Drop)
        # elif time_since_heartbeat > self.heartbeat_timeout_sec:
        #     is_safe = False
        #     fault_reason = f"Heartbeat Stale ({time_since_heartbeat:.2f}s)"

        # 3. Check Command Timeout (Teleop Node Crashed)
        # Note: We check teleop timeout even in auto mode to ensure the "kill switch" path is alive
        # elif time_since_cmd > self.command_timeout_sec:
        #     is_safe = False
        #     fault_reason = f"Command Stale ({time_since_cmd:.2f}s)"

        # --- ACTION ---
        if is_safe:
            if not self.was_safe:
                self.get_logger().info("SYSTEM SAFE. Passing commands to hardware.")
                self.was_safe = True
                
            # Forward the muxed command
            self.safe_pub.publish(mux_cmd)
            
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
