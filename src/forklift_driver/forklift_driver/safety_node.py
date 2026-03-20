import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32
from forklift_msgs.msg import ForkliftDirectCommand

class ForkliftSafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # --- Parameters & State ---
        self.heartbeat_timeout_sec = 0.75  # 750ms
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

    def check_teleop_safety(self) -> tuple[bool, str]:
        """
        Checks if the teleop connection is healthy.
        Returns (is_safe, reason).
        """
        now = self.get_clock().now()
        time_since_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        
        is_safe = True
        reason = ""

        # 1. Check Heartbeat Status Code
        # 0 = safe, 1 = unfocused (unsafe), 2 = high latency (safe for now), 3 = disconnected (unsafe)
        if self.current_status_code in [1, 3]:
            is_safe = False
            reason = f"Status Code {self.current_status_code} (Unfocused/Disconnected)"
        
        # 2. Check Heartbeat Timeout (Network Drop)
        elif time_since_heartbeat > self.heartbeat_timeout_sec:
            is_safe = False
            reason = f"Heartbeat Stale ({time_since_heartbeat:.2f}s)"

        # 3. Check Command Timeout (Teleop Node Crashed)
        elif time_since_cmd > self.command_timeout_sec:
            is_safe = False
            reason = f"Command Stale ({time_since_cmd:.2f}s)"
            
        return is_safe, reason

    def check_automation_safety(self) -> tuple[bool, str]:
        """
        Checks if the automation connection is healthy.
        Returns (is_safe, reason).
        """
        now = self.get_clock().now()
        time_since_auto = (now - self.last_auto_msg_time).nanoseconds / 1e9
        
        is_safe = True
        reason = ""
        
        if time_since_auto > self.AUTO_TIMEOUT_SEC:
            is_safe = False
            reason = f"Auto Stale ({time_since_auto:.2f}s)"
            
        return is_safe, reason

    def watchdog_loop(self):
        now = self.get_clock().now()
        time_since_activity = (now - self.last_teleop_activity_time).nanoseconds / 1e9

        # --- SAFETY CHECKS ---
        teleop_safe, teleop_reason = self.check_teleop_safety()
        auto_safe, auto_reason = self.check_automation_safety()

        # --- MUX LOGIC: Determine Base Command ---
        mux_cmd = ForkliftDirectCommand()
        
        # Are we in Teleop Priority Mode?
        is_teleop_mode = (time_since_activity < self.TELEOP_PRIORITY_TIMEOUT)

        is_global_safe = True
        fault_reason = ""

        if is_teleop_mode:
            # Case 1: Teleop is active. Priority given to Teleop.
            if teleop_safe:
                mux_cmd = self.latest_teleop_cmd
            else:
                is_global_safe = False
                fault_reason = f"TELEOP SAFETY TRIP: {teleop_reason}"
        else:
            # Case 2: Teleop is idle. Switch to Auto.
            # We NO LONGER check teleop_safe here. Auto can run even if teleop is disconnected.
            
            if auto_safe:
                # Only apply lift effort if not canceled
                if not self.auto_fork_canceled:
                    mux_cmd.lift_speed = self.latest_auto_effort
                else:
                    mux_cmd.lift_speed = 0.0
            else:
                is_global_safe = False
                fault_reason = f"AUTO SAFETY TRIP: {auto_reason}"

        # --- ACTION ---
        if is_global_safe:
            if not self.was_safe:
                self.get_logger().info("SYSTEM SAFE. Passing commands to hardware.")
                self.was_safe = True
                
            self.safe_pub.publish(mux_cmd)
            
        else:
            if self.was_safe:
                self.get_logger().warn(f"SAFETY TRIP! Reason: {fault_reason}. Clamping to 0.0.")
                self.was_safe = False
                
            # Publish stop command
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
