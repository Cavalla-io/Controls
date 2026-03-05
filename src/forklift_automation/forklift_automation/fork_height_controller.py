import os
import yaml
import math
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32


class ForkHeightController(Node):
    def __init__(self):
        super().__init__('fork_height_controller')

        # --- Load PID config ---
        try:
            pkg_share = get_package_share_directory('forklift_automation')
            config_path = os.path.join(pkg_share, 'config', 'pid.yaml')
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f).get('pid_config', {})
        except Exception as e:
            self.get_logger().warn(f"Could not load pid.yaml: {e}. Using defaults.")
            cfg = {}

        self.kp = cfg.get('kp', 0.002)
        self.ki = cfg.get('ki', 0.0)
        self.kd = cfg.get('kd', 0.0001)
        self.deadband_mm = cfg.get('deadband_mm', 10.0)
        self.integral_max = cfg.get('integral_max', 500.0)

        # --- State ---
        self.current_height_mm = 0.0
        self.target_height_m = None
        self.last_target_time = None
        self.TARGET_TIMEOUT_S = 0.5

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.pid_last_time = None
        self.was_active = False

        # --- ROS interfaces ---
        self.create_subscription(Float32, '/forklift/fork_height', self.height_cb, 1)
        self.create_subscription(Float32, '/forklift/target_fork_height', self.target_cb, 1)

        self.effort_pub = self.create_publisher(Float32, '/forklift/auto_lift_effort', 10)

        # Run at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f"Fork Height Controller started — Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, "
            f"deadband={self.deadband_mm}mm"
        )

    # ----- Callbacks -----

    def height_cb(self, msg: Float32):
        self.current_height_mm = msg.data

    def target_cb(self, msg: Float32):
        self.target_height_m = msg.data
        self.last_target_time = self.get_clock().now()

    # ----- Control Loop -----

    def is_active(self):
        """Active when a valid (>0) target is being published and hasn't timed out."""
        if self.target_height_m is None or self.target_height_m <= 0:
            return False
        if self.last_target_time is None:
            return False
        age_s = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        return age_s <= self.TARGET_TIMEOUT_S

    def reset_pid(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.pid_last_time = None

    def control_loop(self):
        msg = Float32()
        active = self.is_active()

        # Mode transitions
        if active and not self.was_active:
            self.reset_pid()
            self.get_logger().info(
                f"AUTO ENGAGED — target: {self.target_height_m:.3f}m, "
                f"current: {self.current_height_mm:.1f}mm"
            )
        elif not active and self.was_active:
            self.reset_pid()
            self.get_logger().info(
                f"AUTO DISENGAGED — height at {self.current_height_mm:.1f}mm"
            )
        self.was_active = active

        if not active:
            # Publish NaN to signal driver to use teleop
            msg.data = float('nan')
            self.effort_pub.publish(msg)
            return

        # --- PID compute ---
        now = self.get_clock().now()

        if self.pid_last_time is None:
            self.pid_last_time = now
            msg.data = 0.0
            self.effort_pub.publish(msg)
            return

        dt = (now - self.pid_last_time).nanoseconds / 1e9
        self.pid_last_time = now
        if dt <= 0:
            msg.data = 0.0
            self.effort_pub.publish(msg)
            return

        target_mm = self.target_height_m * 1000.0
        error = target_mm - self.current_height_mm

        # Deadband
        if abs(error) < self.deadband_mm:
            self.integral = 0.0
            self.prev_error = error
            msg.data = 0.0
            self.effort_pub.publish(msg)
            return

        # PID terms
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-1.0, min(1.0, output))

        msg.data = output
        self.effort_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForkHeightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
