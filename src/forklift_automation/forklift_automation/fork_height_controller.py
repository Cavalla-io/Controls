import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from forklift_config import load_controls_config


class ForkHeightController(Node):
    def __init__(self):
        super().__init__("fork_height_controller")

        cfg = load_controls_config()
        self._fh_cfg = cfg.fork_height
        pid = self._fh_cfg.pid
        topics = self._fh_cfg.ros_topics

        self.kp = pid.kp
        self.ki = pid.ki
        self.kd = pid.kd
        self.deadband_mm = pid.deadband_mm
        self.integral_max = pid.integral_max
        self.TARGET_TIMEOUT_S = self._fh_cfg.target_timeout_sec

        self.current_height_mm = 0.0
        self.target_height_m = None
        self.last_target_time = None

        self.integral = 0.0
        self.prev_error = 0.0
        self.pid_last_time = None
        self.was_active = False

        self.create_subscription(
            Float32, topics.fork_position, self.height_cb, 1
        )
        self.create_subscription(
            Float32, topics.target_fork_height, self.target_cb, 1
        )
        self.effort_pub = self.create_publisher(
            Float32, topics.auto_lift_effort, 10
        )

        self.timer = self.create_timer(
            self._fh_cfg.control_loop_period_sec, self.control_loop
        )

        self.get_logger().info(
            f"Fork Height Controller started — Kp={self.kp}, Ki={self.ki}, "
            f"Kd={self.kd}, deadband={self.deadband_mm}mm"
        )

    def height_cb(self, msg: Float32):
        self.current_height_mm = msg.data

    def target_cb(self, msg: Float32):
        self.target_height_m = msg.data
        self.last_target_time = self.get_clock().now()

    def is_active(self):
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
            msg.data = float("nan")
            self.effort_pub.publish(msg)
            return

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

        if abs(error) < self.deadband_mm:
            self.integral = 0.0
            self.prev_error = error
            msg.data = 0.0
            self.effort_pub.publish(msg)
            return

        self.integral += error * dt
        self.integral = max(
            -self.integral_max, min(self.integral_max, self.integral)
        )

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )
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


if __name__ == "__main__":
    main()
