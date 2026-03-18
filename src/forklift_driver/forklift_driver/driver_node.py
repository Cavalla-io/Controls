import os
import yaml
import json
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Float32
from forklift_msgs.msg import ForkliftDirectCommand

from forklift_config import load_controls_config
from .can_interface.mbv15_iface import MBV15Interface


class ForkliftDriverNode(Node):
    def __init__(self):
        super().__init__("forklift_driver")

        cfg = load_controls_config()
        self._driver_cfg = cfg.driver
        self._hw_cfg = cfg.hardware

        self.curtis = MBV15Interface(
            channel=self._hw_cfg.can.channel,
            bitrate=self._hw_cfg.can.bitrate,
            is_mock=self._hw_cfg.can.is_mock,
            mbv15_config=self._hw_cfg.mbv15,
        )
        if self.curtis.connected:
            self.get_logger().info(
                f"Connected to MBV15/Curtis Controller on {self._hw_cfg.can.channel}"
            )
        else:
            self.get_logger().warn(
                f"{self._hw_cfg.can.channel} not found. Running in MOCK mode."
            )

        self.current_height_mm = 0.0

        if self._driver_cfg.presets:
            self.presets = self._driver_cfg.presets
        else:
            config_path = self._driver_cfg.presets_file
            if config_path and not os.path.isabs(config_path) and cfg.config_path:
                config_dir = os.path.dirname(cfg.config_path)
                config_path = os.path.join(config_dir, config_path)
            if not config_path or not os.path.isfile(config_path):
                try:
                    package_share_directory = get_package_share_directory("forklift_driver")
                    config_path = os.path.join(package_share_directory, "config", "presets.yaml")
                except Exception as e:
                    self.get_logger().error(f"Could not find package share directory: {e}")
                    config_path = ""
            self.presets = self.load_presets(config_path)

        failsafe = self._driver_cfg.failsafe
        self._failsafe_dict = {
            "drive_scale": failsafe.drive_scale,
            "steer_scale": failsafe.steer_scale,
            "lift_scale": failsafe.lift_scale,
            "lower_scale": failsafe.lower_scale,
            "allow_fork_movement": failsafe.allow_fork_movement,
            "max_height_mm": failsafe.max_height_mm,
            "min_height_mm": failsafe.min_height_mm,
            "accel_time_s": failsafe.accel_time_s,
            "decel_time_s": failsafe.decel_time_s,
        }

        self.active_preset_name = self._driver_cfg.default_preset
        self.active_config = self.presets.get(
            self._driver_cfg.default_preset, self._failsafe_dict.copy()
        ).copy()
        self.get_logger().info(f"Loaded preset: {self.active_preset_name.upper()}")

        topics = self._driver_cfg.ros_topics
        self.create_subscription(
            ForkliftDirectCommand, topics.safe_raw_command, self.teleop_callback, 1
        )
        self.create_subscription(String, topics.set_preset, self.preset_callback, 1)
        self.create_subscription(Float32, topics.fork_position, self.height_callback, 1)

    def load_presets(self, path):
        if not path:
            return {}
        try:
            with open(path, "r") as file:
                data = yaml.safe_load(file)
                return data.get("presets", {})
        except FileNotFoundError:
            self.get_logger().error(
                f"Presets file not found at {path}. Using failsafe defaults."
            )
            return {}

    def get_failsafe_config(self):
        return self._failsafe_dict.copy()

    def height_callback(self, msg: Float32):
        self.current_height_mm = msg.data

    def preset_callback(self, msg: String):
        text = msg.data.strip()

        if text.startswith("{"):
            try:
                overrides = json.loads(text)
                for key in [
                    "max_height_mm",
                    "min_height_mm",
                    "accel_time_s",
                    "decel_time_s",
                    "lift_scale",
                    "lower_scale",
                    "drive_scale",
                    "steer_scale",
                ]:
                    if key in overrides:
                        self.active_config[key] = float(overrides[key])
                        self.get_logger().info(
                            f"OVERRIDE: {key} set to {self.active_config[key]}"
                        )
                return
            except (json.JSONDecodeError, ValueError):
                return

        if ":" in text:
            key, val = text.split(":", 1)
            key = key.strip().lower()
            try:
                val = float(val.strip())
                if key in ["max_height", "max_height_mm"]:
                    key = "max_height_mm"
                elif key in ["min_height", "min_height_mm"]:
                    key = "min_height_mm"
                elif key in ["accel", "accel_time", "accel_time_s"]:
                    key = "accel_time_s"
                elif key in ["decel", "decel_time", "decel_time_s"]:
                    key = "decel_time_s"
                elif key in ["lift", "lift_scale"]:
                    key = "lift_scale"
                elif key in ["lower", "lower_scale"]:
                    key = "lower_scale"

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

        safe_drive = msg.drive_speed * config.get("drive_scale", 0.0)
        safe_steer = msg.steering_angle * config.get("steer_scale", 0.0)

        if msg.lift_speed > 0:
            safe_lift = msg.lift_speed * config.get("lift_scale", 0.0)
        elif msg.lift_speed < 0:
            safe_lift = msg.lift_speed * config.get("lower_scale", 0.0)
        else:
            safe_lift = 0.0

        safe_tilt = msg.tilt_speed
        safe_shift = msg.side_shift_speed

        if not config.get("allow_fork_movement", False):
            safe_lift = 0.0
            safe_tilt = 0.0
            safe_shift = 0.0
        else:
            max_h = config.get("max_height_mm", 0)
            min_h = config.get("min_height_mm", 0)
            if safe_lift > 0 and self.current_height_mm >= max_h:
                safe_lift = 0.0
            elif safe_lift < 0 and self.current_height_mm <= min_h:
                safe_lift = 0.0

        accel_s = config.get("accel_time_s", 1.0)
        decel_s = config.get("decel_time_s", 1.0)

        self.curtis.send_commands(
            safe_drive,
            safe_steer,
            safe_lift,
            safe_tilt,
            safe_shift,
            accel_s=accel_s,
            decel_s=decel_s,
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


if __name__ == "__main__":
    main()
