"""Operator Relay Node

Subscribes to operator data from the adamo network and republishes to ROS:
  - Joy (gamepad)      -> /joy
  - Safety state       -> /safety
  - Operator layout    -> /master_remop_message  (JSON: {"layout": ...})
  - Selected height    -> /master_remop_message  (JSON: {"selected target height": ...})

Also manages fork target heights with persistent disk storage and publishes
the height state back to the operator widget every 2 seconds.
"""

import json
import os
import struct
import threading
import time

import adamo
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String, Float32

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
API_KEY = "ak_2uvzHFhWOkcyGGugop4BS0QLoG38mPfC"
ROBOT_NAME = "test-unit"
HEIGHTS_FILE = os.environ.get(
    "OPERATOR_RELAY_HEIGHTS_FILE",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "fork_heights.json"),
)

# How often to log Joy messages (seconds)
JOY_LOG_INTERVAL = 1.0


# ---------------------------------------------------------------------------
# CDR envelope helpers (matches the TypeScript encodeRosEnvelope format)
# ---------------------------------------------------------------------------

def parse_envelope(data: bytes) -> tuple[str, str, bytes]:
    """Parse a control envelope: BE-u32 topic_len + topic + BE-u32 type_len + type + CDR."""
    offset = 0
    topic_len = struct.unpack_from(">I", data, offset)[0]
    offset += 4
    topic = data[offset : offset + topic_len].decode("utf-8")
    offset += topic_len
    type_len = struct.unpack_from(">I", data, offset)[0]
    offset += 4
    type_name = data[offset : offset + type_len].decode("utf-8")
    offset += type_len
    cdr_payload = data[offset:]
    return topic, type_name, cdr_payload


CDR_LE_HEADER = b"\x00\x01\x00\x00"


def ensure_cdr_header(data: bytes) -> bytes:
    """Ensure the CDR payload starts with a little-endian CDR header."""
    if data.startswith(CDR_LE_HEADER):
        return data
    if len(data) >= 4 and data[0] == 0x01:
        return CDR_LE_HEADER + data[4:]
    return CDR_LE_HEADER + data


# ---------------------------------------------------------------------------
# CDR decoders
# ---------------------------------------------------------------------------

def decode_cdr_string(cdr: bytes) -> str:
    """Decode CDR-encoded std_msgs/msg/String."""
    cdr = ensure_cdr_header(cdr)
    offset = 4  # skip CDR header
    str_len = struct.unpack_from("<I", cdr, offset)[0]
    offset += 4
    if str_len <= 1:
        return ""
    return cdr[offset : offset + str_len - 1].decode("utf-8")


def decode_cdr_uint8(cdr: bytes) -> int:
    """Decode CDR-encoded std_msgs/msg/UInt8."""
    cdr = ensure_cdr_header(cdr)
    return cdr[4]


def decode_cdr_joy(cdr: bytes) -> Joy:
    """Decode CDR-encoded sensor_msgs/msg/Joy."""
    cdr = ensure_cdr_header(cdr)
    msg = Joy()
    offset = 4  # skip CDR header

    # Header.stamp.sec (int32)
    sec = struct.unpack_from("<i", cdr, offset)[0]
    offset += 4
    # Header.stamp.nanosec (uint32)
    nanosec = struct.unpack_from("<I", cdr, offset)[0]
    offset += 4
    # Header.frame_id (CDR string)
    str_len = struct.unpack_from("<I", cdr, offset)[0]
    offset += 4
    frame_id = cdr[offset : offset + max(0, str_len - 1)].decode("utf-8") if str_len > 0 else ""
    offset += str_len
    # Align to 4 bytes
    offset = (offset + 3) & ~3

    # axes (float32[])
    axes_len = struct.unpack_from("<I", cdr, offset)[0]
    offset += 4
    # Unpack returns a tuple of floats
    axes = struct.unpack_from(f"<{axes_len}f", cdr, offset)
    offset += axes_len * 4

    # buttons (int32[])
    buttons_len = struct.unpack_from("<I", cdr, offset)[0]
    offset += 4
    # Unpack returns a tuple of ints
    buttons = struct.unpack_from(f"<{buttons_len}i", cdr, offset)

    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = frame_id
    msg.axes = list(axes)
    msg.buttons = list(buttons)
    return msg


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------

class OperatorRelayNode(Node):
    def __init__(self):
        super().__init__("operator_relay")

        # ROS publishers
        self.joy_pub = self.create_publisher(Joy, "/joy", 1)
        self.safety_pub = self.create_publisher(UInt8, "/safety", 1)
        self.master_pub = self.create_publisher(String, "/master_remop_message", 1)

        # Logging state
        self._last_joy_log = 0.0
        self._last_safety_state: int | None = None

        # Fork position tracking (from robot)
        self.fork_position = 0.0
        self.fork_sub = self.create_subscription(
            Float32, "/fork_position", self._on_fork_position, 1
        )

        # Heights storage
        self._heights_lock = threading.Lock()
        self.heights: dict[str, float] = self._load_heights()
        self.selected_target_height: float | None = None

        # Publish available heights every 2s
        self.create_timer(2.0, self._publish_heights_tick)

        # Adamo session
        self.get_logger().info("Connecting to adamo network...")
        self.adamo_session = adamo.connect(api_key=API_KEY)
        self.get_logger().info(f"Connected (org={self.adamo_session.org})")

        # Adamo publisher for height state feedback to widget UI
        self.height_state_pub = self.adamo_session.publisher(
            f"{ROBOT_NAME}/status/json/fork_height"
        )

        # Subscribe to specific control topics for this robot
        # We subscribe specifically to avoid the overhead of a wildcard subscription
        self._subs = []

        self._subs.append(self._create_sub(
            f"{ROBOT_NAME}/control/joy", self._handle_joy
        ))
        self._subs.append(self._create_sub(
            f"{ROBOT_NAME}/control/cdr/safety", self._handle_safety
        ))
        self._subs.append(self._create_sub(
            f"{ROBOT_NAME}/control/cdr/operator", self._handle_layout
        ))
        self._subs.append(self._create_sub(
            f"{ROBOT_NAME}/control/json/widget/height", self._handle_height_command
        ))

        # Message counters for heartbeat
        self._msg_counts = {"joy": 0, "safety": 0, "layout": 0, "height": 0}
        self.create_timer(5.0, self._heartbeat)

        self.get_logger().info(f"[DEBUG] Operator relay running — {len(self._subs)} active subscriptions:")
        for topic in [
            f"{ROBOT_NAME}/control/joy",
            f"{ROBOT_NAME}/control/cdr/safety",
            f"{ROBOT_NAME}/control/cdr/operator",
            f"{ROBOT_NAME}/control/json/widget/height",
        ]:
            self.get_logger().info(f"  -> {topic}")

    def _create_sub(self, topic, handler):
        """Helper to create a subscriber with exception handling."""
        def callback(sample):
            try:
                handler(sample.payload)
            except Exception as e:
                self.get_logger().warning(f"Error handling {topic}: {e}")

        return self.adamo_session.subscribe(topic, callback=callback)

    # -- Adamo callbacks (run on zenoh threads) --------------------------------

    def _handle_joy(self, payload: bytes):
        _topic, _type_name, cdr = parse_envelope(payload)
        joy_msg = decode_cdr_joy(cdr)
        self.joy_pub.publish(joy_msg)

        now = time.monotonic()
        if now - self._last_joy_log >= JOY_LOG_INTERVAL:
            self._last_joy_log = now
            axes_str = " ".join(f"{a:+.2f}" for a in joy_msg.axes)
            btns_str = "".join(str(b) for b in joy_msg.buttons)
            self.get_logger().info(f"[joy] axes=[{axes_str}] btns=[{btns_str}]")

    def _handle_safety(self, payload: bytes):
        _topic, _type_name, cdr = parse_envelope(payload)
        msg = UInt8()
        msg.data = decode_cdr_uint8(cdr)
        self.safety_pub.publish(msg)

        state = msg.data
        if state != self._last_safety_state:
            self.get_logger().info(
                f"[safety] state={state}"
                + (f" (was {self._last_safety_state})" if self._last_safety_state is not None else " (init)")
            )
            self._last_safety_state = state

    def _handle_layout(self, payload: bytes):
        self._msg_counts["layout"] += 1
        self.get_logger().info(f"[layout] raw {len(payload)}B: {payload[:64].hex()}")
        _topic, _type_name, cdr = parse_envelope(payload)
        layout_str = decode_cdr_string(cdr)
        # The string is already JSON like {"layout": "name"} - extract the value
        try:
            parsed = json.loads(layout_str)
            value = parsed.get("layout", layout_str)
        except json.JSONDecodeError:
            value = layout_str
        msg = String()
        msg.data = json.dumps({"layout": value})
        self.get_logger().info(f"[layout] -> {msg.data}")
        self.master_pub.publish(msg)

    def _handle_height_command(self, payload: bytes):
        self._msg_counts["height"] += 1
        self.get_logger().info(f"[height] raw {len(payload)}B: {payload.decode('utf-8', errors='replace')}")
        try:
            cmd = json.loads(payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        action = cmd.get("cmd")
        name = cmd.get("name", "")

        with self._heights_lock:
            if action == "save":
                self.heights[name] = self.fork_position
                self._save_heights()
                self.get_logger().info(
                    f"Saved height '{name}' = {self.fork_position:.3f}"
                )
            elif action == "delete":
                removed = self.heights.pop(name, None)
                if removed is not None:
                    self._save_heights()
                    self.get_logger().info(f"Deleted height '{name}'")
            elif action == "goto":
                if name in self.heights:
                    self.selected_target_height = self.heights[name]
                    msg = String()
                    msg.data = json.dumps(
                        {"selected target height": self.selected_target_height}
                    )
                    self.master_pub.publish(msg)
                    self.get_logger().info(
                        f"Selected target height '{name}' = {self.selected_target_height:.3f}"
                    )

            self._publish_height_state()

    def _heartbeat(self):
        counts = self._msg_counts
        self.get_logger().info(
            f"[heartbeat] msgs received — layout:{counts['layout']} height:{counts['height']} "
            f"joy:{counts['joy']} safety:{counts['safety']}"
        )

    # -- ROS callbacks ---------------------------------------------------------

    def _on_fork_position(self, msg: Float32):
        self.fork_position = msg.data

    # -- Periodic tasks --------------------------------------------------------

    def _publish_heights_tick(self):
        with self._heights_lock:
            self._publish_height_state()

    def _publish_height_state(self):
        """Publish current height state to adamo for the operator widget."""
        state = {
            "heights": self.heights,
            "current_height": self.fork_position,
        }
        self.height_state_pub.put(json.dumps(state).encode())

    # -- Disk persistence ------------------------------------------------------

    def _load_heights(self) -> dict[str, float]:
        if os.path.exists(HEIGHTS_FILE):
            try:
                with open(HEIGHTS_FILE) as f:
                    data = json.load(f)
                self.get_logger().info(
                    f"Loaded {len(data)} heights from {HEIGHTS_FILE}"
                )
                return data
            except (json.JSONDecodeError, IOError) as e:
                self.get_logger().warning(f"Failed to load heights: {e}")
        return {}

    def _save_heights(self):
        try:
            with open(HEIGHTS_FILE, "w") as f:
                json.dump(self.heights, f, indent=2)
        except IOError as e:
            self.get_logger().error(f"Failed to save heights: {e}")

    # -- Lifecycle -------------------------------------------------------------

    def destroy_node(self):
        for sub in self._subs:
            sub.close()
        self.height_state_pub.close()
        self.adamo_session.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = OperatorRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
