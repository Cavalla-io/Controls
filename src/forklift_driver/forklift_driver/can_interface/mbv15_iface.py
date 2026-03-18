import can
import struct
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from forklift_config import MBV15Config


class MBV15Interface:
    def __init__(
        self,
        channel="can0",
        bitrate=250000,
        is_mock=False,
        mbv15_config: "MBV15Config",
    ):
        self.cfg = mbv15_config

        self.connected = False
        self.bus = None

        if not is_mock:
            try:
                self.bus = can.interface.Bus(channel=channel, bustype="socketcan", bitrate=bitrate)
                self.connected = True
            except OSError as e:
                print(f"[MBV15] CAN Bus error: {e}. Running in disconnected mode.")

        self.node_id = self.cfg.node_id
        self.rpdo1_id = 0x200 + self.node_id
        self.rpdo2_id = 0x300 + self.node_id
        self.tpdo1_id = 0x180 + self.node_id
        self.tpdo2_id = 0x280 + self.node_id
        self.tpdo3_id = 0x380 + self.node_id

        self.state = {
            "motor_rpm": 0,
            "current_amps": 0.0,
            "steering_rad": 0.0,
            "battery_percent": 0,
            "estop_active": True,
            "drive_fault": 0,
            "steer_fault": 0,
        }

    def send_commands(self, drive_norm, steer_norm, lift_norm, tilt_norm, shift_norm, accel_s=1.0, decel_s=1.0):
        if not self.connected:
            return

        c = self.cfg
        flags = 0x01  # Bit 0: Interlock

        if drive_norm > c.drive_deadband:
            flags |= 1 << 1
        elif drive_norm < -c.drive_deadband:
            flags |= 1 << 2

        is_lowering = lift_norm < -c.lift_lower_threshold
        is_lifting = lift_norm > c.lift_lower_threshold
        is_aux = abs(tilt_norm) > c.aux_threshold or abs(shift_norm) > c.aux_threshold

        if is_lowering:
            flags |= 1 << 4
        elif is_lifting:
            flags |= 1 << 5

        rpm = int(abs(drive_norm) * c.max_drive_rpm)
        rpm = max(0, min(rpm, c.max_drive_rpm))

        accel = max(0, min(255, int(accel_s * 10)))
        decel = max(0, min(255, int(decel_s * 10)))

        steer_can = int(steer_norm * c.max_steer_can)
        steer_can = max(-c.max_steer_can, min(c.max_steer_can, steer_can))

        lowering_pwm = 0
        if is_lowering:
            raw_val = abs(lift_norm)
            pwm_val = c.lowering_pwm_min + (raw_val * (c.lowering_pwm_max - c.lowering_pwm_min))
            lowering_pwm = int(pwm_val)
            lowering_pwm = max(0, min(lowering_pwm, c.lowering_pwm_max))

        data1 = struct.pack("< B H B B h B", flags, rpm, accel, decel, steer_can, lowering_pwm)
        self.bus.send(can.Message(arbitration_id=self.rpdo1_id, data=data1, is_extended_id=False))

        valve_flags = 0x00
        if tilt_norm > c.aux_threshold:
            valve_flags |= 1 << 0
        elif tilt_norm < -c.aux_threshold:
            valve_flags |= 1 << 1
        if shift_norm > c.aux_threshold:
            valve_flags |= 1 << 2
        elif shift_norm < -c.aux_threshold:
            valve_flags |= 1 << 3

        pump_rpm = 0
        if is_lifting:
            pump_rpm = c.pump_rpm_base + int(lift_norm * c.pump_rpm_scale)
        elif is_aux:
            pump_rpm = c.pump_rpm_aux
        if is_lowering:
            pump_rpm = 0
        pump_rpm = max(0, min(c.pump_rpm_max, pump_rpm))

        data2 = struct.pack("< B B B H B B B", valve_flags, 0, 0, pump_rpm, 5, 5, 0)
        self.bus.send(can.Message(arbitration_id=self.rpdo2_id, data=data2, is_extended_id=False))

    def stop_all(self):
        if not self.connected:
            return
        self.bus.send(
            can.Message(arbitration_id=self.rpdo1_id, data=struct.pack("< B H B B h B", 0, 0, 0, 0, 0, 0))
        )
        self.bus.send(
            can.Message(arbitration_id=self.rpdo2_id, data=struct.pack("< B B B H B B B", 0, 0, 0, 0, 5, 5, 0))
        )

    def read_feedback(self, timeout=0.0):
        if not self.connected:
            return self.state
        while True:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                break
            if msg.arbitration_id == self.tpdo1_id:
                self._parse_tpdo1(msg.data)
            elif msg.arbitration_id == self.tpdo2_id:
                self._parse_tpdo2(msg.data)
            elif msg.arbitration_id == self.tpdo3_id:
                self._parse_tpdo3(msg.data)
        return self.state

    def _parse_tpdo1(self, data):
        if len(data) < 8:
            return
        rpm, current_raw, fault, batt, temp = struct.unpack("< h H B B h", data)
        self.state["motor_rpm"] = rpm
        self.state["current_amps"] = current_raw * 0.1
        self.state["drive_fault"] = fault
        self.state["battery_percent"] = batt

    def _parse_tpdo2(self, data):
        if len(data) < 8:
            return
        odom, pump_io, ctrl_io, drive_sw, drive_out = struct.unpack("< I B B B B", data)
        self.state["estop_active"] = (drive_sw & (1 << 0)) == 0

    def _parse_tpdo3(self, data):
        if len(data) < 8:
            return
        angle_raw, steer_fault, _ = struct.unpack("< h B 5s", data)
        self.state["steering_rad"] = math.radians(angle_raw / 100.0)
        self.state["steer_fault"] = steer_fault
