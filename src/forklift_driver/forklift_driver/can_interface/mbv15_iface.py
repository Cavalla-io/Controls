import can
import struct
import math
import time

class MBV15Interface:
    def __init__(self, channel='can0', bitrate=250000, is_mock=False):
        """
        Hardware interface for the MBV15 Forklift (Curtis Controller).
        """
        self.connected = False
        self.bus = None
        
        if not is_mock:
            try:
                self.bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=bitrate)
                self.connected = True
            except OSError as e:
                print(f"[MBV15] CAN Bus error: {e}. Running in disconnected mode.")

        # CANopen Node ID Setup
        self.node_id = 0x03
        
        # Command IDs (RPDO)
        self.rpdo1_id = 0x200 + self.node_id  # 0x203 (Motion)
        self.rpdo2_id = 0x300 + self.node_id  # 0x303 (Hydraulics)
        
        # Feedback IDs (TPDO)
        self.tpdo1_id = 0x180 + self.node_id  # 0x183 (Drive Feedback)
        self.tpdo2_id = 0x280 + self.node_id  # 0x283 (Status Feedback)
        self.tpdo3_id = 0x380 + self.node_id  # 0x383 (Steering Feedback)

        # Internal State Dictionary (Updated by TPDOs)
        self.state = {
            'motor_rpm': 0,
            'current_amps': 0.0,
            'steering_rad': 0.0,
            'battery_percent': 0,
            'estop_active': True,  # Default to True for safety until we hear otherwise
            'drive_fault': 0,
            'steer_fault': 0
        }

    # ==========================================
    # DOWNSTREAM: SENDING COMMANDS (RPDO)
    # ==========================================

    def send_motion(self, drive_speed_norm, steering_rad, lift_speed_norm):
        """
        Packs and sends RPDO1 (Motion Control)
        """
        if not self.connected: return

        # Byte 0: Flags
        flags = 0x01  # Bit 0: enable_interlock_brake_release
        if drive_speed_norm > 0.02:   flags |= 0x02  # Bit 1: enable_forward
        elif drive_speed_norm < -0.02: flags |= 0x04  # Bit 2: enable_reverse

        # Bytes 1-2: Target RPM
        rpm = int(abs(drive_speed_norm) * 4000)
        rpm = max(0, min(rpm, 4000))

        # Bytes 3-4: Accel / Decel (0.1s scale) -> 10 = 1.0 second
        accel = 10
        decel = 10

        # Bytes 5-6: Steering Angle (0.01 degree scale)
        steer_deg = math.degrees(steering_rad)
        steer_can = int(steer_deg * 100)
        steer_can = max(-12000, min(12000, steer_can))

        # Byte 7: Lowering Valve PWM
        lowering_pwm = 0
        if lift_speed_norm < -0.05:
            lowering_pwm = int(abs(lift_speed_norm) * 200)
            lowering_pwm = max(0, min(lowering_pwm, 200))

        # Pack: uint8, uint16, uint8, uint8, int16, uint8 (Little Endian)
        data = struct.pack('< B H B B h B', flags, rpm, accel, decel, steer_can, lowering_pwm)
        self.bus.send(can.Message(arbitration_id=self.rpdo1_id, data=data, is_extended_id=False))

    def send_hydraulics(self, lift_speed_norm, tilt_norm, shift_norm):
        """
        Packs and sends RPDO2 (Hydraulics Control)
        """
        if not self.connected: return

        # Byte 0: Valve Flags (Map these to your specific manifold)
        valve_flags = 0x00
        if tilt_norm > 0.5:   valve_flags |= (1 << 0)  # valve_1l
        elif tilt_norm < -0.5: valve_flags |= (1 << 1)  # valve_1r
        
        if shift_norm > 0.5:   valve_flags |= (1 << 2)  # valve_2l
        elif shift_norm < -0.5: valve_flags |= (1 << 3)  # valve_2r

        # Only spin the pump if a hydraulic action is actively requested
        pump_rpm = 0
        if abs(lift_speed_norm) > 0.05 or abs(tilt_norm) > 0.5 or abs(shift_norm) > 0.5:
            pump_rpm = 3000

        # Pack: uint8, uint8, uint8, uint16, uint8, uint8, uint8
        data = struct.pack('< B B B H B B B', valve_flags, 0, 0, pump_rpm, 5, 5, 0)
        self.bus.send(can.Message(arbitration_id=self.rpdo2_id, data=data, is_extended_id=False))

    def stop_all(self):
        """
        Immediately zero-out all movement.
        """
        if not self.connected: return
        self.bus.send(can.Message(
            arbitration_id=self.rpdo1_id, 
            data=struct.pack('< B H B B h B', 0, 0, 0, 0, 0, 0), 
            is_extended_id=False
        ))

    # ==========================================
    # UPSTREAM: READING FEEDBACK (TPDO)
    # ==========================================

    def read_feedback(self, timeout=0.01):
        """
        Polls the CAN bus for new messages and updates the internal state.
        Returns the updated state dictionary.
        """
        if not self.connected: return self.state

        # Drain the CAN buffer of all available messages within the timeout
        while True:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                break  # No more messages

            if msg.arbitration_id == self.tpdo1_id:
                self._parse_tpdo1(msg.data)
            elif msg.arbitration_id == self.tpdo2_id:
                self._parse_tpdo2(msg.data)
            elif msg.arbitration_id == self.tpdo3_id:
                self._parse_tpdo3(msg.data)

        return self.state

    def _parse_tpdo1(self, data):
        """Drive Feedback (0x183)"""
        if len(data) < 8: return
        rpm, current_raw, fault, batt, temp = struct.unpack('< h H B B h', data)
        self.state['motor_rpm'] = rpm
        self.state['current_amps'] = current_raw * 0.1
        self.state['battery_percent'] = batt
        self.state['drive_fault'] = fault

    def _parse_tpdo2(self, data):
        """Status Feedback (0x283)"""
        if len(data) < 8: return
        odom, pump_io, ctrl_io, drive_sw, drive_out = struct.unpack('< I B B B B', data)
        
        # E-Stop is bit 0 of drive_sw (Active Low: 0 means E-Stop is pressed)
        estop_status = (drive_sw & (1 << 0)) == 0
        self.state['estop_active'] = estop_status

    def _parse_tpdo3(self, data):
        """Steering Feedback (0x383)"""
        if len(data) < 8: return
        angle_raw, steer_fault, _ = struct.unpack('< h B 5s', data)
        
        # Convert back from 0.01 degree scale to radians
        angle_deg = angle_raw / 100.0
        self.state['steering_rad'] = math.radians(angle_deg)
        self.state['steer_fault'] = steer_fault