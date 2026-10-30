"""Minimal Feetech STS bus driver (protocol 1) over pyserial."""
import threading

import serial

# STS3215 control table (RAM)
TORQUE_ENABLE = 40
ACCELERATION = 41
GOAL_POSITION = 42      # 2 bytes, followed by goal time (2) and goal speed (2)
PRESENT_POSITION = 56   # 2 bytes

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE
TICKS_PER_REV = 4096


def _packet(sid, instr, params=b''):
    body = bytes([sid, len(params) + 2, instr]) + params
    return b'\xff\xff' + body + bytes([(~sum(body)) & 0xFF])


class FeetechBus:
    """Thread-safe access to one serial bus of STS servos."""

    def __init__(self, port, baud=1000000, timeout=0.02):
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self._lock = threading.Lock()

    def close(self):
        self._ser.close()

    def _transact(self, sid, instr, params=b'', reply_len=6):
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(_packet(sid, instr, params))
            self._ser.flush()
            if sid == BROADCAST:
                return None
            resp = self._ser.read(reply_len)
        # Find the status frame from this servo (skips an echo of our own packet, if any)
        i = resp.find(b'\xff\xff')
        while i != -1:
            frame = resp[i:i + reply_len]
            if len(frame) == reply_len and frame[2] == sid and frame[4] != instr:
                if (~sum(frame[2:-1])) & 0xFF == frame[-1]:
                    return frame
            i = resp.find(b'\xff\xff', i + 1)
        return None

    def ping(self, sid):
        return self._transact(sid, PING) is not None

    def read_position(self, sid):
        """Raw position 0..4095, or None if the servo did not answer."""
        frame = self._transact(sid, READ, bytes([PRESENT_POSITION, 2]), reply_len=8)
        if frame is None:
            return None
        raw = frame[5] | (frame[6] << 8)
        return -(raw & 0x7FFF) if raw & 0x8000 else raw

    def sync_read_positions(self, ids):
        """{id: raw position} in one bus round trip; servos that did not answer are left out."""
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(_packet(BROADCAST, SYNC_READ, bytes([PRESENT_POSITION, 2, *ids])))
            self._ser.flush()
            resp = self._ser.read(8 * len(ids))
        positions = {}
        i = resp.find(b'\xff\xff')
        while i != -1:
            frame = resp[i:i + 8]
            valid = len(frame) == 8 and frame[2] in ids and frame[3] == 4
            if valid and (~sum(frame[2:-1])) & 0xFF == frame[-1]:
                raw = frame[5] | (frame[6] << 8)
                positions[frame[2]] = -(raw & 0x7FFF) if raw & 0x8000 else raw
                i = resp.find(b'\xff\xff', i + 8)
            else:
                i = resp.find(b'\xff\xff', i + 1)
        return positions

    def write_byte(self, sid, addr, value):
        return self._transact(sid, WRITE, bytes([addr, value & 0xFF])) is not None

    def set_torque(self, ids, enable):
        for sid in ids:
            self.write_byte(sid, TORQUE_ENABLE, 1 if enable else 0)

    def sync_write_positions(self, goals):
        """goals: {id: raw position}. Goal time and speed are 0, i.e. as fast as possible."""
        if not goals:
            return
        data = b''
        for sid, raw in goals.items():
            raw = max(0, min(TICKS_PER_REV - 1, int(round(raw))))
            data += bytes([sid, raw & 0xFF, raw >> 8, 0, 0, 0, 0])
        self._transact(BROADCAST, SYNC_WRITE, bytes([GOAL_POSITION, 6]) + data)
