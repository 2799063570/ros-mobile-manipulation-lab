"""Wire protocol shared with RemoteProtocol.c. All multibyte values are little endian."""
import struct
from dataclasses import dataclass

MAGIC = b'\xa5\x5a'
VERSION = 2
MAX_PAYLOAD = 32
POLL, INPUT, FEEDBACK = 1, 2, 3
SOURCE_NONE, SOURCE_ROS, SOURCE_DEMO = 0, 1, 2
INPUT_STRUCT = struct.Struct('<IHHhhBBBBH')
FEEDBACK_STRUCT = struct.Struct('<IBB6hHBBBB')
JOINT_NAMES = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
               'wrist1_joint', 'wrist2_joint', 'wrist3_joint')


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xffff
    return crc


@dataclass(frozen=True)
class Frame:
    kind: int
    sequence: int
    payload: bytes

    def encode(self):
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError('payload exceeds 32 bytes')
        body = struct.pack('<BBHB', VERSION, self.kind, self.sequence, len(self.payload)) + self.payload
        return MAGIC + body + struct.pack('<H', crc16(body))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(MAGIC)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == MAGIC[:1] else b''
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 7:
                break
            version, kind, sequence, length = struct.unpack_from('<BBHB', self.buffer, 2)
            if version != VERSION or length > MAX_PAYLOAD:
                self.errors += 1
                del self.buffer[0]
                continue
            total = length + 9
            if len(self.buffer) < total:
                break
            expected, = struct.unpack_from('<H', self.buffer, total - 2)
            if expected != crc16(self.buffer[2:total - 2]):
                self.errors += 1
                del self.buffer[0]
                continue
            frames.append(Frame(kind, sequence, bytes(self.buffer[7:total - 2])))
            del self.buffer[:total]
        return frames


@dataclass(frozen=True)
class InputState:
    raw_x: int
    raw_y: int
    x: int
    y: int
    button: int
    flags: int
    page: int
    command: int = 0
    request_id: int = 0

    @property
    def valid(self):
        return self.flags == 3

    @property
    def axes(self):
        return (self.x / 1000.0, self.y / 1000.0) if self.valid else (0.0, 0.0)


def decode_input(frame, session):
    if frame.kind != INPUT or len(frame.payload) != INPUT_STRUCT.size:
        raise ValueError('wrong input type or length')
    actual_session, rx, ry, x, y, button, flags, page, command, request_id = INPUT_STRUCT.unpack(frame.payload)
    if (actual_session != session or not 0 <= rx <= 4095 or not 0 <= ry <= 4095
            or not -1000 <= x <= 1000 or not -1000 <= y <= 1000
            or button not in (0, 1) or not 0 <= flags <= 3 or not 0 <= page <= 4 or not 0 <= command <= 3):
        raise ValueError('invalid input fields')
    if flags != 3 and (x or y or button):
        raise ValueError('uncalibrated/invalid ADC must send neutral input')
    return InputState(rx, ry, x, y, button, flags, page, command, request_id)


def feedback_frame(session, sequence, source, mask, joints, ack_id=0, ack_command=0, ack_state=0, motion_state=10, target=0):
    if source not in (SOURCE_NONE, SOURCE_ROS, SOURCE_DEMO) or not 0 <= mask <= 63:
        raise ValueError('invalid feedback source/mask')
    if source == SOURCE_NONE and mask:
        raise ValueError('missing source cannot supply valid joints')
    return Frame(FEEDBACK, sequence, FEEDBACK_STRUCT.pack(session, source, mask, *joints, ack_id, ack_command, ack_state, motion_state, target))
