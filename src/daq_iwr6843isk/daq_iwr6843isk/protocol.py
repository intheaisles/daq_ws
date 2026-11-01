"""TI SDK 3.x legacy Out-of-Box packets; independent of ROS and serial ports."""

from dataclasses import dataclass
import struct

import numpy as np

MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'
HEADER = struct.Struct('<8s8I')
MAX_PACKET_BYTES = 1024 * 1024


@dataclass(frozen=True)
class RadarFrame:
    frame_number: int
    cpu_cycles: int
    subframe_number: int
    # x, y, z [m], doppler [m/s], snr [dB], noise [dB]
    points: np.ndarray
    version: int
    platform: int


def decode_packet(packet):
    if len(packet) < HEADER.size:
        raise ValueError('Truncated TI header')
    magic, version, length, platform, frame, cycles, count, tlvs, subframe = HEADER.unpack_from(packet)
    if magic != MAGIC or length != len(packet):
        raise ValueError('Invalid TI packet boundary')
    if subframe != 0:
        raise ValueError('Only legacy frameCfg (subframe 0) is supported')
    if count > (length - HEADER.size) // 16 or tlvs > (length - HEADER.size) // 8:
        raise ValueError('Impossible point/TLV count')
    points = np.full((count, 6), np.nan, dtype=np.float32)
    offset = HEADER.size
    found_points = False
    found_side = False
    for _ in range(tlvs):
        if offset + 8 > length:
            raise ValueError('Truncated TLV header')
        kind, payload_length = struct.unpack_from('<II', packet, offset)
        offset += 8
        end = offset + payload_length  # SDK 3.x OOB length excludes the TLV header.
        if end > length:
            raise ValueError('TLV extends past packet')
        if kind == 1:
            if found_points or payload_length != count * 16:
                raise ValueError('Invalid detected-points TLV')
            points[:, :4] = np.frombuffer(packet, dtype='<f4', count=count * 4,
                                         offset=offset).reshape(count, 4)
            found_points = True
        elif kind == 7:
            if found_side or payload_length != count * 4:
                raise ValueError('Invalid side-info TLV')
            points[:, 4:] = np.frombuffer(packet, dtype='<i2', count=count * 2,
                                         offset=offset).reshape(count, 2) * 0.1
            found_side = True
        offset = end
    if count and not found_points:
        raise ValueError('Header declares points but point TLV is missing')
    if not np.all(np.isfinite(points[:, :4])):
        raise ValueError('Invalid point coordinates/velocity')
    # Remaining bytes are SDK alignment padding, not another frame.
    return RadarFrame(frame, cycles, subframe, points, version, platform)


class PacketStream:
    """Reassemble partial/coalesced reads without replaying old frames."""

    def __init__(self):
        self.buffer = bytearray()
        self.malformed_packets = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                discard = max(0, len(self.buffer) - len(MAGIC) + 1)
                self.discarded_bytes += discard
                del self.buffer[:discard]
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            length = struct.unpack_from('<I', self.buffer, 12)[0]
            if not HEADER.size <= length <= MAX_PACKET_BYTES:
                self.malformed_packets += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                break
            packet = bytes(self.buffer[:length])
            try:
                frame = decode_packet(packet)
            except ValueError:
                self.malformed_packets += 1
                # Search for the next magic even if the bad length swallowed it.
                del self.buffer[0]
                continue
            del self.buffer[:length]
            frames.append(frame)
        return frames


class FrameSequence:
    def __init__(self):
        self.last = None
        self.missing = 0
        self.duplicates = 0
        self.discontinuities = 0

    def accept(self, number):
        if self.last is None:
            self.last = number
            return True, 0, False
        delta = (number - self.last) & 0xffffffff
        if delta == 0:
            self.duplicates += 1
            return False, 0, False
        reset = delta >= 0x80000000
        gap = 0 if reset else delta - 1
        if reset:
            self.discontinuities += 1
        self.missing += gap
        self.last = number
        return True, gap, reset


def config_commands(text):
    commands = [line.strip() for line in text.splitlines()
                if line.strip() and not line.lstrip().startswith(('%', '#'))]
    names = {line.split()[0] for line in commands}
    if not {'profileCfg', 'frameCfg', 'sensorStart'} <= names:
        raise ValueError('Config requires profileCfg, frameCfg and sensorStart')
    if 'advFrameCfg' in names or any(line.split()[:2] == ['dfeDataOutputMode', '3']
                                     for line in commands):
        raise ValueError('Advanced subframes are not supported')
    if commands[-1].split()[0] != 'sensorStart':
        raise ValueError('sensorStart must be the last config command')
    return commands


def config_summary(text):
    result = {}
    for line in config_commands(text):
        words = line.split()
        if words[0] == 'frameCfg':
            result['frame_period_ms'] = float(words[5])
            result['expected_hz'] = 1000.0 / float(words[5])
        elif words[0] == 'aoaFovCfg':
            result['aoa_fov_deg'] = {
                'min_azimuth': float(words[2]), 'max_azimuth': float(words[3]),
                'min_elevation': float(words[4]), 'max_elevation': float(words[5]),
            }
    return result
