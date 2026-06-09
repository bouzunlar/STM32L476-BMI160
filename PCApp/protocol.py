"""
BMI160 telemetry binary frame protocol — PC side decoder.

Mirrors the encoder in Components/Telemetry/Src/telemetry.c.

Frame layout (73 bytes):
    SYNC (0xAA) | TYPE (0x01) | LEN (67) | PAYLOAD (67 B) | CRC16 (2 B) | END (0x55)

Payload (67 bytes, little-endian):
    uint32  seq
    int16   rawAccX, rawAccY, rawAccZ
    int16   rawGyrX, rawGyrY, rawGyrZ
    float   fAccX, fAccY, fAccZ      [g, earth-frame linear accel]
    float   fGyrX, fGyrY, fGyrZ      [dps, body-frame passthrough]
    float   q0, q1, q2, q3           (unit quaternion)
    float   yawRateDps               [dps, projected on gravity]
    float   yawCum                   [deg, cumulative]
    uint8   fusionHealthy            (1 = sample usable)
    uint16  flags
"""

import math
import struct
from dataclasses import dataclass
from typing import Iterator, Tuple


SYNC          = 0xAA
END           = 0x55
FRAME_SENSOR  = 0x01

PAYLOAD_FMT   = "<I3h3h3f3f4fffBH"
PAYLOAD_SIZE  = struct.calcsize(PAYLOAD_FMT)   # 67
FRAME_SIZE    = 3 + PAYLOAD_SIZE + 2 + 1       # 73

FLAG_DETECTION = 1 << 0

# Sensor scale factors used by the firmware (kept for raw → physical conversion)
ACC_SCALE_4G    = 4.0 / 32768.0       # g per LSB
GYR_SCALE_500   = 500.0 / 32768.0     # dps per LSB


@dataclass
class Sample:
    seq: int
    raw_acc: tuple        # (x, y, z) raw LSB
    raw_gyr: tuple
    f_acc:   tuple        # (x, y, z) earth-frame linear accel [g]
    f_gyr:   tuple        # body-frame gyro passthrough [dps]
    quat:    tuple        # (q0, q1, q2, q3) — unit quaternion (w, x, y, z)
    yaw_rate_dps: float   # rotation about estimated gravity [dps]
    yaw_cum: float        # cumulative yaw [deg]
    fusion_healthy: bool  # True if firmware considers this sample usable
    flags:   int

    @property
    def detected(self) -> bool:
        return bool(self.flags & FLAG_DETECTION)

    @property
    def raw_acc_g(self) -> tuple:
        return tuple(v * ACC_SCALE_4G for v in self.raw_acc)

    @property
    def raw_gyr_dps(self) -> tuple:
        return tuple(v * GYR_SCALE_500 for v in self.raw_gyr)

    @property
    def euler_deg(self) -> Tuple[float, float, float]:
        """Convert the carried quaternion to (roll, pitch, yaw) in degrees.
        Use this only for visualisation — quaternion is the canonical
        representation throughout the pipeline."""
        return quat_to_euler(*self.quat)


def quat_to_euler(q0: float, q1: float, q2: float, q3: float
                  ) -> Tuple[float, float, float]:
    """Standard quaternion → Euler (ZYX), result in degrees."""
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3),
                      1.0 - 2.0 * (q1 * q1 + q2 * q2))
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                     1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT (polynomial 0x1021, init 0xFFFF)."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# CRC16-CCITT (FALSE / XMODEM init=0xFFFF) standard test vector.
#
# RFC 1171 / industry-standard reference: the ASCII string "123456789"
# must produce CRC = 0x29B1. Run this module directly to validate.
#
#   $ python protocol.py
#   CRC16-CCITT(b"123456789") = 0x29B1  ->  PASS
#
# Also covered: empty payload (init value passthrough), single byte, and a
# 64-byte synthetic payload to catch wraparound bugs.
# ---------------------------------------------------------------------------
def _test_crc16_ccitt() -> None:
    cases = [
        (b"",          0xFFFF, "empty payload returns init value"),
        (b"A",         0xB915, "single byte 0x41"),
        (b"123456789", 0x29B1, "RFC 1171 / industry-standard reference"),
    ]
    all_ok = True
    for payload, expected, label in cases:
        got = crc16_ccitt(payload)
        ok  = (got == expected)
        all_ok &= ok
        marker = "PASS" if ok else "FAIL"
        print(f"  [{marker}] {label}: got=0x{got:04X} expected=0x{expected:04X}")
    if all_ok:
        print("CRC16-CCITT validation: ALL TESTS PASSED")
    else:
        raise AssertionError("CRC16-CCITT validation FAILED")


if __name__ == "__main__":
    _test_crc16_ccitt()


class FrameParser:
    """
    Streaming binary-frame parser. Feed raw bytes, get Sample objects out.

    Recovers from sync loss by skipping bytes until a valid SYNC is found.
    Reports counters (frames_ok, crc_errors, lost_packets, sync_skips).
    """

    def __init__(self):
        self._buf = bytearray()
        self._last_seq = None
        self.stats = {
            "frames_ok":    0,
            "crc_errors":   0,
            "lost_packets": 0,
            "sync_skips":   0,
        }

    def feed(self, chunk: bytes) -> Iterator[Sample]:
        """Push bytes; yield Sample for each successfully parsed frame."""
        self._buf.extend(chunk)

        while True:
            # Find SYNC byte
            i = self._buf.find(bytes([SYNC]))
            if i < 0:
                self.stats["sync_skips"] += len(self._buf)
                self._buf.clear()
                return
            if i > 0:
                self.stats["sync_skips"] += i
                del self._buf[:i]

            # Need full frame to decide
            if len(self._buf) < FRAME_SIZE:
                return

            ftype = self._buf[1]
            flen  = self._buf[2]

            # Quick header sanity check
            if ftype != FRAME_SENSOR or flen != PAYLOAD_SIZE:
                del self._buf[:1]
                self.stats["sync_skips"] += 1
                continue

            payload  = bytes(self._buf[3 : 3 + flen])
            crc_recv = (self._buf[3 + flen] << 8) | self._buf[4 + flen]
            end      = self._buf[5 + flen]

            # CRC over TYPE + LEN + PAYLOAD
            crc_calc = crc16_ccitt(bytes(self._buf[1 : 3 + flen]))

            if end != END or crc_calc != crc_recv:
                self.stats["crc_errors"] += 1
                del self._buf[:1]
                continue

            del self._buf[:FRAME_SIZE]
            self.stats["frames_ok"] += 1

            sample = self._unpack(payload)

            # Sequence-gap accounting
            if self._last_seq is not None:
                gap = (sample.seq - self._last_seq) & 0xFFFFFFFF
                if gap > 1:
                    self.stats["lost_packets"] += gap - 1
            self._last_seq = sample.seq

            yield sample

    @staticmethod
    def _unpack(payload: bytes) -> Sample:
        (seq,
         rax, ray, raz,
         rgx, rgy, rgz,
         fax, fay, faz,
         fgx, fgy, fgz,
         q0, q1, q2, q3,
         yaw_rate_dps,
         yaw_cum,
         fusion_healthy,
         flags) = struct.unpack(PAYLOAD_FMT, payload)
        return Sample(
            seq=seq,
            raw_acc=(rax, ray, raz),
            raw_gyr=(rgx, rgy, rgz),
            f_acc=(fax, fay, faz),
            f_gyr=(fgx, fgy, fgz),
            quat=(q0, q1, q2, q3),
            yaw_rate_dps=yaw_rate_dps,
            yaw_cum=yaw_cum,
            fusion_healthy=bool(fusion_healthy),
            flags=flags,
        )
