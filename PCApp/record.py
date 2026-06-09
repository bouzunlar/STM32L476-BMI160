"""
Standalone CSV recorder for BMI160 telemetry.

Captures binary frames from the STM32 serial stream and writes every parsed
sample to a CSV. The resulting file feeds replay.py for offline detection
algorithm validation.

Usage:
    python3 record.py --port /dev/tty.usbmodem11303 --output clean_circles.csv
    python3 record.py --port COM5 --output mixed.csv --duration 30

Defaults:
    baud      = 115200
    duration  = run until Ctrl+C (no explicit limit)

The CSV includes every Sample field used by detection so the offline replay
can be cycle-accurate with the firmware:
    seq, rawAcc{X,Y,Z}, rawGyr{X,Y,Z},
    fAcc{X,Y,Z} (earth-frame linear accel, g),
    fGyr{X,Y,Z} (body-frame gyro, dps),
    q0, q1, q2, q3, yawRateDps, yawCum, fusionHealthy, flags
"""

import argparse
import csv
import sys
import time

import serial

from protocol import FrameParser


CSV_HEADER = [
    "seq",
    "rawAccX", "rawAccY", "rawAccZ",
    "rawGyrX", "rawGyrY", "rawGyrZ",
    "fAccX",   "fAccY",   "fAccZ",
    "fGyrX",   "fGyrY",   "fGyrZ",
    "q0", "q1", "q2", "q3",
    "yawRateDps", "yawCum",
    "fusionHealthy",
    "flags",
]


def sample_to_row(s):
    """Flatten a Sample dataclass into a CSV row."""
    return [
        s.seq,
        *s.raw_acc,           # rawAcc XYZ
        *s.raw_gyr,           # rawGyr XYZ
        *s.f_acc,             # fAcc XYZ (earth-frame linear, g)
        *s.f_gyr,             # fGyr XYZ (body-frame gyro, dps)
        *s.quat,              # q0..q3
        s.yaw_rate_dps,
        s.yaw_cum,
        int(s.fusion_healthy),
        s.flags,
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description="BMI160 telemetry CSV recorder")
    ap.add_argument("--port", required=True,
                    help="Serial port, e.g. /dev/tty.usbmodem11303 or COM5")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    ap.add_argument("--output", required=True,
                    help="Output CSV file name")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Recording duration in seconds (0 = until Ctrl+C)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
    except Exception as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    parser = FrameParser()
    written = 0
    start   = time.time()
    last_progress = start

    print(f"Recording → {args.output}")
    if args.duration > 0:
        print(f"  Auto-stop after {args.duration:.1f} s. Or Ctrl+C any time.")
    else:
        print("  Press Ctrl+C to stop.")
    print()

    try:
        with open(args.output, "w", newline="") as fcsv:
            w = csv.writer(fcsv)
            w.writerow(CSV_HEADER)

            while True:
                # Duration check
                now = time.time()
                if args.duration > 0 and (now - start) >= args.duration:
                    print(f"\nDuration reached ({args.duration:.1f} s).")
                    break

                chunk = ser.read(256)
                if not chunk:
                    continue

                for sample in parser.feed(chunk):
                    w.writerow(sample_to_row(sample))
                    written += 1

                # Console progress once a second
                if (now - last_progress) >= 1.0:
                    elapsed = now - start
                    rate    = written / elapsed if elapsed > 0 else 0.0
                    flag_count = sum(
                        1 for _ in []  # placeholder: see end-of-run stats
                    )
                    print(f"  t={elapsed:6.1f} s  samples={written:6d}  "
                          f"rate={rate:5.1f} Hz  "
                          f"frames_ok={parser.stats['frames_ok']}  "
                          f"crc={parser.stats['crc_errors']}  "
                          f"lost={parser.stats['lost_packets']}",
                          end="\r")
                    last_progress = now

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        ser.close()

    elapsed = time.time() - start
    print(f"\n\nSaved {written} samples ({elapsed:.1f} s) → {args.output}")
    print(f"Parser stats: {parser.stats}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
