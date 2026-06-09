# PC-Side Tools — STM32 + BMI160 Circular-Motion Detection

These Python tools talk to the STM32 firmware over the ST-Link virtual COM port
(115200 bps) and read the 73-byte binary telemetry frame. There are four tools
that share one frame parser. Detection always runs on the STM32 — the PC only
visualises and validates.

## Setup

```bash
cd PCApp
python3 -m venv .venv
source .venv/bin/activate          # macOS / Linux
# .venv\Scripts\activate           # Windows

pip install -r requirements.txt    # pyserial + PyQt6
```

Find the serial port the Nucleo enumerates:

| OS      | Example                       |
| ------- | ----------------------------- |
| macOS   | `/dev/tty.usbmodem11103`      |
| Linux   | `/dev/ttyACM0`                |
| Windows | `COM5`                        |

> Only one program can open the serial port at a time — do not run the
> dashboard and the recorder together.

## The four tools

| File              | Purpose                                                        |
| ----------------- | -------------------------------------------------------------- |
| `protocol.py`     | Shared frame parser: SYNC/LEN/END + CRC16-CCITT, unpacks the 73-byte frame. Used by the others. Run it directly to validate the CRC against the standard vector. |
| `dashboard_qt.py` | Live text dashboard for the demo: values, detection gate states, cumulative arc, and a pop-up when a circle is detected. |
| `record.py`       | Records the telemetry stream to a CSV file.                    |
| `replay.py`       | Offline validator: runs a recorded CSV through a Python copy of the firmware detection algorithm (no board needed). |

## 1. Live dashboard (demo)

Build the firmware with `DIAG_TEST = 0` (binary telemetry), then:

```bash
python3 dashboard_qt.py --port /dev/tty.usbmodem11103
```

Draw a full circle → a `CIRCLE DETECTED` pop-up appears. Shake / half / quarter
circles do not trigger.

## 2. Record test motions

```bash
python3 record.py --port /dev/tty.usbmodem11103 --output clean.csv --duration 35
```

Stops after `--duration` seconds, or on Ctrl+C. Each CSV row is one sample
(seq, raw + fused accel/gyro, quaternion, yaw rate, fusion-healthy flag, flags).

## 3. Offline algorithm validation

`replay.py` runs the same detection logic as the firmware (`detection.c`:
64-sample window, same thresholds, accel-vector cumulative arc 360°).

```bash
# 5 development stages of the algorithm (default)
python3 replay.py clean.csv clean_paused.csv yaw_shake.csv quarter.csv \
                  half.csv half_plus.csv mixed.csv mixed_paused.csv

# STRICT / NORMAL / LOOSE tolerance-profile comparison
python3 replay.py --profiles clean.csv clean_paused.csv yaw_shake.csv quarter.csv \
                  half.csv half_plus.csv mixed.csv mixed_paused.csv
```

Expected counts (ground truth) are matched from the file name, so the tool
prints detections vs. expected for each scenario.

## 4. Validate the CRC implementation

```bash
python3 protocol.py
# -> CRC16-CCITT("123456789") = 0x29B1  PASS
```
