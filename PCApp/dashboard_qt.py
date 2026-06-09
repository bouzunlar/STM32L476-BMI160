"""
Simple text dashboard for the BMI160 telemetry stream (PyQt6 variant).

Same idea as dashboard.py but built on PyQt6 to avoid the tkinter / libexpat
problem on Homebrew Python. PyQt6 is already a dependency of viewer.py via
pyqtgraph, so no extra install is needed.

Usage:
    python dashboard_qt.py --port /dev/tty.usbmodem*    (macOS / Linux)
    python dashboard_qt.py --port COM5                  (Windows)
"""

import argparse
import math
import sys
from collections import deque

import serial
from PyQt6 import QtCore, QtGui, QtWidgets

from protocol import FrameParser


BG       = "#FFFFFF"
FG       = "#222222"
ACCENT   = "#0B5FFF"
GREEN    = "#0A8A3A"
ALARM_BG = "#FFEFE0"
ALARM_FG = "#C63B00"

# --- Detection thresholds: MUST mirror Components/Detection/Inc/detection.h
#     (NORMAL profile). These are shown live next to the computed values so
#     you can see exactly which gate passes/fails for a given motion. -------
DET_WINDOW         = 64        # DETECTION_WINDOW_SIZE (matches firmware)
IMU_DT_SEC         = 0.01      # 100 Hz
YAW_RATE_MIN       = 150.0     # DET_YAW_RATE_MEAN_DPS_MIN
VAR_G2_MIN         = 0.011     # DET_ALIN_VAR_G2_MIN
VAR_G2_MAX         = 0.40      # DET_ALIN_VAR_G2_MAX (reject fast/violent arcs)
COV_K              = 0.65      # DET_COV_K_FACTOR
VAR_RATIO_MAX      = 3.0       # DET_VAR_RATIO_MAX
MIN_ZC             = 2         # DET_MIN_ZERO_CROSS_EACH_AXIS
QUARTERS_REQUIRED  = 4         # DET_YAW_QUARTERS_REQUIRED
ARC_MIN_DEG        = 130.0     # DET_MIN_ARC_DEG (raised for 64-sample window)
YAW_ABS_MEAN_MIN   = 50.0      # DET_YAW_ABS_MEAN_MIN_DPS (wrist-rotation gate)
CUM_ARC_THRESH     = 360.0     # DET_CUM_ARC_THRESHOLD_DEG (full revolution)

SETTLE_SECONDS     = 2.0       # "do not move" window while Mahony settles


def compute_gates(window):
    """Replicate detection.c's per-window math on the PC side so the dashboard
    can show each gate's value vs its threshold. `window` is a list of
    (fAccX, fAccY, yawRateDps). Returns a dict of computed metrics + pass
    flags. Mirrors Detection_Process exactly (NORMAL profile)."""
    n = len(window)
    if n < DET_WINDOW:
        return None

    # Direction consistency over quarters
    yaw_sum = sum(w[2] for w in window)
    quarter = [0.0, 0.0, 0.0, 0.0]
    for i, w in enumerate(window):
        quarter[i // (DET_WINDOW // 4)] += w[2]
    expected_sign = 1 if yaw_sum >= 0 else -1
    matching = 0
    for q in quarter:
        if abs(q) > 1e-3 and ((1 if q >= 0 else -1) == expected_sign):
            matching += 1
    dir_ok = matching >= QUARTERS_REQUIRED
    # Absolute mean |yaw| = wrist rotation activity (direction-independent).
    mean_abs_yaw = sum(abs(w[2]) for w in window) / n

    # Mean of earth-frame linear accel XY
    mean_x = sum(w[0] for w in window) / n
    mean_y = sum(w[1] for w in window) / n

    # Variance / covariance / zero-crossings
    var_x = var_y = cov_xy = 0.0
    zcx = zcy = 0
    for i, w in enumerate(window):
        dx = w[0] - mean_x
        dy = w[1] - mean_y
        var_x += dx * dx
        var_y += dy * dy
        cov_xy += dx * dy
        if i > 0:
            pdx = window[i-1][0] - mean_x
            pdy = window[i-1][1] - mean_y
            if (pdx > 0 and dx <= 0) or (pdx < 0 and dx >= 0):
                zcx += 1
            if (pdy > 0 and dy <= 0) or (pdy < 0 and dy >= 0):
                zcy += 1
    var_x /= n
    var_y /= n
    cov_xy /= n

    delta_psi = sum(w[2] * IMU_DT_SEC for w in window)
    min_v = min(var_x, var_y)
    max_v = max(var_x, var_y)
    var_ratio = max_v / (min_v + 1e-6)
    mean_yaw = abs(yaw_sum) / n

    shape_ok      = var_ratio <= VAR_RATIO_MAX
    not_straight  = (cov_xy * cov_xy) < (var_x * var_y * COV_K)
    zc_ok         = (zcx >= MIN_ZC) and (zcy >= MIN_ZC)
    arc_ok        = abs(delta_psi) >= ARC_MIN_DEG
    phase_ok      = arc_ok if MIN_ZC == 0 else (zc_ok or arc_ok)

    # Candidate is now LINEAR-ACCEL only (matches firmware): a 2D orbit with
    # balanced axes and enough energy. Yaw gates are kept for the trace but
    # no longer gate the candidate.
    candidate = (not_straight and shape_ok
                 and VAR_G2_MIN < var_x < VAR_G2_MAX
                 and VAR_G2_MIN < var_y < VAR_G2_MAX)

    return {
        "mean_yaw": mean_yaw, "mean_yaw_ok": mean_yaw > YAW_RATE_MIN,
        "var_x": var_x, "var_x_ok": var_x > VAR_G2_MIN,
        "var_y": var_y, "var_y_ok": var_y > VAR_G2_MIN,
        "var_ratio": var_ratio, "shape_ok": shape_ok,
        "cov2": cov_xy * cov_xy, "cov_lim": var_x * var_y * COV_K,
        "not_straight": not_straight,
        "matching": matching, "dir_ok": dir_ok,
        "zcx": zcx, "zcy": zcy, "arc": delta_psi, "phase_ok": phase_ok,
        "candidate": candidate, "expected_sign": expected_sign,
        "mean_abs_yaw": mean_abs_yaw,
    }


class _SerialWorker(QtCore.QObject):
    """Pumps the serial port in a background thread and emits parsed samples."""

    sample_received = QtCore.pyqtSignal(object, dict)   # (Sample, stats dict)

    def __init__(self, port: str, baud: int):
        super().__init__()
        self._port = port
        self._baud = baud
        self._stop = False

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            ser = serial.Serial(self._port, baudrate=self._baud, timeout=0.05)
        except Exception as exc:
            print(f"Serial open failed: {exc}", file=sys.stderr)
            return
        parser = FrameParser()
        while not self._stop:
            try:
                chunk = ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            for sample in parser.feed(chunk):
                self.sample_received.emit(sample, dict(parser.stats))

    def stop(self) -> None:
        self._stop = True


class Dashboard(QtWidgets.QMainWindow):
    """Text-only telemetry dashboard."""

    POPUP_LIFETIME_MS = 2000

    def __init__(self, port: str, baud: int):
        super().__init__()
        self.setWindowTitle("BMI160 Telemetry Dashboard")
        self.resize(720, 780)
        self.setStyleSheet(f"background-color: {BG};")

        self._detection_count = 0
        self._prev_det_flag   = False
        self._popup = None

        # Rolling window for live gate computation (mirrors firmware).
        self._win = deque(maxlen=DET_WINDOW)
        # PC-side cumulative-arc tracker (accel-vector angle integration,
        # matches firmware). Illustrative; firmware owns the real decision.
        self._cum_arc = 0.0
        self._last_accel_angle = None    # previous atan2(fAccY, fAccX), rad
        self._cum_arc_at_fire = 0.0      # cumArc captured when it crosses
        self._acc_sx = 0.0               # EMA-smoothed accel for the angle
        self._acc_sy = 0.0

        # Boot detection: the firmware resets its sample sequence to 0 on
        # power-up (Fusion_Init), so a drop in seq means the device just
        # (re)booted -- even while this dashboard stays open. We re-arm the
        # settle countdown on every detected boot.
        self._last_seq = None
        self._settle_remaining = 0.0
        self._settle_timer = QtCore.QTimer(self)
        self._settle_timer.timeout.connect(self._tick_settle)

        self._build_ui()
        self._start_worker(port, baud)

        # Until the first frame arrives, prompt the user to (re)connect/power.
        self._settle_lbl.setVisible(True)
        self._settle_lbl.setText("Waiting for device…")

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(24, 18, 24, 18)
        layout.setSpacing(6)

        # Title
        title = QtWidgets.QLabel("BMI160 — Real-Time Fusion & Detection")
        title.setStyleSheet(f"color: {ACCENT}; font: bold 20px 'Helvetica';")
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Settle banner -- shown for the first SETTLE_SECONDS so the user
        # keeps the device still while Mahony's PI integrator converges.
        self._settle_lbl = QtWidgets.QLabel("")
        self._settle_lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._settle_lbl.setStyleSheet(
            f"color: {ALARM_FG}; background-color: {ALARM_BG}; "
            f"font: bold 15px 'Helvetica'; padding: 6px; "
            f"border: 1px solid {ALARM_FG};")
        self._settle_lbl.setVisible(False)
        layout.addWidget(self._settle_lbl)
        layout.addSpacing(8)

        # Fusion section
        self._quat_lbl    = self._add_row(layout, "Quaternion (q0 q1 q2 q3)")
        self._yaw_lbl     = self._add_row(layout, "Yaw rate (gravity-projected)")
        self._yawc_lbl    = self._add_row(layout, "Yaw cumulative")
        self._health_lbl  = self._add_row(layout, "Fusion health")

        layout.addWidget(self._separator())

        # Motion section
        self._acc_lbl = self._add_row(layout, "Earth-frame accel (g) X Y Z")
        self._gyr_lbl = self._add_row(layout, "Body-frame gyro (dps) X Y Z")

        layout.addWidget(self._separator())

        layout.addWidget(self._separator())

        # ---- Detection gates (live) -----------------------------------
        # Each gate shows: computed value, the threshold, and a PASS/FAIL
        # mark. ALL must pass in the same window for it to be a "candidate";
        # the cumulative arc must then reach 360 deg for an alarm.
        gates_title = QtWidgets.QLabel("Detection gates (live window, 320 ms)")
        gates_title.setStyleSheet(f"color: {ACCENT}; font: bold 13px 'Helvetica';")
        layout.addWidget(gates_title)

        self._g_yaw   = self._add_row(layout, "  mean yaw rate")
        self._g_varx  = self._add_row(layout, "  var X")
        self._g_vary  = self._add_row(layout, "  var Y")
        self._g_ratio = self._add_row(layout, "  shape (varRatio)")
        self._g_cov   = self._add_row(layout, "  not-straight (cov²)")
        self._g_quart = self._add_row(layout, "  direction quarters")
        self._g_phase = self._add_row(layout, "  phase (arc / zc)")
        self._g_cum   = self._add_row(layout, "  cumulative arc")
        self._g_cand  = self._add_row(layout, "  >> window candidate?")

        layout.addWidget(self._separator())

        # Last-detection snapshot — ALL conditions frozen at the moment
        # FLAG_DETECTION went high, so you can read exactly which values
        # triggered the alarm.
        ld_title = QtWidgets.QLabel("Last detection — frozen conditions")
        ld_title.setStyleSheet(f"color: {ALARM_FG}; font: bold 13px 'Helvetica';")
        layout.addWidget(ld_title)

        self._last_seq_lbl   = self._add_row(layout, "  seq")
        self._last_cum_lbl   = self._add_row(layout, "  cumulative arc at trigger")
        self._last_yawr_lbl  = self._add_row(layout, "  mean yaw rate")
        self._last_varx_lbl  = self._add_row(layout, "  var X")
        self._last_vary_lbl  = self._add_row(layout, "  var Y")
        self._last_ratio_lbl = self._add_row(layout, "  shape (varRatio)")
        self._last_quart_lbl = self._add_row(layout, "  direction quarters")
        self._last_arc_lbl   = self._add_row(layout, "  window arc")

        layout.addWidget(self._separator())

        # Detection summary
        det_row = QtWidgets.QHBoxLayout()
        det_lbl = QtWidgets.QLabel("Circle detections:")
        det_lbl.setStyleSheet(f"color: {FG}; font: 13px 'Helvetica';")
        self._det_count_lbl = QtWidgets.QLabel("0")
        self._det_count_lbl.setStyleSheet(
            f"color: {ACCENT}; font: bold 30px 'Helvetica';")
        det_row.addWidget(det_lbl)
        det_row.addSpacing(10)
        det_row.addWidget(self._det_count_lbl)
        det_row.addStretch(1)
        layout.addLayout(det_row)
        layout.addStretch(1)

        # Status bar at the bottom — same compact format as viewer.py so the
        # audience can read the link health without scanning the main body.
        self._status_lbl = QtWidgets.QLabel("connecting…")
        self._status_lbl.setStyleSheet(
            f"color: {FG}; font: 12px 'Menlo'; padding: 4px;")
        self.statusBar().addWidget(self._status_lbl)
        self.statusBar().setStyleSheet(f"background-color: #F2F2F2;")

    def _add_row(self, parent_layout, label: str) -> QtWidgets.QLabel:
        row = QtWidgets.QHBoxLayout()
        name = QtWidgets.QLabel(label)
        name.setStyleSheet(f"color: {FG}; font: 13px 'Helvetica';")
        name.setFixedWidth(280)
        value = QtWidgets.QLabel("--")
        value.setStyleSheet(f"color: {FG}; font: 14px 'Menlo';")
        row.addWidget(name)
        row.addWidget(value, 1)
        parent_layout.addLayout(row)
        return value

    def _separator(self) -> QtWidgets.QFrame:
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        line.setStyleSheet("color: #DDD;")
        return line

    # ---------------------------------------------------------------- DATA
    def _start_worker(self, port: str, baud: int) -> None:
        self._thread = QtCore.QThread()
        self._worker = _SerialWorker(port, baud)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.sample_received.connect(self._on_sample)
        self._thread.start()

    @QtCore.pyqtSlot(object, dict)
    def _on_sample(self, sample, stats: dict) -> None:
        # --- Device-boot detection (seq reset) --------------------------
        # First frame ever, or seq dropped -> the device just booted.
        if self._last_seq is None or sample.seq < self._last_seq:
            self._start_settle()
            self._win.clear()          # stale pre-boot window is meaningless
            self._cum_arc = 0.0
            self._last_accel_angle = None
        self._last_seq = sample.seq

        q0, q1, q2, q3 = sample.quat
        self._quat_lbl.setText(f"{q0:+.3f}  {q1:+.3f}  {q2:+.3f}  {q3:+.3f}")
        self._yaw_lbl.setText(f"{sample.yaw_rate_dps:+8.2f} dps")
        self._yawc_lbl.setText(f"{sample.yaw_cum:+8.1f} deg")
        if sample.fusion_healthy:
            self._health_lbl.setText("OK")
            self._health_lbl.setStyleSheet(f"color: {GREEN}; font: 14px 'Menlo';")
        else:
            self._health_lbl.setText("saturated")
            self._health_lbl.setStyleSheet(f"color: {ALARM_FG}; font: 14px 'Menlo';")

        ax, ay, az = sample.f_acc
        self._acc_lbl.setText(f"{ax:+6.3f}  {ay:+6.3f}  {az:+6.3f}")
        gx, gy, gz = sample.f_gyr
        self._gyr_lbl.setText(f"{gx:+7.2f}  {gy:+7.2f}  {gz:+7.2f}")

        self._status_lbl.setText(
            f"frames={stats['frames_ok']}   "
            f"CRC errs={stats['crc_errors']}   "
            f"lost={stats['lost_packets']}   "
            f"sync skips={stats['sync_skips']}")

        # --- Live detection-gate computation ----------------------------
        self._win.append((ax, ay, sample.yaw_rate_dps))
        self._update_gates()

        # Rising-edge detection counter + popup + frozen snapshot
        if sample.detected and not self._prev_det_flag:
            self._detection_count += 1
            self._det_count_lbl.setText(str(self._detection_count))
            self._snapshot_detection(sample)
            self._show_popup()
        self._prev_det_flag = sample.detected

    # ----------------------------------------------------------- SETTLE
    def _start_settle(self):
        """(Re)arm the settle countdown -- called on device boot."""
        self._settle_remaining = SETTLE_SECONDS
        self._settle_lbl.setVisible(True)
        # Reset to the orange "keep still" style (may have been green before).
        self._settle_lbl.setStyleSheet(
            f"color: {ALARM_FG}; background-color: {ALARM_BG}; "
            f"font: bold 15px 'Helvetica'; padding: 6px; "
            f"border: 1px solid {ALARM_FG};")
        if not self._settle_timer.isActive():
            self._settle_timer.start(100)   # 10 Hz countdown
        self._tick_settle()                 # paint immediately

    def _tick_settle(self):
        """Countdown banner: keep the device still while Mahony converges."""
        if self._settle_remaining > 0.0:
            self._settle_lbl.setVisible(True)
            self._settle_lbl.setText(
                f"⚠  DEVICE BOOTED — KEEP STILL while fusion settles  "
                f"({self._settle_remaining:.1f} s)")
            self._settle_remaining -= 0.1
        else:
            self._settle_lbl.setText("✓  Ready — you may start drawing circles")
            self._settle_lbl.setStyleSheet(
                f"color: {GREEN}; background-color: #EAF7EE; "
                f"font: bold 15px 'Helvetica'; padding: 6px; "
                f"border: 1px solid {GREEN};")
            self._settle_timer.stop()
            # Hide the "ready" note after 3 s.
            QtCore.QTimer.singleShot(3000,
                                     lambda: self._settle_lbl.setVisible(False))

    # ------------------------------------------------------------ GATES
    def _set_gate(self, lbl, text, ok):
        """Render a gate row: green if passing, orange (alarm color) if not."""
        mark = "✓" if ok else "✗"
        color = GREEN if ok else ALARM_FG
        lbl.setText(f"{text}   {mark}")
        lbl.setStyleSheet(f"color: {color}; font: 14px 'Menlo';")

    def _update_gates(self):
        g = compute_gates(list(self._win))
        if g is None:
            return  # window not full yet

        self._set_gate(self._g_yaw,
            f"{g['mean_yaw']:6.1f} dps   (> {YAW_RATE_MIN:.0f})",
            g["mean_yaw_ok"])
        self._set_gate(self._g_varx,
            f"{g['var_x']:.4f} g²   (> {VAR_G2_MIN:.3f})",
            g["var_x_ok"])
        self._set_gate(self._g_vary,
            f"{g['var_y']:.4f} g²   (> {VAR_G2_MIN:.3f})",
            g["var_y_ok"])
        self._set_gate(self._g_ratio,
            f"{g['var_ratio']:5.2f}   (≤ {VAR_RATIO_MAX:.1f})",
            g["shape_ok"])
        self._set_gate(self._g_cov,
            f"{g['cov2']:.5f} < {g['cov_lim']:.5f}",
            g["not_straight"])
        self._set_gate(self._g_quart,
            f"{g['matching']}/4 same dir   (≥ {QUARTERS_REQUIRED})",
            g["dir_ok"])
        self._set_gate(self._g_phase,
            f"arc={g['arc']:+5.0f}° (≥{ARC_MIN_DEG:.0f})  zc={g['zcx']},{g['zcy']}",
            g["phase_ok"])

        # PC-side cumulative-arc mirror: integrate the rotation of the
        # earth-frame linear-accel (centripetal) vector -- matches firmware.
        ax, ay, _ = self._win[-1]
        self._acc_sx = 0.85 * self._acc_sx + 0.15 * ax   # EMA de-jitter
        self._acc_sy = 0.85 * self._acc_sy + 0.15 * ay
        a_mag = math.hypot(self._acc_sx, self._acc_sy)
        enough_motion = ((g["var_x"] + g["var_y"]) > (2.0 * VAR_G2_MIN)
                         and g["var_x"] < VAR_G2_MAX and g["var_y"] < VAR_G2_MAX)
        wrist_active  = g["mean_abs_yaw"] > YAW_ABS_MEAN_MIN
        if enough_motion and wrist_active and a_mag > 0.05:
            angle = math.atan2(self._acc_sy, self._acc_sx)
            if self._last_accel_angle is not None:
                d = angle - self._last_accel_angle
                if d > math.pi:    d -= 2 * math.pi
                elif d < -math.pi: d += 2 * math.pi
                self._cum_arc += math.degrees(d)
            self._last_accel_angle = angle
        else:
            self._last_accel_angle = None         # angle unreliable, skip
            self._cum_arc *= 0.97                  # bleed off when still
            if abs(self._cum_arc) < 5.0:
                self._cum_arc = 0.0
        # Alarm on accumulated angle alone (no candidate gate, matches fw).
        if abs(self._cum_arc) >= CUM_ARC_THRESH:
            self._cum_arc_at_fire = self._cum_arc   # capture before reset
            self._cum_arc = 0.0
            self._last_accel_angle = None
        self._set_gate(self._g_cum,
            f"{abs(self._cum_arc):5.0f}°   (→ {CUM_ARC_THRESH:.0f})",
            abs(self._cum_arc) >= CUM_ARC_THRESH)

        self._set_gate(self._g_cand,
            "ALL GATES PASS" if g["candidate"] else "waiting",
            g["candidate"])

    # ----------------------------------------------------------- SNAPSHOT
    def _snapshot_detection(self, sample) -> None:
        """Freeze ALL detection conditions at the moment of detection."""
        g = compute_gates(list(self._win))

        self._last_seq_lbl.setText(
            f"#{self._detection_count}   (sample seq={sample.seq})")
        # cumArc that crossed the threshold (PC-side mirror of firmware).
        self._last_cum_lbl.setText(
            f"{abs(self._cum_arc_at_fire):5.0f}°   (threshold {CUM_ARC_THRESH:.0f})")

        if g is not None:
            self._last_yawr_lbl.setText(
                f"{g['mean_yaw']:6.1f} dps   (> {YAW_RATE_MIN:.0f})")
            self._last_varx_lbl.setText(
                f"{g['var_x']:.4f} g²   (> {VAR_G2_MIN:.3f})")
            self._last_vary_lbl.setText(
                f"{g['var_y']:.4f} g²   (> {VAR_G2_MIN:.3f})")
            self._last_ratio_lbl.setText(
                f"{g['var_ratio']:5.2f}   (≤ {VAR_RATIO_MAX:.1f})")
            self._last_quart_lbl.setText(
                f"{g['matching']}/4 same dir   (≥ {QUARTERS_REQUIRED})")
            self._last_arc_lbl.setText(
                f"{g['arc']:+5.0f}°   (≥ {ARC_MIN_DEG:.0f})")

        for lbl in (self._last_seq_lbl, self._last_cum_lbl, self._last_yawr_lbl,
                    self._last_varx_lbl, self._last_vary_lbl,
                    self._last_ratio_lbl, self._last_quart_lbl,
                    self._last_arc_lbl):
            lbl.setStyleSheet(f"color: {ALARM_FG}; font: 14px 'Menlo';")

    # --------------------------------------------------------------- POPUP
    def _show_popup(self) -> None:
        if self._popup is not None:
            self._popup.close()
        popup = QtWidgets.QLabel(self)
        popup.setText(f"CIRCLE DETECTED\n#{self._detection_count}")
        popup.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        popup.setStyleSheet(
            f"background-color: {ALARM_BG}; "
            f"color: {ALARM_FG}; "
            f"border: 2px solid {ALARM_FG}; "
            f"font: bold 20px 'Helvetica'; "
            f"padding: 12px;")
        popup.setWindowFlags(
            QtCore.Qt.WindowType.FramelessWindowHint |
            QtCore.Qt.WindowType.Tool)
        popup.resize(360, 100)
        # Center over the main window.
        geo = self.geometry()
        popup.move(geo.x() + (geo.width()  - 360) // 2,
                   geo.y() + (geo.height() - 100) // 2)
        popup.show()
        self._popup = popup
        QtCore.QTimer.singleShot(self.POPUP_LIFETIME_MS, popup.close)

    # ----------------------------------------------------------- SHUTDOWN
    def closeEvent(self, event) -> None:
        self._worker.stop()
        self._thread.quit()
        self._thread.wait(500)
        super().closeEvent(event)


def main() -> int:
    ap = argparse.ArgumentParser(description="BMI160 simple dashboard (Qt)")
    ap.add_argument("--port", required=True,
                    help="Serial port, e.g. /dev/tty.usbmodem1422 or COM5")
    ap.add_argument("--baud", type=int, default=115200,
                    help="Baud rate (default: 115200)")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    win = Dashboard(args.port, args.baud)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
