"""
Offline detection-algorithm validator.

Replays record.py CSVs through the detection algorithm in pure Python and
compares four development stages side-by-side on identical data (no reflash):

  1. INSTRUCTOR  - yaw-based, window-level decision (recommendation.md NORMAL)
  2. ACCEL-ARC   - centripetal accel-vector arc + wrist-activity gate
  3. +EMA        - stage 2 with EMA smoothing of the accel vector
  4. +VAR-MAX    - stage 3 with the XY-variance upper bound (shipped firmware)

Usage:
    python3 replay.py clean.csv yaw_shake.csv quarter.csv half.csv \
                      half_plus.csv mixed.csv
"""

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field

IMU_DT_SEC            = 0.01
DETECTION_WINDOW_SIZE = 64


@dataclass
class Config:
    name:          str
    mode:          str          # "yaw" or "accel"
    yaw_min:       float
    var_min:       float
    cov_k:         float
    var_ratio_max: float
    min_zc:        int
    quarters:      int
    arc_min:       float
    cum_thresh:    float = 0.0   # accel mode
    yaw_abs_min:   float = 50.0  # accel mode wrist-activity
    use_ema:       bool = False
    var_max:       float = 0.0   # 0 = no upper bound
    keep_window:   bool = False  # keep window on detect/unhealthy (recall fix)


# Stage 1: instructor NORMAL (yaw-based, no cumArc).
ST1 = Config("1. INSTRUCTOR (yaw window)", mode="yaw",
             yaw_min=100.0, var_min=0.015, cov_k=0.65, var_ratio_max=3.0,
             min_zc=1, quarters=3, arc_min=70.0)

# Stage 2: accel-vector arc + wrist-activity (no EMA, no var upper bound).
ST2 = Config("2. ACCEL-ARC (no EMA)", mode="accel",
             yaw_min=100.0, var_min=0.011, cov_k=0.65, var_ratio_max=3.0,
             min_zc=2, quarters=3, arc_min=130.0,
             cum_thresh=360.0, yaw_abs_min=50.0, use_ema=False)

# Stage 3: + EMA smoothing.
ST3 = Config("3. + EMA", mode="accel",
             yaw_min=100.0, var_min=0.011, cov_k=0.65, var_ratio_max=3.0,
             min_zc=2, quarters=3, arc_min=130.0,
             cum_thresh=360.0, yaw_abs_min=50.0, use_ema=True)

# Stage 4: + variance upper bound (shipped firmware).
ST4 = Config("4. + VAR-MAX (shipped)", mode="accel",
             yaw_min=100.0, var_min=0.011, cov_k=0.65, var_ratio_max=3.0,
             min_zc=2, quarters=3, arc_min=130.0,
             cum_thresh=360.0, yaw_abs_min=50.0, use_ema=True, var_max=0.40)

# Stage 5: keep the window after a detection / fusion-unhealthy event instead
# of clearing it; removes the 0.64 s blind spot that dropped back-to-back
# circles (found via offline replay, masked in live one-at-a-time testing).
ST5 = Config("5. + KEEP-WINDOW (recall)", mode="accel",
             yaw_min=100.0, var_min=0.011, cov_k=0.65, var_ratio_max=3.0,
             min_zc=2, quarters=3, arc_min=130.0,
             cum_thresh=360.0, yaw_abs_min=50.0, use_ema=True, var_max=0.40,
             keep_window=True)

CONFIGS = [ST1, ST2, ST3, ST4, ST5]

# Tolerance-profile comparison: the SAME final algorithm (accel-arc + EMA +
# var-max + keep-window) run under the three detection.h profiles. In the
# accel-arc design the profile mainly changes the variance floor (var_min),
# which gates arc accumulation via enoughMotion = (varX+varY) > 2*var_min.
PROF_STRICT = Config("STRICT", mode="accel",
                     yaw_min=200.0, var_min=0.020, cov_k=0.50, var_ratio_max=2.0,
                     min_zc=2, quarters=4, arc_min=100.0,
                     cum_thresh=360.0, yaw_abs_min=50.0, use_ema=True,
                     var_max=0.40, keep_window=True)
PROF_NORMAL = Config("NORMAL (shipped)", mode="accel",
                     yaw_min=150.0, var_min=0.011, cov_k=0.65, var_ratio_max=3.0,
                     min_zc=2, quarters=4, arc_min=130.0,
                     cum_thresh=360.0, yaw_abs_min=50.0, use_ema=True,
                     var_max=0.40, keep_window=True)
PROF_LOOSE = Config("LOOSE", mode="accel",
                    yaw_min=60.0, var_min=0.008, cov_k=0.85, var_ratio_max=6.0,
                    min_zc=0, quarters=0, arc_min=45.0,
                    cum_thresh=360.0, yaw_abs_min=50.0, use_ema=True,
                    var_max=0.40, keep_window=True)

PROFILES = [PROF_STRICT, PROF_NORMAL, PROF_LOOSE]


@dataclass
class Sample:
    seq: int
    f_acc_x: float
    f_acc_y: float
    yaw_rate_dps: float
    fusion_healthy: int
    flags: int


def load_csv(path):
    out = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            out.append(Sample(
                seq=int(row["seq"]),
                f_acc_x=float(row["fAccX"]),
                f_acc_y=float(row["fAccY"]),
                yaw_rate_dps=float(row["yawRateDps"]),
                fusion_healthy=int(row["fusionHealthy"]),
                flags=int(row["flags"]),
            ))
    return out


def window_metrics(win):
    """Compute the per-window features shared by all stages."""
    n = len(win)
    yaw_sum = sum(w[2] for w in win)
    abs_yaw = sum(abs(w[2]) for w in win)
    quarter = [0.0, 0.0, 0.0, 0.0]
    for i, w in enumerate(win):
        quarter[i // (n // 4)] += w[2]
    expected_sign = 1 if yaw_sum >= 0 else -1
    matching = sum(1 for q in quarter
                   if abs(q) > 1e-3 and (1 if q >= 0 else -1) == expected_sign)

    mx = sum(w[0] for w in win) / n
    my = sum(w[1] for w in win) / n
    vx = vy = cov = 0.0
    zcx = zcy = 0
    for i, w in enumerate(win):
        dx, dy = w[0] - mx, w[1] - my
        vx += dx * dx
        vy += dy * dy
        cov += dx * dy
        if i > 0:
            pdx, pdy = win[i-1][0] - mx, win[i-1][1] - my
            if (pdx > 0 and dx <= 0) or (pdx < 0 and dx >= 0): zcx += 1
            if (pdy > 0 and dy <= 0) or (pdy < 0 and dy >= 0): zcy += 1
    vx /= n; vy /= n; cov /= n
    arc = sum(w[2] * IMU_DT_SEC for w in win)
    ratio = max(vx, vy) / (min(vx, vy) + 1e-6)
    return dict(yaw_sum=yaw_sum, mean_yaw=abs(yaw_sum) / n,
                mean_abs_yaw=abs_yaw / n, matching=matching,
                expected_sign=expected_sign, var_x=vx, var_y=vy, cov=cov,
                zcx=zcx, zcy=zcy, arc=arc, ratio=ratio)


@dataclass
class Detector:
    cfg: Config
    win: list = field(default_factory=list)
    cum_arc: float = 0.0
    last_angle: float = None
    acc_sx: float = 0.0
    acc_sy: float = 0.0
    detections: list = field(default_factory=list)

    def feed(self, s):
        if s.fusion_healthy == 0:
            if self.cfg.keep_window:
                self.last_angle = None    # pause arc, keep window
            else:
                self.win.clear()
            return
        self.win.append((s.f_acc_x, s.f_acc_y, s.yaw_rate_dps))
        if len(self.win) > DETECTION_WINDOW_SIZE:
            self.win.pop(0)
        if len(self.win) < DETECTION_WINDOW_SIZE:
            return
        m = window_metrics(self.win)
        c = self.cfg

        if c.mode == "yaw":
            self._feed_yaw(s, m)
        else:
            self._feed_accel(s, m)

    def _feed_yaw(self, s, m):
        c = self.cfg
        dir_ok = m["matching"] >= c.quarters
        not_straight = m["cov"] ** 2 < m["var_x"] * m["var_y"] * c.cov_k
        shape_ok = m["ratio"] <= c.var_ratio_max
        zc_ok = m["zcx"] >= c.min_zc and m["zcy"] >= c.min_zc
        arc_ok = abs(m["arc"]) >= c.arc_min
        phase_ok = arc_ok if c.min_zc == 0 else (zc_ok or arc_ok)
        cand = (dir_ok and not_straight and shape_ok and phase_ok
                and m["mean_yaw"] > c.yaw_min
                and m["var_x"] > c.var_min and m["var_y"] > c.var_min)
        if cand:                      # window-level decision = alarm
            self.detections.append((s.seq, m["expected_sign"]))
            self.win.clear()

    def _feed_accel(self, s, m):
        c = self.cfg
        # Arc accumulation gates: bounded energy + wrist actually rotating.
        enough = (m["var_x"] + m["var_y"]) > 2.0 * c.var_min
        if c.var_max > 0.0:
            enough = enough and m["var_x"] < c.var_max and m["var_y"] < c.var_max
        wrist = m["mean_abs_yaw"] > c.yaw_abs_min

        ax, ay = self.win[-1][0], self.win[-1][1]
        if c.use_ema:
            self.acc_sx = 0.85 * self.acc_sx + 0.15 * ax
            self.acc_sy = 0.85 * self.acc_sy + 0.15 * ay
            ax, ay = self.acc_sx, self.acc_sy
        a_mag = math.hypot(ax, ay)

        if enough and wrist and a_mag > 0.05:
            angle = math.atan2(ay, ax)
            if self.last_angle is not None:
                d = angle - self.last_angle
                if d > math.pi: d -= 2 * math.pi
                elif d < -math.pi: d += 2 * math.pi
                self.cum_arc += math.degrees(d)
            self.last_angle = angle
        else:
            self.last_angle = None
            self.cum_arc *= 0.97
            if abs(self.cum_arc) < 5.0:
                self.cum_arc = 0.0

        if abs(self.cum_arc) >= c.cum_thresh:
            self.detections.append((s.seq, 1 if self.cum_arc >= 0 else -1))
            self.cum_arc = 0.0
            self.last_angle = None
            if not c.keep_window:
                self.win.clear()


GROUND_TRUTH = {
    "clean_paused": (10, "10 clean circles, ~1.5 s pause each (intended use)"),
    "clean":      (10, "10 clean circles, 22.5 cm plate (continuous)"),
    "yaw_shake":  (0,  "yaw shake only"),
    "quarter":    (0,  "5x fast quarter circles"),
    "half_plus":  (0,  "5x fast half-plus arcs"),   # check before "half"
    "half":       (0,  "5x fast half circles"),
    "mixed_paused": (6, "3 CW + shake + 3 CCW + half, ~1.5 s pauses (intended use)"),
    "mixed":      (6,  "3 CW + shake + 3 CCW + half (continuous)"),
}


def expected_for(path):
    name = path.lower()
    for key, (exp, note) in GROUND_TRUTH.items():
        if key in name:
            return exp, note
    return None, ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_files", nargs="+")
    ap.add_argument("--profiles", action="store_true",
                    help="Compare STRICT/NORMAL/LOOSE tolerance profiles "
                         "(final algorithm) instead of the 5 development stages")
    args = ap.parse_args()

    configs = PROFILES if args.profiles else CONFIGS

    for path in args.csv_files:
        try:
            samples = load_csv(path)
        except FileNotFoundError:
            print(f"missing: {path}", file=sys.stderr)
            continue
        exp, note = expected_for(path)
        dur = len(samples) * IMU_DT_SEC
        print("=" * 64)
        print(f"{path}   ({len(samples)} samples, ~{dur:.1f}s)")
        if exp is not None:
            print(f"  expected = {exp}   [{note}]")
        print("=" * 64)
        for cfg in configs:
            det = Detector(cfg=cfg)
            for s in samples:
                det.feed(s)
            got = len(det.detections)
            if exp is None:
                acc = ""
            elif exp == 0:
                acc = "  PERFECT" if got == 0 else f"  {got} FALSE POS"
            else:
                tp = min(got, exp)
                extra = max(0, got - exp)
                acc = f"  TP={tp}/{exp}" + (f"  +{extra} extra" if extra else "")
            print(f"  {cfg.name:32s} -> {got:3d} detections{acc}")
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
