"""Physical unit conversion helpers — count/rpm <-> rad / (rad/s).

Design (this package is a generic motor driver):
- The conversion is motor-independent; the only per-motor value is
  `counts_per_rev`. Pole count / encoder PPR are not hard-coded — the caller
  supplies `counts_per_rev` at runtime.
- `counts_per_rev` depends on the feedback source:
    * hall-based: a controller-specific value (typically 3 x pole count;
      e.g. ~24 measured on an 8-pole motor).
    * encoder-based: 4 x PPR (quadrature). The controller may not quadruple it,
      so measuring is recommended.
  Prefer measuring with `examples/calibrate_counts_per_rev.py` over the datasheet.
- Speed does NOT need `counts_per_rev`: the controller already reports speed in
  mechanical rpm, so `rpm * 2*pi/60` works regardless of pole count / PPR.
- Gear ratio is not handled here — joints are motor-shaft based; motor-shaft to
  wheel conversion belongs to the higher-level robot/odometry layer.
"""

from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


def counts_to_rad(count: float, counts_per_rev: float) -> float:
    """Convert a position count to motor-shaft angle (rad).

    counts_per_rev is counts per revolution (> 0). The sign is preserved (+ = CCW).
    """
    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be > 0, got {counts_per_rev}")
    return count * TWO_PI / counts_per_rev


def rad_to_counts(rad: float, counts_per_rev: float) -> int:
    """Convert motor-shaft angle (rad) to the nearest position count (inverse of counts_to_rad)."""
    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be > 0, got {counts_per_rev}")
    return int(round(rad * counts_per_rev / TWO_PI))


def rpm_to_rad_s(rpm: float) -> float:
    """Convert mechanical rpm to rad/s. No counts_per_rev needed."""
    return rpm * TWO_PI / 60.0


def rad_s_to_rpm(rad_s: float) -> float:
    """Convert rad/s to mechanical rpm."""
    return rad_s * 60.0 / TWO_PI


# --- slow-start / slow-down conversion (NOT yet hardware-verified) -------------------
SLOW_RAW_MAX = 1023
# Default full-scale time. The controller can raise it via PID_MAX_SS_TIME (15-60 s).
SLOW_DEFAULT_FULL_SCALE_S = 15.0


def slow_seconds_to_raw(seconds: float, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> int:
    """Convert a slow-start/down time (seconds) to the raw 0-1023 register value.

    Linear map 0..full_scale_s -> 0..1023, clamped to that range. The real full scale
    is set by PID_MAX_SS_TIME on the controller (default 15 s). NOT hardware-verified.
    """
    if seconds < 0:
        raise ValueError(f"seconds must be >= 0, got {seconds}")
    if full_scale_s <= 0:
        raise ValueError(f"full_scale_s must be > 0, got {full_scale_s}")
    raw = round(seconds / full_scale_s * SLOW_RAW_MAX)
    return max(0, min(SLOW_RAW_MAX, raw))


def slow_raw_to_seconds(raw: int, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
    """Convert a raw 0-1023 slow register value back to seconds (inverse of slow_seconds_to_raw)."""
    if full_scale_s <= 0:
        raise ValueError(f"full_scale_s must be > 0, got {full_scale_s}")
    return raw / SLOW_RAW_MAX * full_scale_s
