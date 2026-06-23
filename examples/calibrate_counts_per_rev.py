#!/usr/bin/env python3
"""Measure counts_per_rev — counts per revolution of the MOTOR shaft, per motor.

Publishing joint_states in rad needs the counts-per-revolution of each motor.
It varies per motor (hall >= 3 x pole count; encoder = 4 x PPR) and depends on how
the controller counts, so measure it instead of trusting the datasheet. Put the
measured value into the ROS node `counts_per_rev` parameter (or your own conversion).

Measure at the MOTOR shaft. The controller counts position (and reports speed) at
the motor, and the rad/s conversion uses the motor rate, so a value taken at a
geared output shaft would make position and velocity disagree by the gear ratio.
Turn the motor shaft (not a geared output) and handle the gearbox in the layer
above (e.g. diff_drive_controller wheel_radius).

Two methods (you need an independent reference for the number of turns):
  manual (default): torque_off, then turn the MOTOR shaft **exactly N revolutions**
                    by hand -> delta_count / N. Safest and most accurate (no
                    commanded motion).
  driven          : spin slowly and press Enter at start / after N revolutions
                    while watching a mark on the MOTOR shaft -> delta_count / N.
                    (use when you cannot turn the motor shaft by hand)

Safety: 'driven' turns the motor. Keep it unloaded, slow, e-stop ready. In
'manual' the shaft also spins freely.

Usage:
    python3 examples/calibrate_counts_per_rev.py --port /dev/ttyUSB0 --revs 10
    python3 examples/calibrate_counts_per_rev.py --port /dev/ttyUSB0 --type dual
    python3 examples/calibrate_counts_per_rev.py --port /dev/ttyUSB0 --method driven --rpm 30
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow running straight from the repo without installing mdrobot:
# add src/mdrobot to the import path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot"))

from mdrobot import DualMotorDriver, SingleMotorDriver  # noqa: E402


def read_positions(driver, dual: bool) -> list[int]:
    if dual:
        return list(driver.get_positions())
    return [driver.get_position()]


def main() -> int:
    ap = argparse.ArgumentParser(description="counts_per_rev calibration")
    ap.add_argument("--port", required=True)
    ap.add_argument("--type", choices=["single", "dual"], default="single")
    ap.add_argument("--method", choices=["manual", "driven"], default="manual")
    ap.add_argument("--revs", type=float, default=10.0, help="number of revolutions to turn (default 10)")
    ap.add_argument("--rpm", type=int, default=30, help="low speed for driven mode")
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--id", type=int, default=1)
    args = ap.parse_args()

    dual = args.type == "dual"
    cls = DualMotorDriver if dual else SingleMotorDriver
    with cls.open(args.port, args.baud, slave_id=args.id) as driver:
        print(f"version={driver.get_version()}  voltage={driver.get_voltage()}V  type={args.type}")
        driver.reset_position()
        time.sleep(0.2)
        base = read_positions(driver, dual)
        print(f"baseline position = {base}")

        try:
            if args.method == "manual":
                driver.torque_off_both() if dual else driver.torque_off()
                print(f"\n[manual] Shaft is free. Turn the MOTOR shaft **exactly {args.revs} revolutions**.")
                input("  Press Enter when done...")
            else:  # driven
                driver.enable()
                print(f"\n[driven] Spinning at {args.rpm} rpm. Count MOTOR-shaft revolutions by a mark. (motor turns)")
                driver.set_velocities(args.rpm, args.rpm) if dual else driver.set_velocity(args.rpm)
                input(f"  Press Enter the moment it has turned exactly {args.revs} revolutions...")
                driver.stop()
                time.sleep(0.8)

            end = read_positions(driver, dual)
            print(f"final position = {end}")

            print("\n=== result ===")
            cprs = []
            for i, (b, e) in enumerate(zip(base, end)):
                cpr = abs(e - b) / args.revs if args.revs > 0 else float("nan")
                cprs.append(cpr)
                print(f"  channel {i + 1}: delta_count={e - b:+d} / {args.revs} rev -> counts_per_rev ~ {cpr:.2f}")
            arr = "[" + ", ".join(f"{v:.1f}" for v in cprs) + "]"
            print(f"\nROS parameter:  -p counts_per_rev:={arr}")
            print("  (sign ignored, magnitude only; a negative delta just means the hand/command direction was reversed.)")
        finally:
            if dual:
                driver.stop(); driver.torque_off_both()
            else:
                driver.stop(); driver.torque_off()
            driver.disable()
            print("\n[stop] stop + torque_off + disable")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
