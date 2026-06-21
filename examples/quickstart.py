#!/usr/bin/env python3
"""Minimal mdrobot example.

Read-only by default (the motor does not move): prints version / voltage /
status / monitor. Pass --drive to spin the motor briefly at low speed
(SAFETY: keep an e-stop ready, low speed, short).

Usage:
    # read-only
    python3 examples/quickstart.py --port /dev/ttyUSB0 --type single
    python3 examples/quickstart.py --port /dev/ttyUSB0 --type dual
    # low-speed drive (the motor WILL turn!)
    python3 examples/quickstart.py --port /dev/ttyUSB0 --type single --drive --rpm 40
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


def show(driver, dual: bool) -> None:
    print(f"version : {driver.get_version()}")
    print(f"voltage : {driver.get_voltage()} V")
    print(f"status  : {driver.get_status().active or 'none'}")
    mon = driver.read_monitor()
    if dual:
        print(f"monitor : M1 spd={mon.motor1.speed_rpm} pos={mon.motor1.position} | "
              f"M2 spd={mon.motor2.speed_rpm} pos={mon.motor2.position}")
    else:
        print(f"monitor : speed={mon.speed_rpm} current={mon.current_a}A pos={mon.position}")


def drive(driver, dual: bool, rpm: int, hold: float) -> None:
    print(f"\n[drive] enable(), {rpm} rpm for {hold}s ... (motor will turn)")
    driver.enable()  # UI_COM=1 + START/STOP arm
    try:
        if dual:
            driver.set_velocities(rpm, rpm)
        else:
            driver.set_velocity(rpm)
        # Some dual-channel controllers start turning ~1 s after the command;
        # wait long enough before reading back.
        for _ in range(int(hold / 0.3)):
            time.sleep(0.3)
            show(driver, dual)
    finally:
        if dual:
            driver.stop(); driver.torque_off_both()
        else:
            driver.stop(); driver.torque_off()
        driver.disable()
        print("[stop] stop + torque_off + disable")


def main() -> int:
    ap = argparse.ArgumentParser(description="mdrobot minimal example")
    ap.add_argument("--port", required=True)
    ap.add_argument("--type", choices=["single", "dual"], required=True)
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--drive", action="store_true", help="low-speed drive (motor turns)")
    ap.add_argument("--rpm", type=int, default=40)
    ap.add_argument("--hold", type=float, default=1.5)
    args = ap.parse_args()

    dual = args.type == "dual"
    cls = DualMotorDriver if dual else SingleMotorDriver
    with cls.open(args.port, args.baud, slave_id=args.id) as driver:
        show(driver, dual)
        # Some single-channel controllers need USE_LIMIT_SW=0 for serial drive
        # (see the troubleshooting section in docs/manual/).
        if args.drive and not dual:
            driver.client.write_register(17, 0)  # USE_LIMIT_SW = 0
        if args.drive:
            drive(driver, dual, args.rpm, args.hold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
