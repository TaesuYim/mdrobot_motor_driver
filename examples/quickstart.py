#!/usr/bin/env python3
"""mdrobot 최소 사용 예제.

기본은 **읽기 전용**(모터 무회전): version/voltage/status/monitor를 출력한다.
`--drive`를 주면 저속으로 잠깐 구동한다(안전: 비상정지 준비, 저속, 짧게).

사용:
    # 읽기 전용
    python3 examples/quickstart.py --port /dev/ttyUSB1 --type single
    python3 examples/quickstart.py --port /dev/ttyUSB0 --type dual
    # 저속 구동 (모터가 회전한다!)
    python3 examples/quickstart.py --port /dev/ttyUSB1 --type single --drive --rpm 40
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# mdrobot가 설치되지 않은 경우(저장소에서 직접 실행) repo 루트를 import path에 추가.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mdrobot import DualMotorDriver, SingleMotorDriver  # noqa: E402


def show(driver, dual: bool) -> None:
    print(f"version : {driver.get_version()}")
    print(f"voltage : {driver.get_voltage()} V")
    print(f"status  : {driver.get_status().active or '없음'}")
    mon = driver.read_monitor()
    if dual:
        print(f"monitor : M1 spd={mon.motor1.speed_rpm} pos={mon.motor1.position} | "
              f"M2 spd={mon.motor2.speed_rpm} pos={mon.motor2.position}")
    else:
        print(f"monitor : speed={mon.speed_rpm} current={mon.current_a}A pos={mon.position}")


def drive(driver, dual: bool, rpm: int, hold: float) -> None:
    print(f"\n[구동] enable() 후 {rpm}rpm, {hold}s … (회전 주의)")
    driver.enable()  # UI_COM=1 + START_STOP arm
    try:
        if dual:
            driver.set_velocities(rpm, rpm)
        else:
            driver.set_velocity(rpm)
        # PNT50은 명령 후 회전까지 ~1초 지연이 있으니 충분히 기다린다.
        for _ in range(int(hold / 0.3)):
            time.sleep(0.3)
            show(driver, dual)
    finally:
        if dual:
            driver.stop(); driver.torque_off_both()
        else:
            driver.stop(); driver.torque_off()
        driver.disable()
        print("[정지] stop + torque_off + disable")


def main() -> int:
    ap = argparse.ArgumentParser(description="mdrobot 최소 예제")
    ap.add_argument("--port", required=True)
    ap.add_argument("--type", choices=["single", "dual"], required=True)
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--drive", action="store_true", help="저속 구동(모터 회전)")
    ap.add_argument("--rpm", type=int, default=40)
    ap.add_argument("--hold", type=float, default=1.5)
    args = ap.parse_args()

    dual = args.type == "dual"
    cls = DualMotorDriver if dual else SingleMotorDriver
    with cls.open(args.port, args.baud, slave_id=args.id) as driver:
        show(driver, dual)
        # MD400은 serial 구동에 USE_LIMIT_SW=0이 필요할 수 있다(매뉴얼 §6 참고).
        if args.drive and not dual:
            driver.client.write_register(17, 0)  # PID_USE_LIMIT_SW=0
        if args.drive:
            drive(driver, dual, args.rpm, args.hold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
