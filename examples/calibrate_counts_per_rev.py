#!/usr/bin/env python3
"""counts_per_rev 실측 — joint_states를 rad로 발행하려면 모터별 1회전당 count가 필요하다.

값은 모터마다 다르고(홀 ≈ 3×극수, 엔코더 = 4×PPR) 컨트롤러 카운팅 방식에 좌우되므로
사양값을 믿기보다 실측한다. 측정한 값을 ROS 노드 `counts_per_rev` 파라미터(또는 직접 변환)에 넣는다.

독립적인 회전수 기준이 필요하므로 두 방법을 제공한다:
  manual (기본): torque_off로 출력축을 free로 만든 뒤 **정확히 N바퀴** 손으로 돌린다 → Δcount / N.
                 명령 모션이 없어 가장 안전하고 정확하다.
  driven       : 저속으로 구동하고, 축의 마크를 보며 시작/‘N바퀴 후’에 Enter를 친다 → Δcount / N.
                 (감속비가 커서 손으로 못 돌릴 때)

안전: driven은 모터가 회전한다. 무부하·저속·비상정지 준비. manual도 축이 자유롭게 돈다.

사용:
    python3 examples/calibrate_counts_per_rev.py --port /dev/ttyUSB0 --revs 10
    python3 examples/calibrate_counts_per_rev.py --port /dev/ttyUSB0 --type dual
    python3 examples/calibrate_counts_per_rev.py --port /dev/ttyUSB0 --method driven --rpm 30
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# mdrobot가 설치되지 않은 경우(저장소에서 직접 실행) repo 루트를 import path에 추가.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mdrobot import DualMotorDriver, SingleMotorDriver  # noqa: E402


def read_positions(driver, dual: bool) -> list[int]:
    if dual:
        return list(driver.get_positions())
    return [driver.get_position()]


def main() -> int:
    ap = argparse.ArgumentParser(description="counts_per_rev 실측 캘리브레이션")
    ap.add_argument("--port", required=True)
    ap.add_argument("--type", choices=["single", "dual"], default="single")
    ap.add_argument("--method", choices=["manual", "driven"], default="manual")
    ap.add_argument("--revs", type=float, default=10.0, help="돌릴 회전수(기본 10)")
    ap.add_argument("--rpm", type=int, default=30, help="driven 모드 저속 rpm")
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
        print(f"기준 position = {base}")

        try:
            if args.method == "manual":
                driver.torque_off_both() if dual else driver.torque_off()
                print(f"\n[manual] 출력축이 free 상태입니다. 출력축을 **정확히 {args.revs}바퀴** 돌리세요.")
                input("  다 돌렸으면 Enter…")
            else:  # driven
                driver.enable()
                print(f"\n[driven] {args.rpm}rpm로 구동합니다. 축의 마크를 보며 회전수를 세세요. (회전 주의)")
                driver.set_velocities(args.rpm, args.rpm) if dual else driver.set_velocity(args.rpm)
                input(f"  정확히 {args.revs}바퀴 돈 순간 Enter…")
                driver.stop()
                time.sleep(0.8)

            end = read_positions(driver, dual)
            print(f"최종 position = {end}")

            print("\n=== 결과 ===")
            cprs = []
            for i, (b, e) in enumerate(zip(base, end)):
                cpr = abs(e - b) / args.revs if args.revs > 0 else float("nan")
                cprs.append(cpr)
                print(f"  채널{i + 1}: Δcount={e - b:+d} / {args.revs}rev → counts_per_rev ≈ {cpr:.2f}")
            arr = "[" + ", ".join(f"{v:.1f}" for v in cprs) + "]"
            print(f"\nROS 파라미터:  -p counts_per_rev:={arr}")
            print("  (방향 부호 무시, 크기만 사용. 음수면 손/명령 방향이 반대였던 것뿐.)")
        finally:
            if dual:
                driver.stop(); driver.torque_off_both()
            else:
                driver.stop(); driver.torque_off()
            driver.disable()
            print("\n[정지] stop + torque_off + disable")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
