"""물리 단위 변환 helper — count/rpm ↔ rad/(rad·s) (Phase 11).

설계 원칙(이 패키지는 범용 모터 드라이버, CLAUDE.md §1):
- 변환식은 모터에 **무관한 일반식**이고, 모터마다 달라지는 값은 `counts_per_rev` **하나**다.
  따라서 극수/엔코더 PPR을 코드에 박지 않고 호출자가 `counts_per_rev`를 런타임에 준다.
- `counts_per_rev`는 피드백 출처에 따라:
    * 홀 기반: 컨트롤러 고유값(MD400 HALL_TYPE=1/8극에서 실측 ≈24, 관례적으로 3×극수).
    * 엔코더 기반: 4×PPR(4체배 quadrature). 컨트롤러가 4체배 안 할 수도 있어 **실측 권장**.
  사양값을 믿기보다 `examples/calibrate_counts_per_rev.py`로 측정해 확정한다.
- **속도는 `counts_per_rev`가 필요 없다.** 컨트롤러가 속도를 이미 기계 rpm으로 주므로
  극수/PPR과 무관하게 `rpm × 2π/60`이면 된다.
- 감속비(gear ratio)는 여기서 다루지 않는다 — joint은 **모터축** 기준이고, 모터축↔바퀴 변환은
  상위 로봇/오도메트리 계층(URDF·diff_drive)의 몫이다.
"""

from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


def counts_to_rad(count: float, counts_per_rev: float) -> float:
    """position count를 모터축 각도(rad)로 변환한다.

    counts_per_rev는 1회전당 count 수(>0). 부호는 그대로 유지된다(+=CCW).
    """
    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be > 0, got {counts_per_rev}")
    return count * TWO_PI / counts_per_rev


def rad_to_counts(rad: float, counts_per_rev: float) -> int:
    """모터축 각도(rad)를 가장 가까운 position count로 변환한다(counts_to_rad의 역)."""
    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be > 0, got {counts_per_rev}")
    return int(round(rad * counts_per_rev / TWO_PI))


def rpm_to_rad_s(rpm: float) -> float:
    """기계 회전속도 rpm을 rad/s로 변환한다. counts_per_rev 불필요."""
    return rpm * TWO_PI / 60.0


def rad_s_to_rpm(rad_s: float) -> float:
    """rad/s를 기계 회전속도 rpm으로 변환한다."""
    return rad_s * 60.0 / TWO_PI
