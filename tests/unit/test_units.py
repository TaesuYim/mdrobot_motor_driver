"""물리 단위 변환 helper 단위 테스트 (units.py, Phase 11)."""

import math

import pytest

from mdrobot.units import counts_to_rad, rad_s_to_rpm, rad_to_counts, rpm_to_rad_s


def test_counts_to_rad_full_rev():
    # 1회전 = 2π rad. 홀 24 counts/rev 가정.
    assert counts_to_rad(24, 24) == pytest.approx(2 * math.pi)
    assert counts_to_rad(12, 24) == pytest.approx(math.pi)
    assert counts_to_rad(0, 24) == 0.0


def test_counts_to_rad_sign_preserved():
    assert counts_to_rad(-6, 24) == pytest.approx(-math.pi / 2)


def test_counts_to_rad_encoder_resolution():
    # 엔코더 4×PPR = 65536 counts/rev.
    cpr = 4 * 16384
    assert counts_to_rad(cpr, cpr) == pytest.approx(2 * math.pi)
    assert counts_to_rad(1, cpr) == pytest.approx(2 * math.pi / cpr)


def test_rad_to_counts_inverse():
    for cpr in (24, 65536):
        for count in (0, 1, -1, 7, -13, 1000):
            assert rad_to_counts(counts_to_rad(count, cpr), cpr) == count


def test_rad_to_counts_rounds():
    # 24 counts/rev에서 1 count = 2π/24 rad. 그 절반 rad는 0 또는 1로 반올림.
    assert rad_to_counts(counts_to_rad(0.4, 24), 24) == 0
    assert rad_to_counts(counts_to_rad(0.6, 24), 24) == 1


def test_counts_per_rev_must_be_positive():
    with pytest.raises(ValueError):
        counts_to_rad(10, 0)
    with pytest.raises(ValueError):
        counts_to_rad(10, -24)
    with pytest.raises(ValueError):
        rad_to_counts(1.0, 0)


def test_rpm_rad_s_roundtrip():
    # 속도 변환은 counts_per_rev 무관.
    assert rpm_to_rad_s(60) == pytest.approx(2 * math.pi)      # 60rpm = 1rev/s
    assert rpm_to_rad_s(0) == 0.0
    assert rpm_to_rad_s(-30) == pytest.approx(-math.pi)
    for rpm in (-100, 0, 1, 3000):
        assert rad_s_to_rpm(rpm_to_rad_s(rpm)) == pytest.approx(rpm)
