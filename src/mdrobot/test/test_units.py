"""Physical unit conversion helper unit tests (units.py)."""

import math

import pytest

from mdrobot.units import (
    SLOW_RAW_MAX,
    counts_to_rad,
    rad_s_to_rpm,
    rad_to_counts,
    rpm_to_rad_s,
    slow_raw_to_seconds,
    slow_seconds_to_raw,
)


def test_counts_to_rad_full_rev():
    # one revolution = 2*pi rad; assume hall 24 counts/rev.
    assert counts_to_rad(24, 24) == pytest.approx(2 * math.pi)
    assert counts_to_rad(12, 24) == pytest.approx(math.pi)
    assert counts_to_rad(0, 24) == 0.0


def test_counts_to_rad_sign_preserved():
    assert counts_to_rad(-6, 24) == pytest.approx(-math.pi / 2)


def test_counts_to_rad_encoder_resolution():
    # encoder 4 x PPR = 65536 counts/rev.
    cpr = 4 * 16384
    assert counts_to_rad(cpr, cpr) == pytest.approx(2 * math.pi)
    assert counts_to_rad(1, cpr) == pytest.approx(2 * math.pi / cpr)


def test_rad_to_counts_inverse():
    for cpr in (24, 65536):
        for count in (0, 1, -1, 7, -13, 1000):
            assert rad_to_counts(counts_to_rad(count, cpr), cpr) == count


def test_rad_to_counts_rounds():
    # At 24 counts/rev, 1 count = 2*pi/24 rad. Half of that rounds to 0 or 1.
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
    # speed conversion is independent of counts_per_rev.
    assert rpm_to_rad_s(60) == pytest.approx(2 * math.pi)      # 60 rpm = 1 rev/s
    assert rpm_to_rad_s(0) == 0.0
    assert rpm_to_rad_s(-30) == pytest.approx(-math.pi)
    for rpm in (-100, 0, 1, 3000):
        assert rad_s_to_rpm(rpm_to_rad_s(rpm)) == pytest.approx(rpm)


# --- slow-start / slow-down conversion ----------------------------------------------

def test_slow_seconds_to_raw_default_scale():
    assert slow_seconds_to_raw(0) == 0
    assert slow_seconds_to_raw(15) == SLOW_RAW_MAX
    assert slow_seconds_to_raw(7.5) == round(7.5 / 15 * SLOW_RAW_MAX)


def test_slow_seconds_to_raw_clamps_and_validates():
    assert slow_seconds_to_raw(30) == SLOW_RAW_MAX  # clamp above full scale
    assert slow_seconds_to_raw(30, full_scale_s=60) == round(30 / 60 * SLOW_RAW_MAX)
    with pytest.raises(ValueError):
        slow_seconds_to_raw(-1)
    with pytest.raises(ValueError):
        slow_seconds_to_raw(1, full_scale_s=0)


def test_slow_raw_to_seconds():
    assert slow_raw_to_seconds(0) == 0.0
    assert slow_raw_to_seconds(SLOW_RAW_MAX) == pytest.approx(15.0)
    assert slow_raw_to_seconds(512) == pytest.approx(512 / SLOW_RAW_MAX * 15.0)


def test_slow_roundtrip_within_quantization():
    step = 15.0 / SLOW_RAW_MAX
    for sec in (0.0, 1.0, 5.0, 15.0):
        assert slow_raw_to_seconds(slow_seconds_to_raw(sec)) == pytest.approx(sec, abs=step)
