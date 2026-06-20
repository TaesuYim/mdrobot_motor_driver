"""status.py 단위 테스트: status bit / monitor 디코딩 (doc 05)."""

import pytest

from mdrobot.status import (
    DI_BIT_NAMES,
    STATUS1_BIT_NAMES,
    DualMonitor,
    Monitor,
    StatusBits,
    active_bits,
    decode_monitor,
    decode_pnt_main_data,
    decode_pnt_monitor,
)


def test_status_bits_all_clear():
    bits = StatusBits.from_byte(0x00)
    assert bits.raw == 0
    assert not any(
        [bits.alarm, bits.ctrl_fail, bits.over_voltage, bits.over_temperature,
         bits.overload, bits.hall_or_encoder_fail, bits.inverse_velocity, bits.stall]
    )
    assert bits.active == []


def test_status_bits_all_set():
    bits = StatusBits.from_byte(0xFF)
    assert bits.raw == 0xFF
    assert all(
        [bits.alarm, bits.ctrl_fail, bits.over_voltage, bits.over_temperature,
         bits.overload, bits.hall_or_encoder_fail, bits.inverse_velocity, bits.stall]
    )
    # 8개 bit 이름이 bit 오름차순으로 모두 나온다.
    assert bits.active == [STATUS1_BIT_NAMES[i] for i in range(8)]


def test_status_bits_selected():
    # bit0 ALARM, bit2 OVER_VOLT.
    bits = StatusBits.from_byte(0x05)
    assert bits.alarm and bits.over_voltage
    assert not bits.ctrl_fail and not bits.stall
    assert bits.active == ["ALARM", "OVER_VOLT"]


def test_status_bits_masks_high_bits():
    # 0x100은 byte 범위를 넘으므로 마스킹되어 0이 된다.
    assert StatusBits.from_byte(0x100).raw == 0


def test_active_bits_di():
    # bit2 DIR, bit4 START_STOP.
    word = (1 << 2) | (1 << 4)
    assert active_bits(word, DI_BIT_NAMES) == ["DIR", "START_STOP"]


def test_decode_monitor():
    # speed=100, current=1.2A, output=-50, position=0x12345678,
    # status word: status2=0x02(ENC_FAIL), status1=0x05(ALARM|OVER_VOLT).
    words = [0x0064, 0x000C, 0xFFCE, 0x5678, 0x1234, 0x0205]
    mon = decode_monitor(words)
    assert isinstance(mon, Monitor)
    assert mon.speed_rpm == 100
    assert mon.current_a == pytest.approx(1.2)
    assert mon.output_raw == -50
    assert mon.position == 0x12345678
    assert mon.status.active == ["ALARM", "OVER_VOLT"]
    assert mon.status2_raw == 0x02


def test_decode_monitor_negative_speed_and_position():
    # speed=-100(0xFF9C), position=-1(0xFFFF,0xFFFF).
    words = [0xFF9C, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000]
    mon = decode_monitor(words)
    assert mon.speed_rpm == -100
    assert mon.position == -1


def test_decode_monitor_wrong_length():
    with pytest.raises(ValueError):
        decode_monitor([0, 0, 0])


def test_decode_pnt_monitor():
    # [speed1, pos1L, pos1H, speed2, pos2L, pos2H, status(H=st2,L=st1)]
    # M1 speed=100 pos=0x00000005 status1=0x01(ALARM)
    # M2 speed=-50(0xFFCE) pos=-1 status2(M2)=0x05(ALARM|OVER_VOLT)
    words = [0x0064, 0x0005, 0x0000, 0xFFCE, 0xFFFF, 0xFFFF, 0x0501]
    dm = decode_pnt_monitor(words)
    assert isinstance(dm, DualMonitor)
    assert dm.motor1.speed_rpm == 100
    assert dm.motor1.position == 5
    assert dm.motor1.current_a is None
    assert dm.motor1.output_raw is None
    assert dm.motor1.status.active == ["ALARM"]
    assert dm.motor2.speed_rpm == -50
    assert dm.motor2.position == -1
    assert dm.motor2.status.active == ["ALARM", "OVER_VOLT"]


def test_decode_pnt_monitor_wrong_length():
    with pytest.raises(ValueError):
        decode_pnt_monitor([0] * 6)


def test_decode_pnt_main_data():
    # [s1, cur1, pos1L, pos1H, s2, cur2, pos2L, pos2H, status]
    words = [0x0064, 0x000C, 0x5678, 0x1234, 0xFF9C, 0x0006, 0x0000, 0x0000, 0x0001]
    dm = decode_pnt_main_data(words)
    assert dm.motor1.speed_rpm == 100
    assert dm.motor1.current_a == pytest.approx(1.2)
    assert dm.motor1.position == 0x12345678
    assert dm.motor1.output_raw is None
    assert dm.motor1.status.active == ["ALARM"]
    assert dm.motor2.speed_rpm == -100
    assert dm.motor2.current_a == pytest.approx(0.6)
    assert dm.motor2.position == 0
    assert dm.motor2.status.active == []


def test_decode_pnt_main_data_wrong_length():
    with pytest.raises(ValueError):
        decode_pnt_main_data([0] * 7)
