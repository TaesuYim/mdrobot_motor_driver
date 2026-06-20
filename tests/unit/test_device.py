"""SingleMotorDriver / DualMotorDriver 단위 테스트 (가짜 장치, 하드웨어 없음).

`FakeDevice`는 write 요청의 함수 코드를 보고 실제 컨트롤러처럼 echo/ack 응답을 만들어
준다. 따라서 드라이버가 보내는 프레임과 read 디코딩을 모두 검증할 수 있다.
"""

import pytest

from mdrobot import registers as reg
from mdrobot.crc import append_crc
from mdrobot.device import DualMotorDriver, SingleMotorDriver
from mdrobot.protocol import ModbusClient


class FakeDevice:
    """함수 코드별로 응답을 자동 생성하는 가짜 transport.

    - 0x03 read : registers[pid]에서 word를 꺼내 응답(없으면 0).
    - 0x06 write: 요청 echo(8 byte)를 그대로 응답.
    - 0x10 write: start address/count echo(8 byte) 응답.
    write 프레임은 `self.frames`에 전체 요청 bytes로 기록한다.
    """

    def __init__(self, registers=None) -> None:
        self.registers = {pid: list(words) for pid, words in (registers or {}).items()}
        self.frames: list[bytes] = []
        self._rx = bytearray()

    def write(self, data: bytes) -> int:
        data = bytes(data)
        self.frames.append(data)
        self._rx += self._respond(data)
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def flush_input(self) -> None:
        pass

    def _respond(self, req: bytes) -> bytes:
        slave, func = req[0], req[1]
        pid = (req[2] << 8) | req[3]
        if func == 0x03:
            count = (req[4] << 8) | req[5]
            words = self.registers.get(pid, [])
            words = (words + [0] * count)[:count]
            body = bytearray((slave, 0x03, 2 * count))
            for w in words:
                body.append((w >> 8) & 0xFF)
                body.append(w & 0xFF)
            return append_crc(bytes(body))
        if func == 0x06:
            return req  # echo
        if func == 0x10:
            return append_crc(req[:6])
        raise AssertionError(f"unexpected func 0x{func:02x}")


def make_single(registers=None):
    dev = FakeDevice(registers)
    return SingleMotorDriver(ModbusClient(dev, slave_id=1)), dev


def make_dual(registers=None):
    dev = FakeDevice(registers)
    return DualMotorDriver(ModbusClient(dev, slave_id=1)), dev


def w1(pid, word):
    from mdrobot import frame
    return frame.build_write_single_request(1, pid, word)


# --- 공통 / enable -------------------------------------------------------------------

def test_enable_writes_uicom_comtar_then_start():
    drv, dev = make_single()
    drv.enable()
    assert dev.frames == [
        w1(reg.PID_UI_COM, 1),
        w1(reg.PID_COM_TAR_SPEED, 0),
        w1(reg.PID_START_STOP, 1),
    ]


def test_disable_clears_start_stop():
    drv, dev = make_single()
    drv.disable()
    assert dev.frames == [w1(reg.PID_START_STOP, 0)]


def test_reset_alarm_uses_command_gateway():
    drv, dev = make_single()
    drv.reset_alarm()
    assert dev.frames == [w1(reg.PID_COMMAND, reg.CMD_ALARM_RESET)]


def test_ping_true_false():
    drv, _ = make_single({reg.PID_VERSION: [0x2D]})
    assert drv.ping() is True
    drv2, _ = make_single()  # 버전 응답은 0 -> 통신 자체는 성공
    assert drv2.ping() is True


# --- SingleMotorDriver ---------------------------------------------------------------

def test_single_set_velocity_positive():
    drv, dev = make_single()
    drv.set_velocity(100)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 100)]


def test_single_set_velocity_negative_twos_complement():
    drv, dev = make_single()
    drv.set_velocity(-100)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 0xFF9C)]


def test_single_stop_brake_torque_off():
    drv, dev = make_single()
    drv.stop()
    drv.brake()
    drv.torque_off()
    assert dev.frames == [
        w1(reg.PID_VEL_CMD, 0),
        w1(reg.PID_BRAKE, 0),
        w1(reg.PID_TQ_OFF, 0),
    ]


def test_single_get_speed_signed():
    drv, _ = make_single({reg.PID_INT_RPM_DATA: [0xFF9C]})  # -100
    assert drv.get_speed() == -100


def test_single_get_voltage_and_current():
    drv, _ = make_single({reg.PID_VOLT_IN: [239], reg.PID_TQ_DATA: [12]})
    assert drv.get_voltage() == pytest.approx(23.9)
    assert drv.get_current() == pytest.approx(1.2)


def test_single_get_position_long():
    drv, _ = make_single({reg.PID_POSI_DATA: [0x5678, 0x1234]})  # low word first
    assert drv.get_position() == 0x12345678


def test_single_read_monitor():
    drv, _ = make_single({reg.PID_MONITOR: [0x0064, 0x000C, 0x0000, 0x0005, 0x0000, 0x0000]})
    mon = drv.read_monitor()
    assert mon.speed_rpm == 100
    assert mon.position == 5


def wN(pid, words):
    from mdrobot import frame
    return frame.build_write_multiple_request(1, pid, words)


def test_single_move_to_encodes_position_low_word_first_plus_speed():
    drv, dev = make_single()
    drv.move_to(0x12345678, speed=50)
    # position low word first [0x5678, 0x1234] + speed word
    assert dev.frames == [wN(reg.PID_POSI_VEL_CMD, [0x5678, 0x1234, 50])]


def test_single_move_to_small_target():
    drv, dev = make_single()
    drv.move_to(80, speed=50)
    assert dev.frames == [wN(reg.PID_POSI_VEL_CMD, [80, 0, 50])]


def test_single_move_by_negative_delta():
    drv, dev = make_single()
    drv.move_by(-2, speed=40)
    # -2 -> 0xFFFFFFFE -> low 0xFFFE, high 0xFFFF
    assert dev.frames == [wN(reg.PID_INC_POSI_VEL_CMD, [0xFFFE, 0xFFFF, 40])]


def test_single_get_in_position():
    drv, _ = make_single({reg.PID_IN_POSITION_OK: [1]})
    assert drv.get_in_position() is True
    drv2, _ = make_single({reg.PID_IN_POSITION_OK: [0]})
    assert drv2.get_in_position() is False


# --- DualMotorDriver -----------------------------------------------------------------

def test_dual_set_velocities_uses_two_legacy_pids():
    drv, dev = make_dual()
    drv.set_velocities(40, -40)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 40), w1(reg.PID_VEL_CMD2, 0xFFD8)]


def test_dual_set_velocity_channel():
    drv, dev = make_dual()
    drv.set_velocity(1, 30)
    drv.set_velocity(2, 30)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 30), w1(reg.PID_VEL_CMD2, 30)]


def test_dual_set_velocity_bad_channel():
    drv, _ = make_dual()
    with pytest.raises(ValueError):
        drv.set_velocity(3, 10)


def test_dual_brake_and_torque_off_flags():
    drv, dev = make_dual()
    drv.brake_both()
    drv.brake(1)
    drv.brake(2)
    drv.torque_off_both()
    drv.torque_off(1)
    drv.torque_off(2)
    assert dev.frames == [
        w1(reg.PID_PNT_BRAKE, 0x0101),
        w1(reg.PID_PNT_BRAKE, 0x0001),
        w1(reg.PID_PNT_BRAKE, 0x0100),
        w1(reg.PID_PNT_TQ_OFF, 0x0101),
        w1(reg.PID_PNT_TQ_OFF, 0x0001),
        w1(reg.PID_PNT_TQ_OFF, 0x0100),
    ]


def test_dual_stop_sends_both_zero():
    drv, dev = make_dual()
    drv.stop()
    assert dev.frames == [w1(reg.PID_VEL_CMD, 0), w1(reg.PID_VEL_CMD2, 0)]


def test_dual_read_monitor_and_get_speed():
    regs = {reg.PID_PNT_MONITOR: [0x0064, 0x0005, 0x0000, 0xFFCE, 0xFFFF, 0xFFFF, 0x0000]}
    drv, _ = make_dual(regs)
    dm = drv.read_monitor()
    assert dm.motor1.speed_rpm == 100
    assert dm.motor2.speed_rpm == -50
    assert drv.get_speed(1) == 100
    assert drv.get_speed(2) == -50


def test_dual_read_main_data_has_current():
    regs = {reg.PID_PNT_MAIN_DATA: [0x0064, 0x000C, 0x0000, 0x0000, 0x0000, 0x0006, 0x0000, 0x0000, 0x0000]}
    drv, _ = make_dual(regs)
    dm = drv.read_main_data()
    assert dm.motor1.current_a == pytest.approx(1.2)
    assert dm.motor2.current_a == pytest.approx(0.6)


def test_dual_move_to_both_matches_golden_vector():
    # doc 07 §3.7: pos1=0x12345678 spd1=0x9ABC pos2=0x34567890 spd2=0x0123
    # -> words [0x5678, 0x1234, 0x9ABC, 0x7890, 0x3456, 0x0123]
    drv, dev = make_dual()
    drv.move_to_both(0x12345678, 0x34567890, speed1=0x9ABC, speed2=0x0123)
    assert dev.frames == [wN(reg.PID_PNT_POSI_VEL_CMD, [0x5678, 0x1234, 0x9ABC, 0x7890, 0x3456, 0x0123])]


def test_dual_move_to_both_default_speed2_follows_speed1():
    drv, dev = make_dual()
    drv.move_to_both(50, 80, speed1=60)
    assert dev.frames == [wN(reg.PID_PNT_POSI_VEL_CMD, [50, 0, 60, 80, 0, 60])]


def test_dual_move_by_both_negative_delta():
    drv, dev = make_dual()
    drv.move_by_both(20, -20, speed1=50)
    # -20 -> 0xFFFFFFEC -> low 0xFFEC, high 0xFFFF
    assert dev.frames == [wN(reg.PID_PNT_INC_POSI_VEL_CMD, [20, 0, 50, 0xFFEC, 0xFFFF, 50])]


def test_dual_get_positions_and_reset():
    regs = {reg.PID_PNT_MONITOR: [0x0000, 0x0032, 0x0000, 0x0000, 0x0050, 0x0000, 0x0000]}
    drv, dev = make_dual(regs)
    assert drv.get_positions() == (50, 80)
    assert drv.get_position(2) == 80
    drv.reset_position()
    # get_positions/get_position은 read 요청도 보내므로 마지막 write만 확인.
    assert dev.frames[-1] == w1(reg.PID_POSI_RESET, 0)
