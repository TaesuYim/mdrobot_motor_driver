#!/usr/bin/env python3
"""범용 MDROBOT 모터 드라이버 ROS 2 노드.

싱글(MD400)/듀얼(PNT50) 컨트롤러를 `device_type` 파라미터로 공통 처리한다. 로봇 기구학은
두지 않는다(CLAUDE.md §1) — 채널별 속도/위치 명령과 모터 상태만 노출한다. 메시지는
std_msgs / std_srvs / sensor_msgs / diagnostic_msgs(표준)만 사용한다.

인터페이스
----------
파라미터:
  port (str)              직렬 포트, 예: /dev/ttyUSB0
  baudrate (int=19200)
  motor_id (int=1)
  device_type (str)       'single' | 'dual'
  command_timeout (float=0.5)  속도 명령 watchdog 초. 0이면 비활성
  publish_rate (float=20.0)    joint_states 발행 Hz
  diag_rate (float=2.0)        diagnostics 발행 Hz
  position_max_rpm (int=100)   위치 명령 시 최대 속도
  joint_names (str[])     비우면 device_type에 따라 자동(motor1[, motor2])
  auto_enable (bool=True) 시작 시 enable() 호출
  counts_per_rev (double[]) 채널별 1회전당 count. 설정하면 joint_states를 SI(rad, rad/s)로
                  발행한다. 미설정/0/길이 불일치면 raw(count, rpm)로 발행하고 1회 경고한다.
                  값은 모터마다 다르다(홀≈3×극수, 엔코더 4×PPR) — 실측 권장
                  (examples/calibrate_counts_per_rev.py). 감속비는 포함하지 않는다
                  (모터축 기준; 바퀴 변환은 상위 로봇/오도메트리 계층 몫).

구독 (std_msgs/Float64MultiArray):
  ~/cmd_velocity  data=[rpm]            (single) | [rpm1, rpm2] (dual)
  ~/cmd_position  data=[count]          (single) | [count1, count2] (dual)
                  (최대 속도는 position_max_rpm)

발행:
  ~/joint_states (sensor_msgs/JointState)
      counts_per_rev 설정 시: position=rad, velocity=rad/s (SI 표준)
      미설정 시:            position=count, velocity=rpm  (raw)
  ~/diagnostics  (diagnostic_msgs/DiagnosticArray)  voltage / status bits / alarm

서비스 (std_srvs/Trigger):
  ~/enable ~/disable ~/stop ~/brake ~/torque_off ~/reset_alarm ~/reset_position

안전: command_timeout>0이면 마지막 ~/cmd_velocity 이후 그 시간 내 새 명령이 없을 때 정지한다.
콜백은 단일 스레드 executor에서 순차 실행되므로 직렬 포트 접근이 겹치지 않는다.
"""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from mdrobot import DualMotorDriver, SingleMotorDriver
from mdrobot import registers as reg
from mdrobot.exceptions import MdrobotError
from mdrobot.units import counts_to_rad, rpm_to_rad_s


class MotorDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_motor_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("motor_id", 1)
        self.declare_parameter("device_type", "single")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("diag_rate", 2.0)
        self.declare_parameter("position_max_rpm", 100)
        self.declare_parameter("joint_names", [""])
        self.declare_parameter("auto_enable", True)
        # 채널별 counts_per_rev. 설정 시 joint_states를 SI(rad, rad/s)로 발행.
        # 기본 [0.0]=미설정 → raw(count, rpm) 발행. 모터마다 다르므로 함부로 기본값을 박지 않는다.
        self.declare_parameter("counts_per_rev", [0.0])
        # USE_LIMIT_SW 정책: -1=장치 설정 유지(기본), 0=limit 비활성, 1=limit 사용.
        # MD400에 엔코더를 연결하면 엔코더 A/B가 limit 입력과 핀을 공유해 USE_LIMIT_SW=1이면
        # 모션이 막힌다(confirmed-behavior §7). 그런 경우 0으로 둔다.
        self.declare_parameter("use_limit_sw", -1)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.motor_id = int(self.get_parameter("motor_id").value)
        self.device_type = str(self.get_parameter("device_type").value).lower()
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.position_max_rpm = int(self.get_parameter("position_max_rpm").value)

        if self.device_type not in ("single", "dual"):
            raise ValueError(f"device_type must be 'single' or 'dual', got {self.device_type!r}")
        self.channels = 1 if self.device_type == "single" else 2

        joint_names = [n for n in self.get_parameter("joint_names").value if n]
        if not joint_names:
            joint_names = ["motor1"] if self.channels == 1 else ["motor1", "motor2"]
        if len(joint_names) != self.channels:
            raise ValueError(f"joint_names {joint_names} must have {self.channels} entries")
        self.joint_names = joint_names

        # counts_per_rev: 길이가 채널 수와 같고 모두 >0이면 SI 발행, 아니면 raw + 경고.
        cpr = [float(v) for v in self.get_parameter("counts_per_rev").value]
        if len(cpr) == self.channels and all(v > 0 for v in cpr):
            self.counts_per_rev = cpr
            self.publish_si = True
        else:
            self.counts_per_rev = None
            self.publish_si = False

        # 드라이버 연결. open 직후 어댑터 안정화 전 첫 트랜잭션이 흔들릴 수 있어 ping으로 재시도.
        driver_cls = SingleMotorDriver if self.channels == 1 else DualMotorDriver
        self.driver = driver_cls.open(self.port, self.baudrate, slave_id=self.motor_id)
        for attempt in range(5):
            if self.driver.ping():
                break
            self.get_logger().warn(f"초기 통신 재시도 {attempt + 1}/5 ({self.port})")
            time.sleep(0.2)
        else:
            self.driver.close()
            raise RuntimeError(f"{self.port} 초기 통신 실패 — baudrate/포트/배선 확인")
        self.get_logger().info(
            f"열림: {self.port} @ {self.baudrate}, id={self.motor_id}, "
            f"type={self.device_type}, version={self.driver.get_version()}, "
            f"voltage={self.driver.get_voltage()}V"
        )
        self._apply_use_limit_sw()
        if bool(self.get_parameter("auto_enable").value):
            self.driver.enable()
            self.get_logger().info("enable() 완료 (UI_COM=1 + START_STOP arm)")

        self._last_vel_time = None  # 마지막 속도 명령 시각(monotonic ns)

        # 구독
        self.create_subscription(Float64MultiArray, "~/cmd_velocity", self._on_cmd_velocity, 10)
        self.create_subscription(Float64MultiArray, "~/cmd_position", self._on_cmd_position, 10)

        # 발행
        self._joint_pub = self.create_publisher(JointState, "~/joint_states", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "~/diagnostics", 10)

        # 타이머
        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        diag_rate = max(0.2, float(self.get_parameter("diag_rate").value))
        self.create_timer(1.0 / rate, self._publish_joint_states)
        self.create_timer(1.0 / diag_rate, self._publish_diagnostics)
        if self.command_timeout > 0:
            self.create_timer(min(0.1, self.command_timeout / 2.0), self._watchdog)

        # 서비스
        self._make_service("~/enable", lambda: self.driver.enable())
        self._make_service("~/disable", lambda: self.driver.disable())
        self._make_service("~/stop", lambda: self.driver.stop())
        self._make_service("~/torque_off", self._svc_torque_off)
        self._make_service("~/brake", self._svc_brake)
        self._make_service("~/reset_alarm", lambda: self.driver.reset_alarm())
        self._make_service("~/reset_position", lambda: self.driver.reset_position())

        if self.publish_si:
            self.get_logger().info(
                f"joint_states 단위=SI (rad, rad/s), counts_per_rev={self.counts_per_rev}"
            )
        else:
            self.get_logger().warn(
                "joint_states 단위=raw (position=count, velocity=rpm). "
                f"SI(rad)로 발행하려면 counts_per_rev를 {self.channels}개 양수로 설정하세요 "
                "(예: counts_per_rev:=[24.0]). 값은 examples/calibrate_counts_per_rev.py로 실측."
            )
        self.get_logger().info("mdrobot_motor_driver 준비 완료")

    def _apply_use_limit_sw(self) -> None:
        """use_limit_sw 파라미터에 따라 PID_USE_LIMIT_SW(및 듀얼 PID29)를 설정한다.

        -1이면 장치 설정을 건드리지 않고 현재 값만 로깅한다.
        """
        want = int(self.get_parameter("use_limit_sw").value)
        try:
            cur = self.driver.client.read_register(reg.PID_USE_LIMIT_SW)
        except MdrobotError:
            cur = None
        if want < 0:
            self.get_logger().info(f"USE_LIMIT_SW 유지 (현재={cur})")
            return
        val = 1 if want else 0
        try:
            self.driver.client.write_register(reg.PID_USE_LIMIT_SW, val)
            if self.channels == 2:
                self.driver.client.write_register(reg.PID_USE_LIMIT_SW2, val)
            self.get_logger().info(f"USE_LIMIT_SW {cur} -> {val} (param use_limit_sw={want})")
        except MdrobotError as exc:
            self.get_logger().error(f"USE_LIMIT_SW 설정 실패: {type(exc).__name__}: {exc}")

    # --- 서비스 헬퍼 -------------------------------------------------------------------
    def _make_service(self, name: str, action) -> None:
        def cb(_req, resp):
            try:
                action()
                resp.success = True
                resp.message = "ok"
            except MdrobotError as exc:
                resp.success = False
                resp.message = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(f"{name} 실패: {resp.message}")
            return resp

        self.create_service(Trigger, name, cb)

    def _svc_torque_off(self) -> None:
        if self.channels == 1:
            self.driver.torque_off()
        else:
            self.driver.torque_off_both()

    def _svc_brake(self) -> None:
        if self.channels == 1:
            self.driver.brake()
        else:
            self.driver.brake_both()

    # --- 명령 콜백 ---------------------------------------------------------------------
    def _on_cmd_velocity(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) != self.channels:
            self.get_logger().warn(f"cmd_velocity는 {self.channels}개 값이 필요. 받음: {data}")
            return
        try:
            if self.channels == 1:
                self.driver.set_velocity(int(round(data[0])))
            else:
                self.driver.set_velocities(int(round(data[0])), int(round(data[1])))
            self._last_vel_time = self.get_clock().now().nanoseconds
        except MdrobotError as exc:
            self.get_logger().error(f"cmd_velocity 실패: {type(exc).__name__}: {exc}")

    def _on_cmd_position(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) != self.channels:
            self.get_logger().warn(f"cmd_position은 {self.channels}개 값이 필요. 받음: {data}")
            return
        try:
            if self.channels == 1:
                self.driver.move_to(int(round(data[0])), self.position_max_rpm)
            else:
                self.driver.move_to_both(
                    int(round(data[0])), int(round(data[1])), self.position_max_rpm
                )
            self._last_vel_time = None  # 위치 모드 진입 시 속도 watchdog 해제
        except MdrobotError as exc:
            self.get_logger().error(f"cmd_position 실패: {type(exc).__name__}: {exc}")

    # --- watchdog ----------------------------------------------------------------------
    def _watchdog(self) -> None:
        if self._last_vel_time is None:
            return
        elapsed = (self.get_clock().now().nanoseconds - self._last_vel_time) / 1e9
        if elapsed >= self.command_timeout:
            try:
                self.driver.stop()
            except MdrobotError as exc:
                self.get_logger().error(f"watchdog stop 실패: {exc}")
            self._last_vel_time = None
            self.get_logger().warn(f"command_timeout({self.command_timeout}s) 초과 → 정지")

    # --- 발행 --------------------------------------------------------------------------
    def _publish_joint_states(self) -> None:
        if not rclpy.ok():  # 종료 중이면 무효 context에 publish하지 않는다
            return
        try:
            mon = self.driver.read_monitor()
        except MdrobotError as exc:
            self.get_logger().warn(f"monitor 읽기 실패: {type(exc).__name__}: {exc}", throttle_duration_sec=2.0)
            return
        if self.channels == 1:
            counts = [mon.position]
            rpms = [mon.speed_rpm]
        else:
            counts = [mon.motor1.position, mon.motor2.position]
            rpms = [mon.motor1.speed_rpm, mon.motor2.speed_rpm]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        if self.publish_si:
            # 위치만 counts_per_rev가 필요. 속도는 rpm→rad/s로 출처와 무관.
            js.position = [counts_to_rad(c, cpr) for c, cpr in zip(counts, self.counts_per_rev)]
            js.velocity = [rpm_to_rad_s(r) for r in rpms]
        else:
            js.position = [float(c) for c in counts]
            js.velocity = [float(r) for r in rpms]
        try:
            self._joint_pub.publish(js)
        except Exception:  # noqa: BLE001 - 종료 teardown 중 무효 context 경합 무시
            pass

    def _publish_diagnostics(self) -> None:
        if not rclpy.ok():
            return
        status = DiagnosticStatus()
        status.name = "mdrobot/motor_driver"
        status.hardware_id = f"{self.port}#{self.motor_id}"
        try:
            voltage = self.driver.get_voltage()
            bits = self.driver.get_status()
            status.level = DiagnosticStatus.ERROR if bits.alarm else DiagnosticStatus.OK
            status.message = "ALARM" if bits.alarm else "OK"
            status.values = [
                KeyValue(key="voltage_V", value=f"{voltage:.1f}"),
                KeyValue(key="status1", value=",".join(bits.active) or "none"),
                KeyValue(key="device_type", value=self.device_type),
            ]
        except MdrobotError as exc:
            status.level = DiagnosticStatus.ERROR
            status.message = f"read fail: {type(exc).__name__}"
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        try:
            self._diag_pub.publish(arr)
        except Exception:  # noqa: BLE001 - 종료 teardown 중 무효 context 경합 무시
            pass

    # --- 종료 --------------------------------------------------------------------------
    def shutdown(self) -> None:
        """정지(stop+torque_off) 후 포트를 닫는다. 직렬이 wedge돼도 종료가 막히지 않도록
        모든 예외를 흡수한다(write_timeout으로 직렬 write는 무한 대기하지 않음)."""
        try:
            self.driver.stop()
            self._svc_torque_off()
            self.get_logger().info("종료: stop + torque_off")
        except Exception as exc:  # noqa: BLE001 - 종료는 항상 진행되어야 함
            self.get_logger().error(f"종료 정지 실패(무시하고 종료): {type(exc).__name__}: {exc}")
        finally:
            try:
                self.driver.close()
            except Exception:  # noqa: BLE001
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MotorDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C(SIGINT) 또는 SIGTERM에 의한 정상 종료 — traceback 없이 마무리.
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
