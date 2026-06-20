# mdrobot_ros2

MDROBOT MD 시리즈 BLDC/DC 모터 컨트롤러를 **RS485 / Modbus RTU**로 제어하는 범용 드라이버.
순수 Python 통신 라이브러리(`mdrobot`)와 그 위의 ROS 2 노드(`mdrobot_ros2_driver`)로 구성된다.

- **싱글 채널**: MD400 등 모터 1개 제어 장치 → `SingleMotorDriver`
- **듀얼 채널**: PNT50, MD400T, MD200T 등 모터 2개 제어 장치 → `DualMotorDriver`

범용 모터 드라이버이므로 **로봇 기구학(차동구동·odom 등)은 포함하지 않는다** — 채널별 속도/위치
명령과 모터 상태만 노출하며, 기구학 변환은 이를 사용하는 상위 로봇 패키지의 몫이다.

> 검증 하드웨어: MD400(싱글, 8극/10극 모터) + PNT50·MD400T(듀얼). 실물 확정 동작을 코드/문서에 반영했다.

## 구성

```text
mdrobot/                  # 순수 Python 라이브러리 (crc/frame/codec/protocol/registers/status/units/device)
mdrobot_ros2_driver/      # ROS 2 ament_python 노드 패키지 (mdrobot에 의존)
tests/unit/               # 단위 테스트 (하드웨어 불필요)
docs/manual/              # 사용자 매뉴얼
examples/                 # 최소 사용 예제
```

## 요구사항

- Python ≥ 3.10, [`pyserial`](https://pypi.org/project/pyserial/) ≥ 3.5 (실제 직렬 통신 시)
- ROS 2 (Jazzy 검증) — ROS 2 노드 사용 시
- RS485 어댑터(USB-직렬). 기본 통신: **19200 8N1**

## 설치

### 라이브러리

```bash
pip install -e .            # 또는: pip install -e '.[serial]'  (pyserial 포함)
```

### ROS 2 노드 (colcon 워크스페이스 루트에서)

```bash
pip install -e .                      # mdrobot 라이브러리가 import 가능해야 함
colcon build --packages-select mdrobot_ros2_driver
source install/setup.bash
```

## 빠른 시작 — Python 라이브러리

```python
from mdrobot import SingleMotorDriver, DualMotorDriver

# 싱글 (MD400)
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    print(d.get_version(), d.get_voltage(), "V")
    d.enable()                 # UI_COM=1 + START_STOP arm (구동 전 필수)
    d.set_velocity(40)         # +CCW, 저속부터
    # ... 잠시 후 ...
    d.stop(); d.torque_off()

# 듀얼 (PNT50)
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocities(40, 40)   # 명령 후 회전까지 ~1s 지연 있음
    d.stop(); d.torque_off_both()
```

raw 접근은 `d.client`로 항상 유지된다(고수준 API가 없는 PID/CMD 직접 제어). 자세한 사용법·안전·
트러블슈팅은 [`docs/manual/`](./docs/manual/) 참고. 최소 예제는 [`examples/`](./examples/).

## 빠른 시작 — ROS 2 노드

```bash
# 싱글 (MD400). 일부 MD400은 serial 구동에 USE_LIMIT_SW=0 필요 → use_limit_sw:=0
ros2 launch mdrobot_ros2_driver single.launch.py port:=/dev/ttyUSB0 use_limit_sw:=0
# 듀얼 (PNT50)
ros2 launch mdrobot_ros2_driver dual.launch.py port:=/dev/ttyUSB0
```

토픽/서비스(std_msgs/std_srvs/sensor_msgs/diagnostic_msgs): `~/cmd_velocity`, `~/cmd_position`,
`~/joint_states`, `~/diagnostics`, `~/enable`/`~/stop`/`~/torque_off` 등. 상세는
[`mdrobot_ros2_driver/README.md`](./mdrobot_ros2_driver/README.md).

### joint_states 단위 (rad)

`counts_per_rev`(채널별) 파라미터를 주면 `~/joint_states`가 SI(position=rad, velocity=rad/s)로
발행된다. 미설정 시 raw(count, rpm)로 발행하고 경고한다. 값은 모터마다 다르므로(홀 ≈ 3×극수,
엔코더 = 4×PPR) **실측 권장** — 출력축을 정해진 N바퀴 돌리고 `Δcount / N`으로 구한다. 예:
8극 모터 → 24, 10극 모터 → 30. (감속비는 포함하지 않는다 — 모터축 기준.)

## 안전

- **구동 전 `enable()`**, 항상 **저속부터** 시작하고 비상정지/전원 차단을 준비한다.
- ROS 2 노드는 `command_timeout`(기본 0.5s) 내 새 속도 명령이 없으면 자동 정지하고, 종료 시 stop+torque_off.
- **모터가 안 돌면**: `UI_COM=1` → `START_STOP` arm → `USE_LIMIT_SW`(일부 MD400은 serial 구동에 0 필요) 순으로 확인한다.

## 테스트

```bash
pytest tests/unit                 # 단위 테스트(하드웨어 불필요)
```

## 라이선스

[Apache License 2.0](./LICENSE).
