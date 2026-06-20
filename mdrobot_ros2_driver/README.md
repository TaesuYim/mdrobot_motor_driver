# mdrobot_ros2_driver

MDROBOT MD 시리즈 BLDC/DC 모터 컨트롤러(RS485 / Modbus RTU)용 **범용** ROS 2 드라이버 노드.
싱글(MD400)/듀얼(PNT50) 채널을 `device_type` 파라미터로 공통 처리한다. **로봇 기구학은 두지
않는다** — 채널별 속도/위치 명령과 모터 상태만 노출하며, `/cmd_vel`→wheel 변환·odom은 이
드라이버를 사용하는 상위 로봇 패키지의 몫이다.

## 의존성

- ROS 2 (Jazzy 검증), `rclpy`, `std_msgs`, `std_srvs`, `sensor_msgs`, `diagnostic_msgs`
- `mdrobot` 통신 라이브러리(이 저장소 루트). ROS 패키지가 아니라 **Python 의존성**이므로
  노드 실행 환경에서 import 가능해야 한다:
  ```bash
  # repo 루트에서 (둘 중 하나)
  pip install -e . --break-system-packages      # 시스템 python에 설치
  # 또는 실행 시 PYTHONPATH에 repo 루트 추가
  export PYTHONPATH=$PWD:$PYTHONPATH
  ```

## 빌드

```bash
cd <repo>            # colcon workspace 루트
colcon build --packages-select mdrobot_ros2_driver
source install/setup.bash
```

## 실행

```bash
# 싱글 (MD400)
ros2 launch mdrobot_ros2_driver single.launch.py port:=/dev/ttyUSB1
# 듀얼 (PNT50)
ros2 launch mdrobot_ros2_driver dual.launch.py port:=/dev/ttyUSB0
# 또는 직접
ros2 run mdrobot_ros2_driver motor_driver_node --ros-args -p device_type:=dual -p port:=/dev/ttyUSB0
```

## 인터페이스

파라미터: `port`, `baudrate`(19200), `motor_id`(1), `device_type`(single|dual),
`command_timeout`(0.5s, 0=비활성), `publish_rate`(20Hz), `diag_rate`(2Hz),
`position_max_rpm`(100), `joint_names`, `auto_enable`(true),
`use_limit_sw`(-1 유지 / 0 비활성 / 1 사용).

> **`use_limit_sw`**: MD400에 엔코더를 연결하면 엔코더 A/B가 limit switch 입력과 핀을
> 공유해 `USE_LIMIT_SW=1`이면 모션이 막힌다(구형 MD400). 엔코더 연결 환경에서는 `0`으로
> 둔다. `-1`(기본)은 장치 현재 설정을 건드리지 않는다.

| 종류 | 이름 | 타입 | 의미 |
|---|---|---|---|
| sub | `~/cmd_velocity` | `std_msgs/Float64MultiArray` | `[rpm]`(single) / `[rpm1,rpm2]`(dual) |
| sub | `~/cmd_position` | `std_msgs/Float64MultiArray` | `[count]` / `[count1,count2]` (속도=`position_max_rpm`) |
| pub | `~/joint_states` | `sensor_msgs/JointState` | position=count, velocity=rpm (raw; rad는 Phase 11) |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` | voltage / status / alarm |
| srv | `~/enable` `~/disable` `~/stop` `~/brake` `~/torque_off` `~/reset_alarm` `~/reset_position` | `std_srvs/Trigger` | — |

부호 규약(실물 확정): `+` = position 증가 방향(CCW). 단위는 raw count/rpm이며 물리 단위(rad)는
Phase 11(encoder 특성화)에서 채운다.

### 예시

```bash
# 듀얼 양 모터 +40 rpm
ros2 topic pub -1 /pnt50/mdrobot_motor_driver/cmd_velocity std_msgs/Float64MultiArray "{data: [40, 40]}"
# 정지
ros2 service call /pnt50/mdrobot_motor_driver/stop std_srvs/srv/Trigger
# 싱글 위치 이동 (+80 count)
ros2 topic pub -1 /md400/mdrobot_motor_driver/cmd_position std_msgs/Float64MultiArray "{data: [80]}"
```

## 안전

- `command_timeout`>0이면 마지막 `~/cmd_velocity` 후 그 시간 내 새 명령이 없으면 자동 정지.
- 노드 종료 시 `stop` + `torque_off`.
- 콜백은 단일 스레드 executor에서 순차 실행되어 직렬 포트 접근이 겹치지 않는다.
