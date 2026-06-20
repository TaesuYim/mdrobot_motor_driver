# mdrobot 사용자 매뉴얼

MDROBOT MD 시리즈 모터 컨트롤러를 `mdrobot` 라이브러리/ROS 2 노드로 안전하게 연결·읽기·구동하는
방법을 설명한다. 실물로 확정된 동작은 [`../dev/confirmed-behavior.md`](../dev/confirmed-behavior.md)
기준이다.

## 1. 연결

1. **결선**: RS485 어댑터(USB-직렬)를 컨트롤러의 RS485 A/B에 연결. 기본 통신 **19200 8N1**.
2. **포트 권한**(Linux): 사용자가 `dialout` 그룹에 없으면 포트 접근이 막힌다.
   ```bash
   sudo usermod -aG dialout $USER   # 재로그인 필요
   # 또는 일시적으로: sudo chmod a+rw /dev/ttyUSB0
   ```
3. **장치 식별**: 같은 ID(기본 1)를 여러 대가 쓰면 한 버스에서 충돌한다. 서로 다른 포트(버스)면 무방.
   듀얼 전용 PID(`PID_PNT_MONITOR(216)` 등)에 응답하면 듀얼(PNT50), 무응답이면 싱글(MD400)이다.
   - 검증 장비 기준: MD400 = `/dev/ttyUSB1`(FTDI), PNT50 = `/dev/ttyUSB0`(CH340).

## 2. 읽기 (무회전)

```python
from mdrobot import SingleMotorDriver
with SingleMotorDriver.open("/dev/ttyUSB1") as d:
    print("version:", d.get_version())
    print("voltage:", d.get_voltage(), "V")
    print("status :", d.get_status().active)   # alarm/over_volt 등 활성 bit
    print("monitor:", d.read_monitor())         # speed/current/position/status
```

읽기만으로는 모터가 돌지 않는다. 통신이 실패하면 baudrate / controller ID / 결선 / CRC 중
원인을 분리해 확인한다.

## 3. 구동

> **공통 전제**: 구동 전 `enable()` 호출이 필요하다. `enable()`은 `PID_UI_COM=1`(serial 단독) +
> `PID_START_STOP=1`(run-latch arm)을 설정한다. 이것 없이 속도 명령은 echo만 되고 모터가 돌지 않는다.

### 싱글 (MD400)

```python
with SingleMotorDriver.open("/dev/ttyUSB1") as d:
    d.enable()
    d.set_velocity(40)      # signed rpm. + = CCW(position 증가)
    # ...관찰...
    d.set_velocity(-40)
    d.stop()                # 속도 0 (감속 정지)
    d.torque_off()          # free 상태
    # 위치 제어 (UI_COM=1만 필요)
    d.reset_position()
    d.move_to(80, speed=60) # 절대 이동, 도달 시 정지
    d.wait_in_position()
    d.move_by(-40, speed=60)
```

### 듀얼 (PNT50)

```python
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocities(40, 40)      # 모터1=VEL_CMD(130), 모터2=VEL_CMD2(131)
    # 주의: 명령 후 회전까지 ~1초 지연이 있다 (즉시 0을 보내면 회전을 놓침)
    d.stop()
    d.move_to_both(50, 50, speed1=60)   # 듀얼 동시 위치
    d.torque_off_both()
```

### 부호/방향

`+` = CCW = position 증가, `-` = CW = position 감소 (싱글/듀얼 공통). 실제 회전 방향은 설치 상태에서
한 번 확인한다.

## 4. ROS 2 노드

```bash
ros2 launch mdrobot_ros2_driver single.launch.py port:=/dev/ttyUSB1 use_limit_sw:=0
ros2 launch mdrobot_ros2_driver dual.launch.py   port:=/dev/ttyUSB0
```

| 인터페이스 | 이름 | 타입 |
|---|---|---|
| 명령 | `~/cmd_velocity` | `std_msgs/Float64MultiArray` (`[rpm]` / `[rpm1,rpm2]`) |
| 명령 | `~/cmd_position` | `std_msgs/Float64MultiArray` (`[count]` / `[count1,count2]`) |
| 상태 | `~/joint_states` | `sensor_msgs/JointState` (`counts_per_rev` 설정 시 rad/rad·s, 미설정 시 count/rpm) |
| 진단 | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` (voltage/status/alarm) |
| 서비스 | `~/enable` `~/disable` `~/stop` `~/brake` `~/torque_off` `~/reset_alarm` `~/reset_position` | `std_srvs/Trigger` |

```bash
ros2 topic pub -1 /pnt50/mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40,40]}"
ros2 service call /pnt50/mdrobot_motor_driver/stop std_srvs/srv/Trigger
```

## 5. 안전

- 항상 **저속부터** 시작하고, 비상정지/전원 차단을 손 닿는 곳에 둔다.
- ROS 2 노드는 `command_timeout`(기본 0.5s) 내 새 속도 명령이 없으면 자동 정지하고, 종료 시
  stop + torque_off를 보낸다.
- 위치 제어는 목표 도달 시 자동 정지하지만, 잘못된 큰 목표는 과도한 회전을 유발할 수 있다 — 작은
  값부터 확인한다.

## 6. 트러블슈팅 — 모터가 안 돈다

순서대로 확인한다(자세한 근거는 [`confirmed-behavior.md`](../dev/confirmed-behavior.md)).

1. `enable()`을 호출했는가? (`UI_COM=1` + `START_STOP` arm)
2. **MD400**: `PID_USE_LIMIT_SW(17)=0`인가? 이 MD400은 serial 구동에 `0`이 필요하다. 엔코더를
   연결하면 A/B가 limit 입력과 핀을 공유해 더더욱 `0`이 필요하다. ROS 2 노드는 `use_limit_sw:=0`.
3. 듀얼에서 **모터2가 안 도는 경우**: `PID_PNT_VEL_CMD(207)`의 2번째 word는 모터2를 구동하지
   못한다. 모터2는 `PID_VEL_CMD2(131)`로 준다(드라이버 `set_velocities`가 이미 그렇게 한다).
4. PNT50은 명령 후 **~1초 지연** 후 회전한다 — 너무 빨리 정지 판단하지 않는다.
5. `get_status()`에 alarm bit가 있으면 `reset_alarm()`.
6. 읽기값이 이상하면(레지스터 readback이 흔들림) `PID_VERSION` 교차확인으로 통신 정렬을 점검한다
   (일부 어댑터/구형 컨트롤러에서 세션 desync 관찰됨).

## 7. ROS 2 노드 정지/종료 문제

노드는 **Ctrl-C(SIGINT) 또는 `kill <pid>`(SIGTERM)** 를 받으면 `stop` + `torque_off`를 보낸 뒤
스스로 종료한다(직렬이 막혀도 `write_timeout`으로 종료가 hang되지 않고, 종료 중 publish 경합도
무시하도록 처리됨). 그런데도 "정지했는데 프로세스가 살아 있어 kill해야" 하는 경우의 원인과 대처:

1. **`ros2 run`은 2개 프로세스다** — 런처(`ros2 run …`)와 실제 노드(`…/lib/mdrobot_ros2_driver/
   motor_driver_node`). 노드를 **백그라운드(`&`)로 띄우고 런처만** 죽이면 노드가 orphan으로 남는다.
   - 대처: 가능하면 **`ros2 launch`**(노드를 관리하므로 launch에 Ctrl-C 한 번이면 깔끔)나 포그라운드
     실행 후 Ctrl-C를 쓴다. 백그라운드면 **노드 실행 파일 PID를 직접** 종료한다.
2. **엉뚱한 PID/패턴으로 kill** — `pkill -f motor_driver_node`는 같은 문자열을 가진 다른 셸 명령까지
   매칭할 수 있다. 노드 실행 파일 경로로 좁혀 매칭한다.

권장 종료 방법:

```bash
# 포그라운드: Ctrl-C

# 백그라운드 노드만 정상 종료(SIGINT → stop+torque_off 후 종료):
pkill -INT -f 'lib/mdrobot_ros2_driver/motor_driver_node'
# 또는 PID로:
pgrep -f 'lib/mdrobot_ros2_driver/motor_driver_node' | xargs -r kill -INT

# 그래도 안 죽으면 최후 수단(강제 종료 — stop/torque_off가 안 돌 수 있음):
pkill -9 -f 'lib/mdrobot_ros2_driver/motor_driver_node'
```

> SIGKILL(`-9`)로 강제 종료하면 노드의 안전 정지(stop/torque_off)가 실행되지 않을 수 있다. 그 경우
> 모터를 raw로 정지시키거나(`examples/quickstart.py`로 재연결해 `stop`), 전원/비상정지로 차단한다.

## 8. 한계 / 미확정

- **JointState 물리 단위(rad)**: `counts_per_rev`(채널별) 파라미터를 주면 position=rad, velocity=rad/s로
  발행한다(미설정 시 count/rpm + 경고). 값은 모터마다 다르므로(홀≈3×극수, 엔코더 4×PPR) 실측 권장
  (`examples/calibrate_counts_per_rev.py`). 감속비는 포함하지 않는다(모터축 기준 — 바퀴/odom은
  상위 로봇 패키지 몫).
- **엔코더 피드백**: 현재 검증 MD400은 통신으로 엔코더 피드백 활성화가 안 된다(ENC_PPR 설정 거부,
  hall 24/rev 유지). 엔코더의 `counts_per_rev`(4×PPR) 실측은 신형 MD400에서 다룬다(개발 Phase 11 엔코더 부분).
- 전류(current) 부호/단위는 부하 인가 시 재확인 대상.
