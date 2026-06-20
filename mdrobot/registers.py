"""PID / CMD named constant 맵.

CLAUDE.md §2.4: 모든 PID/CMD/상태 비트는 named constant로 정의하고 고수준 코드에
숫자 literal을 쓰지 않는다. 이 모듈은 doc 04(`04-command-pid-map.md`)의 권장 상수명을
그대로 따른다. 원문 오탈자/중복 이름은 doc 04를 기준으로 정리했다.

현재는 Phase 1~8에서 실제로 쓰는 우선순위 PID/CMD(doc 04 §7)를 정의한다.
새 PID가 필요하면 doc 04 표에서 값을 확인해 여기에 추가하고, raw 코드에 숫자를
직접 쓰지 않는다.
"""

from __future__ import annotations

# --- 단일 byte 명령/설정 PID (0-127) -------------------------------------------------
PID_VERSION = 1            # R  firmware/protocol version (DL=13 -> v1.3). ping 후보
PID_TQ_OFF = 5             # W  motor free / natural stop
PID_BRAKE = 6              # W  electric brake
PID_COMMAND = 10           # W  CMD command gateway (아래 CMD_* 사용)
PID_ALARM_RESET = 12       # W  reset alarm
PID_POSI_RESET = 13        # W  reset position to zero
PID_INV_SIGN_CMD = 16      # R/W reference command sign inverse
PID_USE_LIMIT_SW = 17      # R/W CTRL limit switch function (0 cancel, 1 use). 기본 1
PID_INPUT_TYPE = 25        # R/W user input type
PID_USE_LIMIT_SW2 = 29     # R/W MOT2 limit switch function (PNT/MDTx). PID17과 동일 의미
PID_CTRL_STATUS = 34       # R  status bit map (doc 05)
PID_DI = 48                # R  digital input bits (doc 04). 하드웨어 진단 필수
PID_IN_POSITION_OK = 49    # R  position control done (0/1)
PID_UI_COM = 78            # R/C serial communication control (0 CTRL I/O, 1 serial 단독)
PID_START_STOP = 100       # C  start/stop (0 stop, 1 CCW, 2 CW). run-latch arm.
                           #    실물 확인(2026-06-19): serial 속도 구동 enable에 필요(=1).

# --- word 명령/데이터 PID (101-190) ---------------------------------------------------
PID_VEL_CMD = 130          # C  velocity command, signed rpm. core set_velocity
PID_VEL_CMD2 = 131         # C  MOT2 velocity command (MDTx)
PID_ID = 133               # W  controller ID setting (write check 0xAA)
PID_OPEN_VEL_CMD = 134     # C  open-loop output command (-1023..1023). 위험
PID_BAUDRATE = 135         # W  RS485 baudrate setting (write check 필요)
PID_INT_RPM_DATA = 138     # R  motor speed, signed rpm. get_speed
PID_TQ_DATA = 139          # R  current, 0.1A 단위. get_current
PID_TQ_CMD = 140           # C  torque/current command (-1023..1023)
PID_VOLT_IN = 143          # R  supply voltage, 0.1V 단위. get_voltage
PID_RETURN_TYPE = 149      # R/W return type after command
PID_TAR_VEL = 155          # R/W fixed target speed, rpm (replaces internal volume).
                           #    실물(2026-06-19): START_STOP 기반 구동의 속도원은 아님 → PID_COM_TAR_SPEED 사용.
PID_REF_RPM = 166          # R  reference velocity, signed rpm
PID_PNT_TQ_OFF = 174       # C  두 모터 free/tq-off (DL motor1, DH motor2)
PID_PNT_BRAKE = 175        # C  두 모터 electric brake (DL motor1, DH motor2)
PID_TAR_POSI_VEL = 176     # R/W max speed in position control, rpm
PID_COM_TAR_SPEED = 180    # C/R target speed used by PID_START_STOP, rpm.
                           #    실물 확인: COM_TAR_SPEED + START_STOP(1/2) 경로로 구동 가능.

# --- N-byte 데이터/명령 PID (193-253) -------------------------------------------------
PID_MONITOR = 196          # R  single monitor (12 bytes: speed/current/output/position/status)
PID_POSI_DATA = 197        # R  motor position (4 bytes long). get_position
PID_POSI_SET1 = 198        # C  set motor1 position (4 bytes long)
PID_GAIN = 203             # C/R position P, speed P, speed I gain (6 bytes)
PID_PNT_POSI_VEL_CMD = 206 # C  dual position+max speed (12 bytes). PNT/MDTx 핵심
PID_PNT_VEL_CMD = 207      # C  dual velocity command (4 bytes: speed1, speed2). PNT/MDTx 핵심
PID_PNT_OPEN_VEL_CMD = 208 # C  dual open-loop command (4 bytes)
PID_PNT_TQ_CMD = 209       # C  dual torque/current command (4 bytes)
PID_PNT_MAIN_DATA = 210    # R  dual main data (18 bytes)
PID_PNT_INC_POSI_CMD = 215 # C  dual incremental position (8 bytes)
PID_PNT_MONITOR = 216      # R  dual monitor (14 bytes)
PID_POSI_SET = 217         # C  set motor position (4 bytes long). 원문 #defien typo
PID_POSI_SET2 = 218        # C  set motor2 position (4 bytes long, MDTx)
PID_POSI_VEL_CMD = 219     # C  position control with max speed (6 bytes). core move_to
PID_INC_POSI_VEL_CMD = 220 # C  incremental position with max speed (6 bytes). core move_by
PID_MAX_RPM = 221          # R/W max speed (2 bytes word)
PID_INC_POSI_VEL_CMD2 = 224  # C MOT2 incremental position+speed (MDTx)
PID_POSI_VEL_CMD2 = 236    # C  MOT2 position control with max speed (MDT only)
PID_PNT_INC_POSI_VEL_CMD = 242  # C dual incremental position + max speed (12 bytes)
PID_POSI_CMD = 243         # W  target position (4 bytes long)
PID_INC_POSI_CMD = 244     # W  incremental target position (4 bytes long)
PID_PNT_POSI_CMD = 246     # C  dual absolute position (8 bytes)
PID_POSI_CMD2 = 247        # C  MOT2 target position (MDT only)

# --- PID_COMMAND(10) 게이트웨이로 보내는 CMD 번호 (doc 04 §3) -------------------------
CMD_TQ_OFF = 2             # motor free state
CMD_BRAKE = 4              # electric brake
CMD_ALARM_RESET = 8        # reset alarm
CMD_POSI_RESET = 10        # position reset to zero
CMD_TAR_VEL_OFF = 20       # erase target velocity set by PID_TAR_VEL/COM_TAR_SPEED
CMD_EMER_ON = 67           # stop motor + electric brake + UVW short. emergency_stop 후보
CMD_EMER_OFF = 68          # free state / Tq-off. 위험 주석 필요
CMD_BRAKE1 = 69            # electric brake on motor1 (dual)
CMD_BRAKE2 = 70            # electric brake on motor2 (dual)
CMD_RESET_SYSTEM = 79      # controller reboot. 위험. raw-only 또는 명시 API
