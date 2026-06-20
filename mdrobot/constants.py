"""Modbus 함수 코드와 통신 공통 상수.

PID/CMD 표는 registers.py에 둔다. 여기에는 frame/transport 계층이 쓰는
프로토콜 상수만 모은다. (doc 02, doc 04 §2)
"""

from __future__ import annotations

# Modbus function codes — 기본 구현 대상은 이 3개뿐이다 (CLAUDE.md §2.3, doc 02 §2).
FUNC_READ = 0x03            # read holding registers
FUNC_WRITE_SINGLE = 0x06    # write single register
FUNC_WRITE_MULTIPLE = 0x10  # write multiple registers

# 응답 function에 이 비트가 서면 Modbus exception response다 (doc 02 §9).
EXCEPTION_FLAG = 0x80

# ID / write-check 상수 (doc 04 §2).
ID_ALL = 0xFE            # all/broadcast ID. 초기 구현에서는 쓰지 않는다.
ID_WRITE_CHK = 0xAA      # ID/baudrate 등 일부 설정 write check 값.
ID_DEFAULT_CHK = 0x55    # default setting check 값.
ID_DEVELOPER_CHK = 0x77  # developer check 값. 사용처 불명확.

# 통신 기본값 (doc 02 §10). 실물 결과로 조정한다.
DEFAULT_SLAVE_ID = 1
DEFAULT_BAUDRATE = 19200       # MD400/PNT50 검증 장비 기준 (doc 01 §2: 19200 8N1).
DEFAULT_TIMEOUT = 0.3          # 직렬 read timeout(초). 최대 23 byte 프레임에 충분.
