# mdrobot_motor_driver — User Manual

Guide to connecting, reading, and driving MDROBOT MD-series motor controllers —
from the plain Python or C++ library, the ROS 2 node, or `ros2_control`.

| Page | Use it for |
|---|---|
| **[Python library](python.md)** | `mdrobot` — connect, read, drive, position control, slow ramps, raw registers, unit conversion, **full API reference (tables)**, error handling. |
| **[C++ library](cpp.md)** | `mdrobot_cpp` — same API in C++ (`*Connection::open` factory, object lifetime, **API reference tables**, error handling). |
| **[ROS 2 node](ros2.md)** | `mdrobot_ros2_driver` — build, launch, parameters, topics/services, `joint_states` units, shutdown, troubleshooting. |
| **[ros2_control (C++)](ros2_control.md)** | `mdrobot_ros2_control` — the `SystemInterface` plugin: URDF parameters, state/command interfaces, units, controllers, and **twin mode** (two single-channel controllers on one bus). |
| **[Register reference](registers.md)** | Full table of register numbers, command codes and status-1 bits (derived from `registers.py` / `status.py`) for raw access. |

> Both single-channel (one motor) and dual-channel (two motors) controllers are
> supported. The driver is **generic** — it exposes per-motor commands and state
> and contains no robot kinematics; differential drive, odometry and limits
> belong in the robot layer above it.

## Hardware setup

If you are **not using an encoder**, send `ENC_PPR (156) = 0` once to set the encoder
PPR to 0. (Recent firmware ships in encoder mode; older firmware such as v8.1 needs
nothing here.) **Until you do, the first command makes the motor lurch ~0.6 s and then
alarm** — keep clear on the first power-up.

For the full ordered first-drive sequence (comms check → `ENC_PPR` → `USE_LIMIT_SW` →
`enable()` → low rpm + dwell + a stop in reach), see the
[Python manual first-drive checklist](python.md#quick-start).

**Stop input:** for serial-only control, set `USE_LIMIT_SW (17) = 0`. To add a hardware
stop switch on the CTRL connector, set `USE_LIMIT_SW = 1` and wire it to **pin 8
(START/STOP)** — opening it stops the motor (pin 7 RUN/BRAKE is overridden by the
continuous velocity command, so it won't stop a continuously-driven motor).

**Two controllers on one bus (twin):** to drive a skid-steer base from two
single-channel controllers (e.g. two MD400) over one RS485 bus, give each a
distinct Modbus slave id first — with only that unit on the bus, write `PID_ID (133)`
with the wire word `(new_id << 8) | 0xAA` (high byte = new id, low byte = the `0xAA`
write-check; e.g. id 2 → `0x02AA`), power-cycle, then use `device_type=twin`. Full
steps: [ros2_control → Twin mode](ros2_control.md#twin-mode--two-single-channel-controllers-on-one-bus).
