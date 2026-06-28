# mdrobot_motor_driver

ROS 2 driver and Python library for **MDROBOT MD-series BLDC/DC motor controllers**, controlled over **RS485 / Modbus RTU**.

The project is a colcon workspace of complementary packages — use only what you need:

| Package | What it is |
|---|---|
| [`mdrobot`](src/mdrobot) | Pure-Python communication library — framing, CRC, Modbus RTU protocol, registers, status, unit conversion — with **single-channel** and **dual-channel** motor driver classes. Usable on its own (plain Python / `pip`). |
| [`mdrobot_ros2_driver`](src/mdrobot_ros2_driver) | A generic **ROS 2 node** (Python) that wraps the library and exposes per-motor velocity/position commands and motor state. |
| [`mdrobot_cpp`](src/mdrobot_cpp) | **C++ communication library** — the same layers as `mdrobot` (POSIX `termios` transport, CRC, Modbus RTU, registers, status, units, single/dual drivers). `ament_cmake`. |
| [`mdrobot_ros2_control`](src/mdrobot_ros2_control) | A C++ [`ros2_control`](https://control.ros.org) **`SystemInterface` plugin** wrapping `mdrobot_cpp`. One plugin for every shape via `device_type` (single → 1 joint; dual → 2 joints on one two-channel controller; **twin → 2 joints on two single-channel controllers** at distinct slave ids on one bus, for a skid-steer base); exports position/velocity/effort state and velocity/position command interfaces. |
| [`mdrobot_diffbot_example`](src/mdrobot_diffbot_example) | An **example** differential-drive robot (URDF + `diff_drive_controller` + RViz) built on `mdrobot_ros2_control`. Runs with mock hardware (no device) or a real dual controller. |

- **Single-channel** controllers (one motor) → `SingleMotorDriver`
- **Dual-channel** controllers (two motors) → `DualMotorDriver`

This is a *generic* motor driver: it does **not** include robot kinematics (differential drive, odometry, …). It exposes per-motor commands and state only; kinematics belong in a higher-level robot package that consumes this driver.

> **Python and C++.** The Python library/node and the C++ library/`ros2_control` plugin live side by side in one colcon workspace. Build only what you need with `colcon build --packages-select <pkg>`.

## Repository layout

```text
mdrobot_motor_driver/            # this repo == a colcon workspace
└── src/
    ├── mdrobot/                 # Python communication library (ament_python)
    ├── mdrobot_ros2_driver/     # Python ROS 2 node (ament_python), depends on mdrobot
    ├── mdrobot_cpp/             # C++ communication library (ament_cmake)
    ├── mdrobot_ros2_control/    # C++ ros2_control SystemInterface (ament_cmake), depends on mdrobot_cpp
    └── mdrobot_diffbot_example/ # example diff-drive robot (URDF + diff_drive + RViz)
docs/manual/                     # detailed user manual
examples/                        # minimal standalone examples
```

## Requirements

- Python ≥ 3.10
- [`pyserial`](https://pypi.org/project/pyserial/) ≥ 3.5 (for real serial I/O)
- ROS 2 (tested on **Jazzy**) — for the ROS 2 node
- An RS485 (USB-serial) adapter. Default link settings: **19200 8N1**, controller ID **1**

## Install & build (ROS 2)

This repository **is** a colcon workspace — the packages live under `src/`.

```bash
git clone https://github.com/TaesuYim/mdrobot_motor_driver.git
cd mdrobot_motor_driver
rosdep install --from-paths src --ignore-src -r -y   # pulls rclpy, pyserial, ...
colcon build
source install/setup.bash
```

## Install (Python library only, no ROS 2)

```bash
pip install -e 'src/mdrobot[serial]'    # [serial] pulls in pyserial
```

## Quick start

### ROS 2 node

```bash
# set options in config/single.yaml or config/dual.yaml (port, counts_per_rev, ...),
# then launch — no command-line options needed:
ros2 launch mdrobot_ros2_driver single.launch.py   # single-channel
ros2 launch mdrobot_ros2_driver dual.launch.py     # dual-channel

# send a velocity command — single: [rpm], dual: [rpm1, rpm2] (length must match the channel count)
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40]}"      # single
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40, 40]}"  # dual
# stop
ros2 service call /mdrobot_motor_driver/stop std_srvs/srv/Trigger
```

### Python library

```python
from mdrobot import SingleMotorDriver, DualMotorDriver

# read-only first — confirm comms without moving the motor
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    print(d.get_version(), d.get_voltage(), "V", d.get_status().active)

# single-channel drive (motor turns)
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()             # required before motion (UI_COM=1 + START/STOP arm)
    d.set_velocity(40)     # signed rpm; + = CCW
    d.stop(); d.torque_off()

# dual-channel
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocities(40, 40)
    d.stop(); d.torque_off_both()
```

Low-level register/command access is always available via `d.client` for anything the high-level API doesn't cover.

### ros2_control (C++)

```bash
colcon build --packages-select mdrobot_cpp mdrobot_ros2_control
source install/setup.bash

# single (MD400) — joint_state_broadcaster + a velocity forward command controller
ros2 launch mdrobot_ros2_control bringup.launch.py \
    device_type:=single port:=/dev/ttyUSB0 counts_per_rev:=24

# dual (PNT50/MD400T) laid out as a diff-drive base
ros2 launch mdrobot_ros2_control bringup.launch.py \
    device_type:=dual port:=/dev/ttyUSB0 counts_per_rev:=12

# twin: two single-channel controllers (e.g. two MD400) on one bus at slave ids 1 & 2
ros2 launch mdrobot_ros2_control bringup.launch.py \
    device_type:=twin port:=/dev/ttyUSB0 motor_id_1:=1 motor_id_2:=2 reverse_2:=true
```

The hardware plugin (`mdrobot_ros2_control/MdrobotSystemHardware`) is declared in the
robot's URDF `<ros2_control>` block; set `device_type`, `port`, per-joint `counts_per_rev`
(positive → SI rad/rad·s state & commands, otherwise raw count/rpm), and the gating options
there. **Twin** mode needs the two controllers re-IDed to distinct Modbus slave ids first
(it is code-complete and unit-tested, but simultaneous diff-drive is not yet hardware-verified).
See the manual for the full parameter list, twin mode, and a runnable diff-drive example
([`mdrobot_diffbot_example`](src/mdrobot_diffbot_example)).

## Documentation

Full usage, parameters, safety and troubleshooting are in the manual:

- **[ROS 2 usage](docs/manual/ros2.md)** — build, launch, parameters, topics/services, `joint_states` units, shutdown, troubleshooting
- **[Python library usage](docs/manual/python.md)** — connect, read, drive, position control, API reference tables, error handling, raw access
- **[C++ library usage](docs/manual/cpp.md)** — `mdrobot_cpp` API reference tables, `open()` factory, object lifetime, error handling
- **[ros2_control (C++)](docs/manual/ros2_control.md)** — `mdrobot_cpp` library + the `SystemInterface` plugin, URDF parameters, controllers, twin mode
- **[Diff-drive example](src/mdrobot_diffbot_example/README.md)** — runnable differential-drive robot (URDF + `diff_drive_controller` + RViz), mock or real hardware

Minimal runnable examples are in [`examples/`](examples/).

## Tested drivers & firmware

Verified on real hardware (raw `PID_VERSION` DL byte is authoritative; the vX.Y is
the doc convention `DL/10 . DL%10`):

| Model | Type | Firmware (raw DL / approx.) | Verified |
|---|---|---|---|
| MD400 | single | DL=81 / v8.1 | identify, read, velocity (both directions), position (absolute/relative), ROS 2 node |
| MD400 | single | DL=86 / v8.6 | ships in encoder mode → set `ENC_PPR (156) = 0` for hall closed-loop drive (counts/rev = 30); velocity, position, ROS 2 node; `PID_ID (133)` slave-id change |
| PNT50 | dual | DL=45 / v4.5 | identify, read, velocity (both motors), position (simultaneous), ROS 2 node |
| MD400T | dual | DL=72 / v7.2 | identify, read, velocity (both motors), position (simultaneous), ROS 2 node |

> **Twin mode** (two single-channel controllers on one bus) is **code-complete and
> unit-tested**, and the `PID_ID` slave-id change is confirmed on MD400 v8.6, but two
> controllers driving a base together have **not** yet been hardware-verified — treat
> it as experimental.

## License

[Apache License 2.0](LICENSE).
