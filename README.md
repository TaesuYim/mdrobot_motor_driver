# mdrobot_motor_driver

ROS 2 driver and Python library for **MDROBOT MD-series BLDC/DC motor controllers**, controlled over **RS485 / Modbus RTU**.

The project is split into two packages:

| Package | What it is |
|---|---|
| [`mdrobot`](src/mdrobot) | Pure-Python communication library — framing, CRC, Modbus RTU protocol, registers, status, unit conversion — with **single-channel** and **dual-channel** motor driver classes. Usable on its own (plain Python / `pip`). |
| [`mdrobot_ros2_driver`](src/mdrobot_ros2_driver) | A generic **ROS 2 node** that wraps the library and exposes per-motor velocity/position commands and motor state. |

- **Single-channel** controllers (one motor) → `SingleMotorDriver`
- **Dual-channel** controllers (two motors) → `DualMotorDriver`

This is a *generic* motor driver: it does **not** include robot kinematics (differential drive, odometry, …). It exposes per-motor commands and state only; kinematics belong in a higher-level robot package that consumes this driver.

> **C++ is planned.** This release ships the Python library and the Python ROS 2 node. The layout (a `src/` colcon workspace with per-package naming) is set up so C++ packages (e.g. `mdrobot_cpp`) can be added later without restructuring.

## Repository layout

```text
mdrobot_motor_driver/        # this repo == a colcon workspace
└── src/
    ├── mdrobot/             # Python communication library (ament_python)
    └── mdrobot_ros2_driver/ # ROS 2 node (ament_python), depends on mdrobot
docs/manual/                 # detailed user manual
examples/                    # minimal standalone examples
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

# send a velocity command (dual: [rpm1, rpm2]; single: [rpm])
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40, 40]}"
# stop
ros2 service call /mdrobot_motor_driver/stop std_srvs/srv/Trigger
```

### Python library

```python
from mdrobot import SingleMotorDriver, DualMotorDriver

# single-channel
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    print(d.get_version(), d.get_voltage(), "V")
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

## Documentation

Full usage, parameters, safety and troubleshooting are in the manual:

- **[ROS 2 usage](docs/manual/ros2.md)** — build, launch, parameters, topics/services, `joint_states` units, shutdown, troubleshooting
- **[Python library usage](docs/manual/python.md)** — connect, read, drive, position control, API reference, raw access

Minimal runnable examples are in [`examples/`](examples/).

## Safety

- Always call `enable()` before driving, **start at low speed**, and keep an emergency stop / power cut within reach.
- The ROS 2 node auto-stops if no new velocity command arrives within `command_timeout` (default 0.5 s), and sends stop + torque-off on shutdown.
- If a motor won't move, check in order: `enable()` → `START/STOP` arm → `use_limit_sw` (some controllers need `0` for serial drive). See the manual.

## License

[Apache License 2.0](LICENSE).
