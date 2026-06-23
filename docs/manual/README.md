# mdrobot_motor_driver — User Manual

Guide to connecting, reading, and driving MDROBOT MD-series motor controllers —
from the plain Python or C++ library, the ROS 2 node, or `ros2_control`.

| Page | Use it for |
|---|---|
| **[Python library](python.md)** | `mdrobot` — connect, read, drive, position control, slow ramps, raw registers, unit conversion, **full API reference (tables)**, error handling. |
| **[C++ library](cpp.md)** | `mdrobot_cpp` — same API in C++ (`*Connection::open` factory, object lifetime, **API reference tables**, error handling). |
| **[ROS 2 node](ros2.md)** | `mdrobot_ros2_driver` — build, launch, parameters, topics/services, `joint_states` units, shutdown, troubleshooting. |
| **[ros2_control (C++)](ros2_control.md)** | `mdrobot_ros2_control` — the `SystemInterface` plugin: URDF parameters, state/command interfaces, units, controllers, diff-drive example. |

A complete runnable robot is the [`mdrobot_diffbot_example`](../../src/mdrobot_diffbot_example/README.md)
package (URDF + `diff_drive_controller` + RViz, mock or real hardware).

> Both single-channel (one motor) and dual-channel (two motors) controllers are
> supported. The driver is **generic** — it exposes per-motor commands and state
> and contains no robot kinematics; differential drive, odometry and limits
> belong in the robot layer above it.

## Safety first

- Test with the motor **unloaded** first, start at **low speed**, and keep an
  emergency stop / power cut within reach.
- `+` = CCW = increasing position; `-` = CW = decreasing position. Confirm the
  real direction once in your installation.
- Always `enable()` before sending motion (the ROS 2 node does this on startup
  by default via `auto_enable`).
