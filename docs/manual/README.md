# mdrobot_motor_driver — User Manual

Detailed guide to connecting, reading, and driving MDROBOT MD-series motor
controllers with this driver — via either ROS 2 or the plain Python library.

- **[ROS 2 usage](ros2.md)** — building the workspace, launch files, node
  parameters, topics & services, `joint_states` units, shutting the node down,
  and troubleshooting.
- **[Python library usage](python.md)** — installing the library, connecting,
  reading state, driving single- and dual-channel controllers, position control,
  the high-level API reference, and low-level register access.

> Both single-channel (one motor) and dual-channel (two motors) controllers are
> supported. The driver is generic — it exposes per-motor commands and state and
> contains no robot kinematics.

## Safety first

- Test with the motor **unloaded** first, start at **low speed**, and keep an
  emergency stop / power cut within reach.
- `+` = CCW = increasing position; `-` = CW = decreasing position. Confirm the
  real direction once in your installation.
- Always `enable()` before sending motion (the ROS 2 node does this on startup
  by default via `auto_enable`).
