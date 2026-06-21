# mdrobot_ros2_driver

Generic ROS 2 driver node for MDROBOT MD-series BLDC/DC motor controllers
(RS485 / Modbus RTU). Single-channel and dual-channel controllers are handled
through the `device_type` parameter. **No robot kinematics** — it exposes
per-motor velocity/position commands and motor state only; `/cmd_vel` -> wheel
conversion and odometry belong to a higher-level robot package.

It depends on the in-workspace `mdrobot` Python library, which colcon builds
together with this package (declared as `<exec_depend>mdrobot</exec_depend>`).

## Build & run

```bash
# from the workspace root (this repository)
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Set your options in `config/single.yaml` or `config/dual.yaml` (port,
counts_per_rev, use_limit_sw, ...), then launch — no command-line options needed:

```bash
ros2 launch mdrobot_ros2_driver single.launch.py   # single-channel
ros2 launch mdrobot_ros2_driver dual.launch.py     # dual-channel
# use your own parameter file / namespace:
ros2 launch mdrobot_ros2_driver single.launch.py config:=/path/to/my.yaml namespace:=robot1
```

## Interface (summary)

Parameters: `port`, `baudrate` (19200), `motor_id` (1), `device_type`
(single|dual), `command_timeout` (0.5 s, 0 disables), `publish_rate` (20 Hz),
`diag_rate` (2 Hz), `position_max_rpm` (100), `joint_names`, `auto_enable` (true),
`counts_per_rev` (per channel; enables SI `joint_states`),
`use_limit_sw` (-1 leave / 0 disable / 1 enable).

| Kind | Name | Type |
|---|---|---|
| sub | `~/cmd_velocity` | `std_msgs/Float64MultiArray` — `[rpm]` / `[rpm1, rpm2]` |
| sub | `~/cmd_position` | `std_msgs/Float64MultiArray` — `[count]` / `[count1, count2]` (speed = `position_max_rpm`) |
| pub | `~/joint_states` | `sensor_msgs/JointState` (rad / rad·s if `counts_per_rev` set, else count / rpm) |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` — voltage / status / alarm |
| srv | `~/enable` `~/disable` `~/stop` `~/brake` `~/torque_off` `~/reset_alarm` `~/reset_position` | `std_srvs/Trigger` |

Sign convention (verified on hardware): `+` = increasing position (CCW).

**Full documentation** (parameters, `joint_states` units, shutdown,
troubleshooting): see [../../docs/manual/ros2.md](../../docs/manual/ros2.md).

## Safety

- With `command_timeout` > 0 the node auto-stops if no new `~/cmd_velocity`
  arrives within that time.
- On shutdown the node sends `stop` + `torque_off`.
- Callbacks run on a single-threaded executor, so serial-port access never overlaps.
