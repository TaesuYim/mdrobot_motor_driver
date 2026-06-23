# ROS 2 Usage (`mdrobot_ros2_driver`)

A generic ROS 2 node wrapping the `mdrobot` library. It supports single- and
dual-channel controllers via the `device_type` parameter and exposes per-motor
velocity/position commands and motor state. No robot kinematics.

## Build

This repository is a colcon workspace (packages under `src/`).

```bash
git clone https://github.com/TaesuYim/mdrobot_motor_driver.git
cd mdrobot_motor_driver
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Configuration & run

All options are set in a parameter YAML file, **not** on the command line. Each
launch file loads its defaults from `config/`:
- `config/single.yaml` (single-channel) · `config/dual.yaml` (dual-channel)

Edit that file (port, counts_per_rev, use_limit_sw, ...) and launch — no extra
options needed:

```bash
ros2 launch mdrobot_ros2_driver single.launch.py
ros2 launch mdrobot_ros2_driver dual.launch.py
```

Use your own parameter file, or set a namespace:

```bash
ros2 launch mdrobot_ros2_driver single.launch.py config:=/path/to/my.yaml namespace:=robot1
```

To run the node directly (no launch file), pass the same YAML with `--params-file`:

```bash
ros2 run mdrobot_ros2_driver motor_driver_node --ros-args --params-file config/single.yaml
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device |
| `baudrate` | `19200` | serial baud rate |
| `motor_id` | `1` | Modbus slave ID |
| `device_type` | `single` | `single` or `dual` |
| `command_timeout` | `0.5` | seconds; auto-stop if no new velocity command arrives |
| `publish_rate` | `20.0` | Hz; `joint_states` publish rate |
| `diag_rate` | `2.0` | Hz; diagnostics publish rate |
| `position_max_rpm` | `100` | speed cap for position moves |
| `joint_names` | `[]` | names for `joint_states` (auto-generated if empty) |
| `auto_enable` | `true` | call `enable()` on startup |
| `counts_per_rev` | `[0.0]` | per-channel; enables SI `joint_states` (see below) |
| `use_limit_sw` | `-1` | `-1` = leave as-is, `0`/`1` = force; some controllers need `0` for serial drive |

## Topics & services

| Kind | Name | Type |
|---|---|---|
| sub | `~/cmd_velocity` | `std_msgs/Float64MultiArray` — `[rpm]` or `[rpm1, rpm2]` |
| sub | `~/cmd_position` | `std_msgs/Float64MultiArray` — `[count]` or `[count1, count2]` |
| pub | `~/joint_states` | `sensor_msgs/JointState` |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` — voltage / status / alarm |
| srv | `~/enable` `~/disable` `~/stop` `~/brake` `~/torque_off` `~/reset_alarm` `~/reset_position` | `std_srvs/Trigger` |

```bash
# velocity command — the array length MUST match the channel count
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40]}"      # single
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40, 40]}"  # dual
ros2 service call /mdrobot_motor_driver/stop std_srvs/srv/Trigger
```

> A wrong-length array is ignored with a warning (single needs `[rpm]`, dual needs
> `[rpm1, rpm2]`).

Topic/service names sit under the node name (and namespace if you set one);
prefix accordingly.

## `joint_states` units

Without `counts_per_rev`, `~/joint_states` is published in raw units (position in
counts, velocity in rpm) and the node logs a warning. Set `counts_per_rev` (per
channel) to publish SI units (position in rad, velocity in rad/s):

```bash
ros2 run mdrobot_ros2_driver motor_driver_node --ros-args \
  -p device_type:=dual -p counts_per_rev:='[24.0, 24.0]'
```

`counts_per_rev` is counts per **one revolution of the motor shaft** — the
controller reports both position (count) and speed (rpm) at the motor. The value
differs per motor (hall ≈ 3 × pole count; encoder = 4 × PPR), so **measure it**:
turn the motor shaft a known N turns and compute Δcount / N
(see [`examples/calibrate_counts_per_rev.py`](../../examples/calibrate_counts_per_rev.py)).

Gear ratio is **not** applied here: `counts_per_rev` scales the position state only,
while velocity is `rpm → rad/s` regardless. Measuring at a geared output shaft would
make position the wheel angle but leave velocity at the motor rate, so the two would
disagree by the gear ratio. Keep `counts_per_rev` at the motor and account for the
gearbox in the robot layer above (e.g. set `diff_drive_controller`'s `wheel_radius`
to the effective radius = wheel radius ÷ gear ratio).

## Shutting down the node

On **Ctrl-C (SIGINT)** or `kill <pid>` (SIGTERM) the node sends stop +
torque-off and then exits. Notes:

- `ros2 run` is **two processes** — the launcher and the node executable. If you
  background the node (`&`) and kill only the launcher, the node is orphaned.
  Prefer `ros2 launch` (a single Ctrl-C is clean) or foreground + Ctrl-C.
- To stop a backgrounded node gracefully:
  ```bash
  pkill -INT -f 'lib/mdrobot_ros2_driver/motor_driver_node'
  ```
- Last resort (skips the safe stop): `pkill -9 -f 'lib/mdrobot_ros2_driver/motor_driver_node'`.
  After a SIGKILL the motor may not have been stopped — cut power / e-stop, or
  reconnect and `stop` it.

## Troubleshooting — motor won't move

1. Is the node enabled? (`auto_enable` true, or call `~/enable`.) `enable()` sets
   `UI_COM=1` and arms `START/STOP`.
2. **Single-channel**: some controllers need `USE_LIMIT_SW=0` for serial drive —
   set `use_limit_sw: 0` in the config file.
3. **Dual-channel, motor 2 not turning**: handled by the driver (motor 2 uses its
   own command register).
4. Some dual-channel controllers turn **~1 s after** the command.
5. Alarm bit set? Call `~/reset_alarm`.
6. Unstable readbacks may indicate a serial session desync (some adapters); the
   node cross-checks the version register on connect.

