# ros2_control (C++) — `mdrobot_cpp` + `mdrobot_ros2_control`

Two C++ packages let you drive MDROBOT controllers from the
[`ros2_control`](https://control.ros.org) stack:

| Package | Role |
|---|---|
| `mdrobot_cpp` | C++ communication library — a 1:1 port of the Python `mdrobot` library (POSIX `termios` transport, CRC, Modbus RTU protocol, registers, status decoding, unit conversion, `SingleMotorDriver` / `DualMotorDriver`). `ament_cmake`, no ROS dependency. |
| `mdrobot_ros2_control` | A `hardware_interface::SystemInterface` plugin (`pluginlib`) that wraps `mdrobot_cpp`. One plugin handles both single- and dual-channel controllers. |

Both are independent of the Python packages — build only these if C++ is all you need.

## Build

```bash
colcon build --packages-select mdrobot_cpp mdrobot_ros2_control
source install/setup.bash
```

`mdrobot_cpp` ships gtest unit tests (golden Modbus vectors, decoders, unit
conversion). Run them with `colcon test --packages-select mdrobot_cpp`.

## `mdrobot_cpp` as a plain C++ library

```cpp
#include "mdrobot_cpp/device.hpp"

auto s = mdrobot::SingleMotorConnection::open("/dev/ttyUSB0");  // owns the port
s->enable();               // UI_COM=1 + START/STOP arm
s->set_velocity(40);       // signed rpm, + = CCW
auto m = s->read_monitor();    // m.speed_rpm, m.position, m.current_a
s->stop();
s->torque_off();
```

The API mirrors the Python library (`DualMotorDriver` has `set_velocities`,
`move_to_both`, per-channel getters, …). Link with
`ament_target_dependencies(<tgt> mdrobot_cpp)`. **Full C++ API reference, error
handling and object lifetime: [cpp.md](cpp.md).**

## The `SystemInterface` plugin

The plugin class is `mdrobot_ros2_control/MdrobotSystemHardware`. It is declared
in your robot's URDF `<ros2_control>` block. One device shape per component:
`device_type=single` → 1 joint, `device_type=dual` → 2 joints.

### Interfaces (per joint)

- **State:** `position`, `velocity`, `effort` (effort = raw motor current in A — a
  proxy, not calibrated torque). For **dual**, `read()` uses `PNT_MAIN_DATA` so the
  current is real; under no load it is ~0 A.
- **Command:** `velocity` and/or `position` (declare whichever your controller needs).

### Units

A joint with a **positive `counts_per_rev`** exports SI (`position` = rad,
`velocity` = rad/s) and accepts SI commands. Without it the joint stays raw
(`position` = count, `velocity` = rpm). The value is per motor — hall feedback is
`3 × pole count` (e.g. 24 for 8-pole, 30 for 10-pole, 12 for 4-pole). Measure it;
never assume.

### Hardware parameters (`<hardware>`)

| param | default | meaning |
|---|---|---|
| `device_type` | (from joint count) | `single` or `dual` |
| `port` | `/dev/ttyUSB0` | serial port |
| `baudrate` | `19200` | |
| `motor_id` | `1` | Modbus slave id |
| `use_limit_sw` | `-1` | `-1` leave as-is, `0` disable, `1` enable (some controllers need `0` for serial drive) |
| `auto_enable` | `true` | call `enable()` on activation |
| `position_max_rpm` | `100` | speed cap for position commands |
| `timeout` | `0.3` | serial read timeout (s) |
| `max_comm_errors` | `5` | consecutive read/write failures tolerated before the component goes to ERROR (rides out transient serial hiccups; the loop keeps the last state meanwhile) |

`counts_per_rev` is a **per-joint** `<param>` — counts per **one revolution of the
motor shaft** (the controller measures position and speed at the motor). Gear ratio
is **not** applied here: it scales the position state only, while velocity is
`rpm → rad/s` regardless, so a geared output-shaft value would make position and
velocity disagree by the ratio. With a gearbox, keep `counts_per_rev` at the motor
and set `diff_drive_controller`'s `wheel_radius` to the effective radius
(wheel radius ÷ gear ratio). See the [Python manual](python.md#unit-conversion-mdrobotunits).

### Minimal URDF (single)

```xml
<ros2_control name="mdrobot_single" type="system">
  <hardware>
    <plugin>mdrobot_ros2_control/MdrobotSystemHardware</plugin>
    <param name="device_type">single</param>
    <param name="port">/dev/ttyUSB0</param>
  </hardware>
  <joint name="motor1">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="counts_per_rev">24</param>
  </joint>
</ros2_control>
```

Ready-to-use xacro for single and dual are under
[`src/mdrobot_ros2_control/urdf/`](../../src/mdrobot_ros2_control/urdf/).

## Controllers & bringup

`bringup.launch.py` starts `robot_state_publisher`, the `controller_manager`, and
spawns the controllers from `config/<device_type>_controllers.yaml`:

- **single** → `joint_state_broadcaster` + `velocity_cont`
  (`forward_command_controller`, commands `/velocity_cont/commands`).
  Needs `ros-jazzy-ros2-controllers`.
- **dual** → `joint_state_broadcaster` + `diff_cont` (`diff_drive_controller`,
  `/diff_cont/cmd_vel`, `geometry_msgs/TwistStamped`).

```bash
ros2 launch mdrobot_ros2_control bringup.launch.py \
    device_type:=dual port:=/dev/ttyUSB0 counts_per_rev:=12
# drive (dual): linear.x in m/s -> wheels
ros2 topic pub /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.1}}}"
```

The launch arg `counts_per_rev:=0` (default) **keeps the URDF's own value** (single
`24`, dual `12` — both positive, i.e. SI); a positive value overrides it. This is
*not* the same as the per-joint `<param name="counts_per_rev">0</param>`, which
selects **raw** (count/rpm) units — to publish raw, set the URDF `<param>` to `0`
rather than the launch arg.

For a complete, runnable robot (proper URDF geometry, RViz, mock/real switch,
odometry + TF) see the **[`mdrobot_diffbot_example`](../../src/mdrobot_diffbot_example/README.md)**
package.

## Notes

- **Update rate:** each read+write cycle is a few 19200-baud round-trips. A dual
  cycle (monitor read + two velocity writes) is ~50 ms, so keep
  `controller_manager` `update_rate` around **15 Hz** for dual; higher rates
  overrun.
- **Lifecycle:** `on_configure` opens the port, `on_activate` enables, `on_deactivate`
  does stop + torque-off. Mode (velocity vs position) follows whichever command
  interface a controller claims.
- **Safety:** start at low speed, no load, with an emergency stop within reach.
  This is a generic driver — soft limits, odometry and kinematics belong in the
  robot layer above it.
