# ros2_control (C++) — `mdrobot_cpp` + `mdrobot_ros2_control`

Two C++ packages let you drive MDROBOT controllers from the
[`ros2_control`](https://control.ros.org) stack:

| Package | Role |
|---|---|
| `mdrobot_cpp` | C++ communication library — a 1:1 port of the Python `mdrobot` library (POSIX `termios` transport, CRC, Modbus RTU protocol, registers, status decoding, unit conversion, `SingleMotorDriver` / `DualMotorDriver`). `ament_cmake`, no ROS dependency. |
| `mdrobot_ros2_control` | A `hardware_interface::SystemInterface` plugin (`pluginlib`) that wraps `mdrobot_cpp`. One plugin handles single- and dual-channel controllers, plus **twin** (two single-channel controllers on one bus). |

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

- `device_type=single` → 1 joint (one single-channel controller, e.g. MD400).
- `device_type=dual` → 2 joints (**one** two-channel controller, e.g. PNT50 / MD400T).
- `device_type=twin` → 2 joints driven as **two** single-channel controllers on
  one serial bus at distinct Modbus slave ids (e.g. two MD400 for a skid-steer
  base). See [Twin mode](#twin-mode--two-single-channel-controllers-on-one-bus).

### Interfaces (per joint)

- **State:** `position`, `velocity`, `effort` (effort = raw motor current in A — a
  proxy, not calibrated torque). For **dual**, `read()` uses `PNT_MAIN_DATA` so the
  current is real; under no load it is ~0 A. For **twin**, each controller is read
  independently, so current is real per wheel.
- **Command:** `velocity` and/or `position` (declare whichever your controller needs).

### Units

A joint with a **positive `counts_per_rev`** exports SI (`position` = rad,
`velocity` = rad/s) and accepts SI commands. Without it the joint stays raw
(`position` = count, `velocity` = rpm). The value is per motor — hall feedback is
`3 × pole count` (e.g. 24 for 8-pole, 30 for 10-pole, 12 for 4-pole). Measure it;
never assume.

### Hardware parameters

These are the controller's `<hardware>` `<param>`s, but you normally **do not edit
the URDF** — set them in `config/<device_type>_controllers.yaml` under the
**`mdrobot_hardware`** section. `bringup.launch.py` reads that section and injects
it into the URDF for you (`controller_manager` ignores the section). Writing them
directly as URDF `<param>`s still works, e.g. if you embed this plugin in your own
robot description.

| param | default | meaning |
|---|---|---|
| `device_type` | (from joint count) | `single`, `dual`, or `twin`. Blank infers `single`/`dual` from the joint count; **`twin` must be set explicitly** (twin and dual both have 2 joints, so it cannot be inferred). |
| `port` | `/dev/ttyUSB0` | serial port (set in the yaml; or override with the `port:=` launch arg) |
| `baudrate` | `19200` | |
| `motor_id` | `1` | Modbus slave id. For **twin** each controller has its own (`motor_id_L` / `motor_id_R` in the yaml — see below), not one shared `motor_id`. |
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

Ready-to-use xacro for single, dual and twin are under
[`src/mdrobot_ros2_control/urdf/`](../../src/mdrobot_ros2_control/urdf/).

### Twin mode — two single-channel controllers on one bus

> **Status: code-complete and unit-tested; simultaneous diff-drive is not yet
> hardware-verified.** The library (`mdrobot_cpp`) and the slave-id change have
> been confirmed on real hardware, but two controllers driving a base together
> have not. Treat twin as **experimental** until that is verified.

`device_type=twin` drives **two separate single-channel controllers** (e.g. two
MD400) over **one** serial bus, addressed by **distinct Modbus slave ids**, laid
out as a 2-wheel differential base so `diff_drive_controller` can drive it.

This is **not** `device_type=dual`:

| | `dual` | `twin` |
|---|---|---|
| Hardware | **one** controller, two channels | **two** single-channel controllers |
| Bus | one slave id | one bus, **two distinct slave ids** |
| Per wheel | shared device | independent velocity / position, independent state |

The library is unchanged: one `SerialTransport` feeds N `ModbusClient`s with
different slave ids, each wrapping a `SingleMotorDriver`. The control loop is the
single serial owner, so the two controllers are addressed in turn on the one bus.

**Prerequisite — re-ID one controller.** Controllers ship at slave id `1`, so two
on a bus collide. Change one to id `2` *before* wiring them together — with **only
that one controller on the bus**, write `PID_ID (133)` with the wire word
`(new_id << 8) | 0xAA` (high byte = new id, low byte = the `0xAA` write-check,
e.g. id 2 → `0x02AA`), then power-cycle and confirm:

```python
from mdrobot import SingleMotorDriver
with SingleMotorDriver.open("/dev/ttyUSB0") as d:   # only this unit on the bus
    d.client.write_register(133, (2 << 8) | 0xAA)   # set this controller to id 2
# power-cycle, then re-open and check it now answers at id 2
```

`on_init` rejects equal ids. *(Confirmed on MD400 v8.6; older firmware / dual
controllers are untested for this write.)*

**Per-wheel settings.** The two joints are `motor_L` (left wheel, first joint) and
`motor_R` (right wheel, second joint). Set each wheel in `twin_controllers.yaml`
(`mdrobot_hardware` section) — `…_L` for the left controller, `…_R` for the right:

| yaml key | meaning |
|---|---|
| `motor_id_L` / `motor_id_R` | each controller's Modbus slave id — **must differ** (`on_init` rejects equal ids) |
| `reverse_L` / `reverse_R` | `true`/`false`. A skid-steer mounts the two motors mirrored, so one side usually needs `reverse: true` for `+cmd_vel.x` to drive the base forward. Applied symmetrically to commands **and** feedback, so odometry stays consistent. Default `false` on both — drive the base, see which side spins backwards, then set it. |
| `counts_per_rev_L` / `counts_per_rev_R` | per-wheel counts per rev (hall = `3 × pole count`). Left/right may differ if the motors are not matched; keep it **positive** (it is the SI gate — use `reverse` for direction, never a negative `counts_per_rev`). |

**Partial-failure policy.** If one controller stops responding, the driver
commands **zero speed to the other** wheel as well — a mobile base must not keep
one wheel running at its last command and veer. Persistent failure takes the
component to ERROR, which stops and torque-offs both. Because both motors sit
behind one adapter (a single point of failure), do not rely on the soft stop
alone — keep controller-side gating or a power cut available.

**Update rate.** A twin cycle is ~4 serial round-trips (2× `read_monitor` + 2×
velocity writes) — one more than dual — so `twin_controllers.yaml` sets
`update_rate: 10`. Raise it only if a bench measurement of `read()+write()`
confirms < 80 ms.

#### Minimal URDF (twin)

The shipped `mdrobot_twin.urdf.xacro` already has this shape and fills the values
from `twin_controllers.yaml`; it is shown here only so you can see the joint layout
(`motor_L` / `motor_R`, mapped to the controllers by order):

```xml
<ros2_control name="mdrobot_twin" type="system">
  <hardware>
    <plugin>mdrobot_ros2_control/MdrobotSystemHardware</plugin>
    <param name="device_type">twin</param>
    <param name="port">/dev/ttyUSB0</param>
  </hardware>
  <joint name="motor_L">                 <!-- left wheel -->
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">1</param>
    <param name="reverse">false</param>
    <param name="counts_per_rev">24</param>
  </joint>
  <joint name="motor_R">                 <!-- right wheel -->
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">2</param>     <!-- must differ from motor_L -->
    <param name="reverse">true</param>   <!-- mirrored mount -->
    <param name="counts_per_rev">24</param>
  </joint>
</ros2_control>
```

## Controllers & bringup

`bringup.launch.py` starts `robot_state_publisher`, the `controller_manager`, and
spawns the controllers from `config/<device_type>_controllers.yaml`. **Edit that
file** for your robot — both the connection settings (`mdrobot_hardware` section)
and the controller params live there:

- **single** → `joint_state_broadcaster` + `velocity_cont`
  (`forward_command_controller`, commands `/velocity_cont/commands`).
  Needs `ros-jazzy-ros2-controllers`. Joint: `motor1`.
- **dual** → `joint_state_broadcaster` + `diff_cont` (`diff_drive_controller`,
  `/diff_cont/cmd_vel`, `geometry_msgs/TwistStamped`). Wheels: `motor_L` / `motor_R`.
- **twin** → `joint_state_broadcaster` + `diff_cont`, same as dual but at
  `update_rate: 10`; set each wheel's `motor_id_L` / `motor_id_R`, `reverse_L` /
  `reverse_R`, `counts_per_rev_L` / `counts_per_rev_R` in `twin_controllers.yaml`.

```bash
# set port / motor ids / counts_per_rev in config/<type>_controllers.yaml first
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=single
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=dual
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=twin
# (optional) override the port without editing the yaml:
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=twin port:=/dev/ttyUSB1

# drive (dual or twin): linear.x in m/s -> wheels
ros2 topic pub /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.1}}}"
```

`counts_per_rev` selects units: **positive → SI** (`rad`, `rad/s`), **`0` → raw**
(`count`, `rpm`). Set it per wheel in the yaml (`counts_per_rev` for single/dual,
`counts_per_rev_L` / `counts_per_rev_R` for twin). For single/dual you can also
override it on the command line with `counts_per_rev:=<value>` (empty, the default,
keeps the yaml value).

For a complete, runnable robot (proper URDF geometry, RViz, mock/real switch,
odometry + TF) see the **[`mdrobot_diffbot_example`](../../src/mdrobot_diffbot_example/README.md)**
package.

## Notes

- **Update rate:** each read+write cycle is a few 19200-baud round-trips. A dual
  cycle (monitor read + two velocity writes) is ~50 ms, so keep
  `controller_manager` `update_rate` around **15 Hz** for dual; higher rates
  overrun. **Twin** does two reads + two writes on the one bus, so it ships at
  **10 Hz** (see `twin_controllers.yaml`).
- **Lifecycle:** `on_configure` opens the port, `on_activate` enables, `on_deactivate`
  does stop + torque-off. Mode (velocity vs position) follows whichever command
  interface a controller claims.
- **Safety:** start at low speed, no load, with an emergency stop within reach.
  This is a generic driver — soft limits, odometry and kinematics belong in the
  robot layer above it.
- **Firmware & DIP:** recent firmware ships in encoder mode; driving without an
  encoder needs `ENC_PPR (156) = 0` (one-time, set with the python/C++ library).
  See [README → Hardware setup](README.md#hardware-setup).
