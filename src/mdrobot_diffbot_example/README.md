# mdrobot_diffbot_example

A minimal **differential-drive robot** driven by a MDROBOT dual controller
(PNT50 / MD400T) through [`mdrobot_ros2_control`](../mdrobot_ros2_control). It
wires the hardware plugin to `diff_drive_controller` and shows the full
`ros2_control` path: `/cmd_vel` → wheels → odometry + TF, visualised in RViz.

This is an **example consumer** of the generic driver — the robot geometry,
`wheel_radius`, `wheel_separation` and kinematics live here, not in the driver.

## Contents

```
description/diffbot.urdf.xacro          # robot: base + 2 wheels + caster + ros2_control
description/diffbot.ros2_control.xacro   # mock_components <-> MdrobotSystemHardware switch
config/diffbot_controllers.yaml          # joint_state_broadcaster + diff_cont
rviz/diffbot.rviz                        # RobotModel + TF + Odometry
launch/diffbot.launch.py                 # rsp + controller_manager + spawners + RViz
```

## Run

```bash
colcon build --packages-select mdrobot_cpp mdrobot_ros2_control mdrobot_diffbot_example
source install/setup.bash
```

### Mock hardware (no device — try it in RViz)

```bash
ros2 launch mdrobot_diffbot_example diffbot.launch.py
```

`mock_components/GenericSystem` integrates the velocity commands, so the wheels
spin and odometry moves with no controller hardware attached.

### Real hardware (PNT50 / MD400T)

```bash
ros2 launch mdrobot_diffbot_example diffbot.launch.py \
    use_mock_hardware:=false port:=/dev/ttyUSB1 counts_per_rev:=12.0
```

### Drive it

```bash
# keyboard teleop (Jazzy diff_drive expects TwistStamped)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/diff_cont/cmd_vel -p stamped:=true

# or a one-off
ros2 topic pub /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.15}, angular: {z: 0.3}}}"
```

Odometry is on `/diff_cont/odom`; the `odom → base_footprint` TF is published.

## Launch arguments

| arg | default | meaning |
|---|---|---|
| `use_mock_hardware` | `true` | `true`: mock (no device); `false`: real MDROBOT dual |
| `port` | `/dev/ttyUSB1` | serial port (real hardware) |
| `counts_per_rev` | `12.0` | counts/rev per wheel motor (PNT50 measured: 12) |
| `update_rate` | `0` (auto) | controller_manager Hz; `0` → mock 30 / real 15 |
| `rviz` | `true` | launch RViz |

> **Calibrate for your robot.** `wheel_radius` / `wheel_separation` in the URDF
> and `config/diffbot_controllers.yaml` must match your chassis. `counts_per_rev`
> must be **counts per one wheel revolution** — measure it by turning the *wheel*
> (not the motor) exactly N turns, so any gearbox ratio is included; otherwise the
> odometry is off by the gear ratio. The serial link bounds the loop rate — keep
> the real dual at ~15 Hz.
