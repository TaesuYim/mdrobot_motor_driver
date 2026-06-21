# Python Library Usage (`mdrobot`)

`mdrobot` is a pure-Python RS485 / Modbus RTU driver for MDROBOT MD-series motor
controllers. It works without ROS 2.

## Install

```bash
# from the repo root
pip install -e 'src/mdrobot[serial]'    # [serial] installs pyserial
```

Verify the import:

```bash
python -c "import mdrobot; print(mdrobot.__file__)"
```

## Connect

1. Wire the RS485 (USB-serial) adapter to the controller's RS485 A/B. Default
   link settings: **19200 8N1**, controller ID **1**.
2. **Port permission (Linux)** — if your user is not in the `dialout` group, the
   port is not accessible:
   ```bash
   sudo usermod -aG dialout $USER   # then log out / back in
   # or, temporarily:
   sudo chmod a+rw /dev/ttyUSB0
   ```
3. A controller is **dual-channel** if it answers the dual-only monitor
   registers, otherwise it is **single-channel**.

## Read (no motion)

```python
from mdrobot import SingleMotorDriver

with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    print("version:", d.get_version())
    print("voltage:", d.get_voltage(), "V")
    print("status :", d.get_status().active)   # active alarm/status bits
    print("monitor:", d.read_monitor())         # speed / current / position
```

Reading never moves the motor. If communication fails, isolate the cause among
baud rate / controller ID / wiring / CRC.

## Drive

> **Prerequisite:** call `enable()` before driving. `enable()` sets `UI_COM = 1`
> (serial control) and arms `START/STOP`. Without it, velocity commands are
> echoed but the motor does not turn.

### Single-channel

```python
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocity(40)       # signed rpm; + = CCW (increasing position)
    d.set_velocity(-40)
    d.stop()                 # decelerate to 0
    d.torque_off()           # free / no torque

    # position control (needs UI_COM=1 only)
    d.reset_position()
    d.move_to(80, speed=60)  # absolute move, stops on arrival
    d.wait_in_position()
    d.move_by(-40, speed=60) # relative move
```

### Dual-channel

```python
from mdrobot import DualMotorDriver

with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocities(40, 40)        # motor 1 and motor 2
    # Note: some dual-channel controllers take ~1 s to start turning after a
    # command — don't send 0 immediately or you'll miss the motion.
    d.stop()
    d.move_to_both(50, 50, speed1=60)
    d.torque_off_both()
```

### Sign / direction

`+` = CCW = increasing position; `-` = CW = decreasing position (single and
dual alike). Confirm the actual direction once in your installation.

## High-level API

Shared by both drivers: `open()`, `close()`, `get_version()`, `get_voltage()`,
`get_status()`, `ping()`, `enable()`, `disable()`, `reset_alarm()`.

**`SingleMotorDriver`**: `set_velocity(rpm)`, `stop()`, `brake()`,
`torque_off()`, `reset_position()`, `get_speed()`, `get_current()`,
`get_position()`, `read_monitor()`, `move_to(pos, speed)`,
`move_by(delta, speed)`, `get_in_position()`, `wait_in_position()`.

**`DualMotorDriver`**: `set_velocities(rpm1, rpm2)`,
`set_velocity(channel, rpm)`, `stop()`, `stop_channel(channel)`,
`brake_both()`, `brake(channel)`, `torque_off_both()`, `torque_off(channel)`,
`read_monitor()`, `get_speed(channel)`, `get_positions()`,
`get_position(channel)`, `reset_position()`,
`move_to_both(pos1, pos2, speed1, speed2)`,
`move_by_both(delta1, delta2, speed1, speed2)`.

**Acceleration / deceleration (slow-start / slow-down)** — *per protocol docs, not
yet hardware-verified.* Single: `set_slow_start(seconds)` / `get_slow_start()`,
`set_slow_down` / `get_slow_down`, plus `set/get_position_slow_start/down`. Dual:
the same with a leading `channel`, e.g. `set_slow_start(channel, seconds)`. Shared:
`clear_slow_start()`, `clear_slow_down()`, `clear_position_slow_start()`,
`clear_position_slow_down()`. Times use a 0–15 s default scale (full scale set by
`PID_MAX_SS_TIME`); the conversion helpers are `slow_seconds_to_raw` /
`slow_raw_to_seconds`.

## Low-level (raw) access

Anything the high-level API doesn't cover is reachable through the Modbus client
at `d.client`: `read_register`, `read_registers`, `write_register`,
`write_registers`, `read_long`, `write_long`, `command`. Example — force
`USE_LIMIT_SW = 0` (register 17) on a single-channel controller that needs it
for serial drive:

```python
d.client.write_register(17, 0)
```

## Safety

- Start at **low speed** and keep an emergency stop / power cut within reach.
- Position moves stop automatically on arrival, but a wrong large target can
  cause excessive rotation — verify with small values first.

## Troubleshooting — motor won't move

1. Did you call `enable()`? (`UI_COM=1` + `START/STOP` arm)
2. **Single-channel**: some controllers require `USE_LIMIT_SW = 0` (register 17)
   for serial drive. Connecting an encoder can make this mandatory (A/B share
   pins with limit inputs).
3. **Dual-channel, motor 2 not turning**: motor 2 must be commanded on its own
   register — `set_velocities()` already does this.
4. Some dual-channel controllers turn **~1 s after** the command — don't judge
   "stopped" too early.
5. If `get_status()` shows an alarm bit, call `reset_alarm()`.
6. If readbacks look unstable, cross-check the version register to confirm the
   transaction is aligned (some adapters/controllers show session desync).
