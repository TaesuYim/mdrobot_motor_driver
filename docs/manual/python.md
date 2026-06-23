# Python Library Usage (`mdrobot`)

`mdrobot` is a pure-Python RS485 / Modbus RTU driver for MDROBOT MD-series motor
controllers. It works without ROS 2.

- **Single-channel** controllers (one motor) → `SingleMotorDriver`
- **Dual-channel** controllers (two motors) → `DualMotorDriver`
- Low-level register/command access is always available via `driver.client`.

## Install

```bash
pip install -e 'src/mdrobot[serial]'    # [serial] installs pyserial
python -c "import mdrobot; print(mdrobot.__file__)"   # verify
```

## Connect

1. Wire the RS485 (USB-serial) adapter to the controller's RS485 A/B, and tie the
   adapter and controller **GND** together. Default link settings: **19200 8N1**,
   controller ID **1**.
2. **Find the port.** The adapter usually enumerates as `/dev/ttyUSB0`; list the
   candidates and watch which one appears when you plug it in:
   ```bash
   ls /dev/serial/by-id/        # stable per-adapter names (survive re-enumeration)
   ls /dev/ttyUSB* /dev/ttyACM*
   dmesg | grep -i tty | tail   # which device attached just now
   ```
   Prefer the `/dev/serial/by-id/...` path when you run more than one adapter.
3. **Port permission (Linux)** — if your user is not in the `dialout` group:
   ```bash
   sudo usermod -aG dialout $USER   # then log out / back in
   sudo chmod a+rw /dev/ttyUSB0     # or, temporarily
   ```
4. A controller is **dual-channel** if it answers the dual-only monitor
   registers, otherwise it is **single-channel**.

> **No reply / `IncompleteResponseError`?** The most common first-connection
> causes are swapped **A/B** lines (try swapping them), a missing common **GND**,
> or the wrong **baud rate / ID**. Termination/bias resistors are rarely needed on
> a short, low-speed (19200) bus. Per-model verification status is in
> [`tested-devices.md`](../dev/tested-devices.md).

## Quick start

```python
from mdrobot import SingleMotorDriver, DualMotorDriver

# --- read only (never moves the motor) ---
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    print(d.get_version(), d.get_voltage(), "V")
    print(d.get_status().active)     # active alarm/status bit names
    print(d.read_monitor())          # speed / current / position

# --- single-channel drive ---
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()                       # REQUIRED before motion
    d.set_velocity(40)               # signed rpm; + = CCW
    d.stop(); d.torque_off()
    d.reset_position()
    d.move_to(80, speed=60); d.wait_in_position()   # absolute move (counts)

# --- dual-channel drive ---
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocities(40, 40)         # motor 1, motor 2
    d.stop(); d.torque_off_both()
```

> **`enable()` is required before any motion.** It sets `UI_COM = 1` (serial
> control) and arms `START/STOP`. Without it, velocity commands are echoed but
> the motor does not turn.
>
> **Sign / direction:** `+` = CCW = increasing position; `-` = CW = decreasing
> (single and dual alike). Some dual controllers start turning **~1 s after** the
> command — don't send `0` immediately or you'll miss the motion.

---

# API reference

`rpm` is signed mechanical rpm. `position` / `count` is an INT32 encoder/hall
count (`+` = CCW). `speed` for position moves is the max rpm magnitude.

## Connection & shared (`SingleMotorDriver` / `DualMotorDriver`)

| Method | Returns | Description |
|---|---|---|
| `SingleMotorDriver.open(port, baudrate=19200, *, slave_id=1, timeout=0.3)` | driver | Open a port and build the driver. Use as a context manager or `close()` it. |
| `DualMotorDriver.open(port, baudrate=19200, *, slave_id=1, timeout=0.3)` | driver | Same, for a dual-channel controller. |
| `close()` | `None` | Close the serial port. (Context-manager `with` does this automatically.) |
| `ping()` | `bool` | `True` if the controller answers a version read. |
| `get_version()` | `int` | Firmware/DL version register. |
| `get_voltage()` | `float` | Input voltage (V). |
| `get_status()` | `StatusBits` | Status-1 bits (see [Data types](#data-types)). |
| `enable()` | `None` | Allow motion: `UI_COM = 1` + arm `START/STOP`. Call before driving. |
| `disable()` | `None` | Clear `UI_COM` (motion gated off). |
| `reset_alarm()` | `None` | Clear a latched alarm. |

```python
with SingleMotorDriver.open("/dev/ttyUSB0", 19200, slave_id=1) as d:
    if d.ping():
        d.enable()
```

## `SingleMotorDriver`

| Method | Returns | Description |
|---|---|---|
| `set_velocity(rpm)` | `None` | Drive at signed `rpm` (`+` = CCW). `0` decelerates to stop. |
| `stop()` | `None` | Command speed `0` (controlled stop). |
| `brake()` | `None` | Short-brake stop. |
| `torque_off()` | `None` | Release torque (free spin). |
| `get_speed()` | `int` | Measured speed (signed rpm). |
| `get_current()` | `float` | Motor current (A). |
| `get_position()` | `int` | Position count (INT32). |
| `read_monitor()` | `Monitor` | Speed + current + output + position in one read. |
| `reset_position()` | `None` | Set the position counter to 0. |
| `move_to(position, speed=100)` | `None` | Absolute position move to `position` counts at ≤ `speed` rpm; stops on arrival. Needs `UI_COM=1` only (no `START/STOP` arm). |
| `move_by(delta, speed=100)` | `None` | Relative move by `delta` counts at ≤ `speed` rpm. |
| `get_in_position()` | `bool` | `True` when the last position move has arrived. |
| `wait_in_position(timeout=10.0, poll=0.1)` | `bool` | Block until in-position or `timeout` s; `True` if arrived. |

```python
d.enable()
d.move_to(-120, speed=50)     # absolute, counts
if d.wait_in_position(timeout=8.0):
    print("arrived at", d.get_position())
```

## `DualMotorDriver`

`channel` is `1` or `2`. `set_velocities` writes each motor on its own register
(motor 2 will not move from a single combined register on tested hardware).

| Method | Returns | Description |
|---|---|---|
| `set_velocities(rpm1, rpm2)` | `None` | Set both motors (signed rpm). |
| `set_velocity(channel, rpm)` | `None` | Set one motor. |
| `stop()` | `None` | Stop both motors. |
| `stop_channel(channel)` | `None` | Stop one motor. |
| `brake_both()` / `brake(channel)` | `None` | Short-brake both / one. |
| `torque_off_both()` / `torque_off(channel)` | `None` | Release torque on both / one. |
| `get_speed(channel)` | `int` | Measured speed of a channel (signed rpm). |
| `get_current(channel)` | `float` | Motor current of a channel (A), from `PNT_MAIN_DATA`. |
| `get_position(channel)` | `int` | Position count of a channel. |
| `get_positions()` | `tuple[int, int]` | Both position counts. |
| `read_monitor()` | `DualMonitor` | Speed + position for both (no current — lighter read). |
| `read_main_data()` | `DualMonitor` | Speed + **current** + position for both. |
| `reset_position()` | `None` | Zero both position counters. |
| `move_to_both(pos1, pos2, speed1=100, speed2=None)` | `None` | Absolute move both (counts). `speed2=None` reuses `speed1`. |
| `move_by_both(delta1, delta2, speed1=100, speed2=None)` | `None` | Relative move both (counts). |

```python
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocities(30, -30)            # spin opposite
    print(d.get_current(1), d.get_current(2))   # A
    d.stop(); d.torque_off_both()
```

## Acceleration / deceleration (slow-start / slow-down)

Ramp time maps to a 0–`PID_MAX_SS_TIME` second scale (default full scale 15 s).
**Speed** slow is hardware-verified; **position** slow is protocol-doc-based.

| Method (single / dual) | Returns | Description |
|---|---|---|
| `set_slow_start(seconds)` / `set_slow_start(channel, seconds)` | `None` | Speed acceleration ramp time (s). |
| `get_slow_start()` / `get_slow_start(channel)` | `float` | Read it back (s). |
| `set_slow_down(...)` / `get_slow_down(...)` | `None` / `float` | Speed deceleration ramp. |
| `set_position_slow_start/down(...)` / `get_position_slow_start/down(...)` | `None` / `float` | Position-mode ramps (doc-based). |
| `clear_slow_start()` / `clear_slow_down()` | `None` | Erase the speed ramps (shared). |
| `clear_position_slow_start()` / `clear_position_slow_down()` | `None` | Erase the position ramps (shared). |

All setters/getters take a keyword `full_scale_s=15.0` if your controller's
`PID_MAX_SS_TIME` differs.

```python
d.set_slow_start(2.0)     # 2 s to ramp up (single)
d.set_slow_down(1.5)
# dual: d.set_slow_start(1, 2.0)
```

## Data types

`StatusBits` (from `get_status()`):

| Field | Type | Meaning |
|---|---|---|
| `raw` | `int` | Raw status-1 byte. |
| `alarm` | `bool` | Any alarm latched. |
| `over_voltage`, `over_temperature`, `overload`, `stall`, `ctrl_fail`, `hall_or_encoder_fail`, `inverse_velocity` | `bool` | Individual fault bits. |
| `active` (property) | `list[str]` | Names of the set bits. |

`Monitor` (from single `read_monitor()`); `DualMonitor` has `.motor1` / `.motor2`
each a `Monitor`:

| Field | Type | Units / note |
|---|---|---|
| `speed_rpm` | `int` | signed rpm |
| `current_a` | `float \| None` | A (`None` for `read_monitor()` on dual — use `read_main_data()`) |
| `output_raw` | `int \| None` | controller output (−1023..1023) |
| `position` | `int` | INT32 count |

## Low-level register access (`driver.client`)

Anything the high-level API doesn't cover is reachable through the Modbus client.
PIDs/commands are in `mdrobot.registers`.

| Method | Returns | Description |
|---|---|---|
| `read_register(pid)` | `int` | Read one 16-bit register. |
| `read_registers(pid, count)` | `list[int]` | Read `count` consecutive registers. |
| `write_register(pid, word)` | `None` | Write one 16-bit register. |
| `write_registers(pid, words)` | `None` | Write consecutive registers. |
| `read_long(pid, *, signed=True)` | `int` | Read an INT32 (low word first). |
| `write_long(pid, value)` | `None` | Write an INT32. |
| `command(cmd)` | `None` | Issue a command code (`PID_COMMAND`). |

```python
from mdrobot import registers as reg
d.client.write_register(reg.PID_USE_LIMIT_SW, 0)   # = write_register(17, 0)
```

## Unit conversion (`mdrobot.units`)

| Function | Returns | Description |
|---|---|---|
| `counts_to_rad(count, counts_per_rev)` | `float` | Count → radians. |
| `rad_to_counts(rad, counts_per_rev)` | `int` | Radians → nearest count. |
| `rpm_to_rad_s(rpm)` | `float` | rpm → rad/s. |
| `rad_s_to_rpm(rad_s)` | `float` | rad/s → rpm. |
| `slow_seconds_to_raw(seconds, full_scale_s=15.0)` | `int` | Ramp seconds → raw (0–1023). |
| `slow_raw_to_seconds(raw, full_scale_s=15.0)` | `float` | Raw → seconds. |

`counts_per_rev` is counts per **one revolution of the motor shaft** — the
controller reports both position (count) and speed (rpm) at the motor, so measure
it there: turn the motor shaft exactly N turns and divide
([`examples/calibrate_counts_per_rev.py`](../../examples/calibrate_counts_per_rev.py)).
Gear ratio is **not** applied here: `counts_to_rad` scales position only, while
`rpm_to_rad_s` needs no `counts_per_rev`. Measuring at a geared output shaft would
make position the output angle while velocity stayed the motor rate — the two would
disagree by the gear ratio. Keep `counts_per_rev` at the motor and handle the
gearbox / wheel in the layer above.

## Error handling

All library errors derive from `MdrobotError`, so one `except` catches everything:

```text
MdrobotError
├── CrcError                 # CRC mismatch in a response
└── ProtocolError           # malformed / unexpected response
    └── IncompleteResponseError   # short read (timeout / wiring / wrong baud)
```

```python
from mdrobot import SingleMotorDriver
from mdrobot.exceptions import MdrobotError, IncompleteResponseError

try:
    with SingleMotorDriver.open("/dev/ttyUSB0") as d:
        d.enable()
        d.set_velocity(40)
except IncompleteResponseError:
    print("no/short reply — check baud rate, ID, wiring")
except MdrobotError as e:
    print("driver error:", type(e).__name__, e)
```

A timeout surfaces as `IncompleteResponseError`. On any failure the safe response
is `stop()` + `torque_off()` (the `with` block still closes the port).

## Safety

- Start at **low speed**, unloaded, with an emergency stop / power cut in reach.
- Position moves stop on arrival, but a wrong large target over-rotates — test
  with small values first.

## Troubleshooting — motor won't move

1. Did you call `enable()`? (`UI_COM=1` + `START/STOP` arm)
2. **Single-channel**: some controllers require `USE_LIMIT_SW = 0` (register 17)
   for serial drive. Connecting an encoder can make this mandatory (A/B share
   pins with limit inputs).
3. **Dual-channel, motor 2 not turning**: `set_velocities()` commands motor 2 on
   its own register — already handled.
4. Some dual controllers turn **~1 s after** the command — don't judge "stopped"
   too early.
5. `get_status().alarm` set? Call `reset_alarm()`.
6. Unstable readbacks → cross-check `get_version()` to confirm the transaction is
   aligned (some adapters show session desync).
