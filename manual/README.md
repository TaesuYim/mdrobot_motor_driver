# mdrobot_motor_driver — User Manual

Guide to connecting, reading, and driving MDROBOT MD-series motor controllers —
from the plain Python or C++ library, the ROS 2 node, or `ros2_control`.

## Read in this order

Pick the interface you actually use — the four manuals are independent of each
other, ordered low-level → high-level. Pass your serial port directly
(`open("/dev/ttyUSB0")`, or `port:` in the ROS yaml); **[Port setup](setup/port-setup.md)
is optional** — a convenience for not retyping the port (a udev fixed name and/or the
`MDROBOT_PORT` default).

| # | Page | Use it for |
|---|---|---|
| 1 | **[Python library](python.md)** | `mdrobot` — connect, read, drive, position control, slow ramps, raw registers, unit conversion, **full API reference (tables)**, error handling. Worth skimming even if you only use ROS 2 — the [first-drive checklist](python.md#first-drive-checklist) lives here. |
| 2 | **[C++ library](cpp.md)** | `mdrobot_cpp` — same API in C++ (`*Connection::open` factory, object lifetime, **API reference tables**, error handling). |
| 3 | **[ROS 2 node](ros2.md)** | `mdrobot_ros2_driver` — build, launch, parameters, topics/services, `joint_states` units, shutdown, troubleshooting. |
| 4 | **[ros2_control (C++)](ros2_control.md)** | `mdrobot_ros2_control` — the `SystemInterface` plugin: URDF parameters, state/command interfaces, units, controllers, and **twin mode** (two single-channel controllers on one bus). |

## Reference

Not a reading step — a lookup companion to every page above:

| Page | Use it for |
|---|---|
| **[Register reference](reference/registers.md)** | Full table of register numbers, command codes and status-1 bits (derived from `registers.py` / `status.py`) for raw access. |

> Both single-channel (one motor) and dual-channel (two motors) controllers are
> supported. The driver is **generic** — it exposes per-motor commands and state
> and contains no robot kinematics; differential drive, odometry and limits
> belong in the robot layer above it.

## Hardware setup

If you are **not using an encoder**, send `ENC_PPR (156) = 0` once to set the encoder
PPR to 0. (Recent firmware ships in encoder mode; older firmware such as v8.1 needs
nothing here.) **Until you do, the first command makes the motor lurch ~0.6 s and then
alarm** — keep clear on the first power-up.

For the full ordered first-drive sequence (comms check → `ENC_PPR` → `USE_LIMIT_SW` →
`enable()` → low rpm + dwell + a stop in reach), see the
[Python manual first-drive checklist](python.md#first-drive-checklist).

**USB-RS485 adapter:** any of them works, but the adapter's buffering — not the baud
rate — sets how fast the control loop can run, and swapping adapters changes the port
name. If you drive two controllers on one bus (`twin`) or want to raise `update_rate`,
read [Port setup → Adapter latency](setup/port-setup.md#adapter-latency--it-sets-your-maximum-update-rate)
first; an FTDI at its factory setting costs about 30 ms per twin cycle.

### Stop input (CTRL connector)

**Serial-only control:** set `USE_LIMIT_SW (17) = 0`. The CTRL inputs are then ignored
entirely and nothing below applies.

**With a hardware stop switch:** set `USE_LIMIT_SW (17) = 1` — but understand what that
does first. Under serial control the CTRL inputs become **one gate per rotation
direction**:

| CTRL pin | Gates | The motor turns that way only if |
|---|---|---|
| **6** `DIR` | CW = **negative** rpm | pin 6 is shorted to GND |
| **8** `START/STOP` | CCW = **positive** rpm | pin 8 is shorted to GND |

CTRL inputs are internally pulled up: **shorted to GND = ON, left open = OFF.**

So wiring **only pin 8** gives you a working stop in one direction and leaves
**negative rpm permanently blocked** — the motor never turns backwards. On a
differential base this is worse than it sounds: a skid-steer mounts the two wheels
mirrored, so one of them runs `reverse: true`, and driving the base *straight forward*
already commands negative rpm on that wheel.

**Wire both gates through one switch.** Per controller, tie pin 6 and pin 8 together
and take them through a single **normally-closed** contact to that controller's own
GND (pin 1 or 9):

```
CTRL  pin 6 (DIR)        ─┬── NC stop contact ── pin 1/9 (GND)
      pin 8 (START/STOP) ─┘
```

Closed = both directions permitted. Open = the motor stops whichever way it was turning.

**Two controllers (twin):** use a **2-pole** NC switch, one pole per controller, each
returning to that controller's own GND pin. A single-pole switch shared across both
units works only if they have a solid common ground (which the RS485 link requires
anyway), but then the pull-up return current depends on that ground path — an easy
thing to get wrong later.

**Before you rely on this:**

- **This is not a functional-safety e-stop.** Releasing the switch **re-arms the motor
  immediately** if the command source is still publishing — and `ros2_control` publishes
  every cycle, so the base drives off again the moment you let go. For a real emergency
  stop use a **power-cut contactor**, and stop the command source as well.
- Pin 8 gives a **coast** stop (the load's own inertia), not a braked stop.
- `USE_LIMIT_SW` may **not survive a power cycle** on some units (a reset to 0 was
  observed on an MD400 v8.6) — and if it resets, the stop switch silently stops working.
  Re-assert it at every startup (`ros2_control`: set `use_limit_sw: 1` in the yaml so
  `on_configure` writes it each run; library: write register 17 before your first
  command), then read it back and confirm it is 1.
- **Not usable with a wired encoder** on MD400: the encoder A/B lines share the CTRL
  limit inputs, so `USE_LIMIT_SW = 1` blocks all motion. Encoder mode means
  `USE_LIMIT_SW = 0` and no CTRL stop.
- **Pin 7 (RUN/BRAKE) will not stop a continuously commanded motor** — a periodic
  velocity command overrides it every cycle. Use pins 6 + 8.

> The pin-6 / pin-8 direction split follows the controller manual and a user report.
> The pin-8 gate and the digital-input bit polarity were measured on an MD400 v8.6 for
> this project; **the pin-6 gate was not measured here.**

**Two controllers on one bus (twin):** to drive a skid-steer base from two
single-channel controllers (e.g. two MD400) over one RS485 bus, give each a
distinct Modbus slave id first — with only that unit on the bus, write `PID_ID (133)`
with the wire word `(new_id << 8) | 0xAA` (high byte = new id, low byte = the `0xAA`
write-check; e.g. id 2 → `0x02AA`), power-cycle, then use `device_type=twin`. Full
steps: [ros2_control → Twin mode](ros2_control.md#twin-mode--two-single-channel-controllers-on-one-bus).
