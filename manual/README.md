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

`USE_LIMIT_SW (17)` selects what the CTRL inputs do under serial control:

- `0` — they are ignored; only the serial command moves the motor.
- `1` — they act as **one gate per rotation direction**:

| CTRL pin | Permits | The motor turns that way while |
|---|---|---|
| **6** `DIR` | CW = **negative** rpm | pin 6 is shorted to GND |
| **8** `START/STOP` | CCW = **positive** rpm | pin 8 is shorted to GND |

The inputs are internally pulled up: **shorted to GND = ON, left open = OFF.**

**Wiring a stop switch.** The two directions are gated separately, so the switch has to
open both. Per controller, tie pin 6 and pin 8 together and take them through one
**normally-closed** contact to that controller's own GND (pin 1 or 9), and set
`USE_LIMIT_SW = 1`:

```
CTRL  pin 6 (DIR)        ─┬── NC stop contact ── pin 1/9 (GND)
      pin 8 (START/STOP) ─┘
```

Closed = both directions permitted. Open = the motor stops whichever way it was turning.

For **twin**, use a **2-pole** NC switch — one pole per controller, each returning to
that controller's own GND pin. A single-pole switch shared across both units relies on
the two having a common ground.

**How it behaves.** Measured on two MD400 v8.6 driven with a periodic velocity command:

- Opening the contact stops the motor within one control cycle. The stop is a **coast** —
  the load's inertia carries it — not a braked stop.
- Closing it again restarts the motor as soon as the next command arrives, and
  `ros2_control` sends one every cycle.
- A switch on **pin 8 alone** stops CCW and leaves CW running: at −30 rpm, holding pin 8
  open for 3.5 s produced no deceleration. Pin 6 is what covers the other direction.
  (On a skid-steer the `reverse: true` wheel runs on negative rpm when the base drives
  forward.)
- `USE_LIMIT_SW` was seen back at `0` after a power cycle, so set it on every startup —
  `use_limit_sw: 1` in the `ros2_control` yaml, which writes it on each `on_configure`,
  or register 17 from the library — and read it back.
- On MD400 the encoder A/B lines share the CTRL limit inputs, so with an encoder wired
  `USE_LIMIT_SW = 1` blocks all motion. Encoder mode means `USE_LIMIT_SW = 0` and no
  CTRL stop.
- Pin 7 (`RUN/BRAKE`) is overridden by the periodic velocity command, so it does not
  stop a continuously driven motor.

> Measured here: the gates are per direction (pin 8 stops CCW, does nothing to CW), and
> pins 6 and 8 both grounded drive both directions. That **pin 6** is the CW gate comes
> from the controller manual and a user report; it was not isolated in a test.

**Two controllers on one bus (twin):** to drive a skid-steer base from two
single-channel controllers (e.g. two MD400) over one RS485 bus, give each a
distinct Modbus slave id first — with only that unit on the bus, write `PID_ID (133)`
with the wire word `(new_id << 8) | 0xAA` (high byte = new id, low byte = the `0xAA`
write-check; e.g. id 2 → `0x02AA`), power-cycle, then use `device_type=twin`. Full
steps: [ros2_control → Twin mode](ros2_control.md#twin-mode--two-single-channel-controllers-on-one-bus).
