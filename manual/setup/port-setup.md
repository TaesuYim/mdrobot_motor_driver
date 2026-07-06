# Serial port setup (optional) — stop retyping the port

**You do not need this page to get started** — just pass your port to `open()`
(`open("/dev/ttyUSB0")`) or set it in the ROS yaml. This page is a convenience: set
the port up **once** so you stop typing `/dev/ttyUSB0` into every command — and stop
chasing it when it silently becomes `ttyUSB1` after a re-plug.

**Which option do I want?**

| If you… | do this |
|---|---|
| just want to run it now | pass the port directly — `open("/dev/ttyUSB0")`, or `port:` in the ROS yaml. You can skip this page. |
| are tired of retyping, or it keeps moving | **Option 1** (udev fixed name) or **Option 2** (`MDROBOT_PORT`) below |
| use ROS 2 | set `port:` in the yaml regardless (the ROS layers do not read `MDROBOT_PORT`) |

Two independent problems this page can solve:

1. **The number drifts.** `/dev/ttyUSB0` is assigned in plug-in order; after a
   re-plug or reboot the same adapter can come back as `ttyUSB1`.
2. **Permissions.** `/dev/ttyUSB*` belongs to the `dialout` group; without
   membership you get *Permission denied* until you `chmod` the device — again
   after every re-plug.

## Option 1 (recommended) — a udev rule: fixed name + permissions, forever

A one-time udev rule gives the adapter a **permanent name** (e.g.
`/dev/mdrobot`) and the right permissions, at the OS level. Every layer —
Python, C++, the ROS 2 node's yaml, `ros2_control`'s yaml — then uses that one
string, and neither problem ever comes back.

First identify the adapter (with it plugged in):

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|{serial}' | head -6
```

This walks **up** the USB tree, so it prints several blocks; the **first (topmost)**
one is the adapter itself — take its values and ignore the parent hubs below it:

```
    ATTRS{idVendor}=="1a86"        # <- the adapter (topmost block): a CH340
    ATTRS{idProduct}=="7523"       #    CH340 has no serial -> match by idVendor/idProduct
    ATTRS{idVendor}=="1d6b"        # a parent USB hub — ignore this and everything below
```

(An FTDI adapter also prints `ATTRS{serial}=="…"` in its top block — pin that.)

Then create `/etc/udev/rules.d/99-mdrobot.rules` with a rule matching your
adapter. For a **CH340** (`1a86:7523`, the common blue adapters):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="mdrobot", MODE="0666"
```

For an **FTDI** (`0403:6001`) — these have a unique serial, so pin it exactly
(replace `FTB6SPL3` with the serial the identify command above printed):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="FTB6SPL3", SYMLINK+="mdrobot", MODE="0666"
```

Activate and verify:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/mdrobot          # -> symlink to the current ttyUSBn
```

Now use `/dev/mdrobot` everywhere: `SingleMotorDriver.open("/dev/mdrobot")`,
`port: /dev/mdrobot` in the ROS yaml files, and so on. To also make the no-arg
`open()` used throughout the library manuals work, combine with Option 2:
`export MDROBOT_PORT=/dev/mdrobot` in `~/.bashrc`.

> `MODE="0666"` makes the device world-read/writable, which also removes the
> `dialout` problem. If you prefer group-based access, use
> `GROUP="dialout", MODE="0660"` instead and add yourself once:
> `sudo usermod -aG dialout $USER` (then log out / back in).

## Option 2 — `MDROBOT_PORT`: a default port for the libraries

The Python and C++ libraries (and the [`examples/`](../../examples/) scripts) fall
back to the **`MDROBOT_PORT`** environment variable when no port is given. Set
it once — e.g. in `~/.bashrc` — and drop the port from your code entirely:

```bash
export MDROBOT_PORT=/dev/mdrobot     # or /dev/ttyUSB0, or a by-id path
```

```python
from mdrobot import SingleMotorDriver
with SingleMotorDriver.open() as d:          # port comes from $MDROBOT_PORT
    print(d.get_version())
```

```cpp
auto conn = mdrobot::SingleMotorConnection::open();  // port from $MDROBOT_PORT
```

```bash
python3 examples/quickstart.py --type single         # --port no longer needed
```

An **explicit port always wins** over the environment variable. If neither is
set, `open()` fails immediately with a message pointing here (Python:
`ValueError`; C++: `std::invalid_argument`) — it never guesses a device.

> **The ROS 2 layers take their port from the yaml, not from `MDROBOT_PORT`.**
> The node and the `ros2_control` plugin never consult the variable themselves —
> a launch setup should be explicit and self-contained — so always set `port`
> in their yaml. (Leaving the yaml `port` empty would fall through to the
> library's fallback; don't rely on that.) Combine the two options: set
> `port: /dev/mdrobot` in the yaml once and it is just as stable.

## Option 3 — a stable `by-id` path (no sudo needed)

The kernel already provides re-enumeration-proof names:

```bash
ls /dev/serial/by-id/
# e.g. usb-1a86_USB_Serial-if00-port0        (CH340)
#      usb-FTDI_USB-RS485_FTB6SPL3-if00-port0 (FTDI)
```

Use that full path as the port (directly, or as `MDROBOT_PORT`). It fixes the
numbering drift without root, but not the permission problem, and note that
no-name CH340 clones carry no serial number — two identical CH340s get the
**same** by-id name and cannot be told apart this way.

## Several motors, several adapters?

- **One controller with two channels (dual), or two controllers on one bus
  (twin) — still one port.** The port belongs to the *bus* (the adapter), not
  the motor; motors are addressed by channel or Modbus slave id. One
  `/dev/mdrobot` / `MDROBOT_PORT` covers the whole bus.
- **Several adapters** (several buses) need one name each. FTDI adapters can be
  told apart by `ATTRS{serial}` as above. Serial-less clones (CH340) can be
  pinned to the *physical USB socket* instead — find it with
  `udevadm info -a -n /dev/ttyUSB0 | grep -m3 KERNELS` and take the **bus-path
  value** (like `1-2`) — not `ttyUSB0` (that would pin plug-in order again, the
  very problem this page solves) and not the `1-2:1.0` interface form — then
  match:

  ```
  SUBSYSTEM=="tty", KERNELS=="1-2", SYMLINK+="mdrobot-left",  MODE="0666"
  SUBSYSTEM=="tty", KERNELS=="1-3", SYMLINK+="mdrobot-right", MODE="0666"
  ```

  (Each adapter then must stay in its socket.) `MDROBOT_PORT` names only one
  default bus — pass the port explicitly for the others.
