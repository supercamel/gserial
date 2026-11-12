# GSerial

GSerial is a lightweight, cross-platform library for serial port communication in GObject-based applications. Written in C with full GObject Introspection support, it is suitable for use in GTK+, GNOME, and GLib applications, and is easily accessible from C, Python, Vala, JavaScript (GJS), and other supported languages.

GSerial is ideal for developers building graphical frontends for embedded systems, Arduino projects, or other serial-connected devices.
## ✨ Features

* Simple, intuitive API
* Fully GObject-based with introspection support
* Asynchronous signal-based I/O ("on_data", "connected", "disconnected")

## Supports:

* Baud rate configuration
* Parity, stop bits, and data bits settings
* String, byte array, and line-based I/O
* Device enumeration on Linux, Windows, and Android
* Android USB OTG serial: FTDI, CDC ACM, CP210x, CH34x, and PL2303 adapters

## 📦 Dependencies

You’ll need:

* Meson and Ninja (build system)
* GObject Introspection
* A C compiler and GLib development headers

Install build tools:

```
sudo pip3 install meson ninja
```

Install development libraries (Debian/Ubuntu):

```
sudo apt install libgirepository1.0-dev libglib2.0-dev
```

On Windows, build with MSYS2 or another environment that supports GObject libraries.
## 🔧 Build & Install

```
meson --prefix=/usr builddir
ninja -C builddir
cd builddir && sudo meson install
```
## 🚀 Getting Started

Here’s a minimal Python example using the introspected library:

```python
import gi
gi.require_version('GSerial', '1.0')
from gi.repository import GSerial, GLib

port = GSerial.Port.new()
port.set_baud(57600)

def on_data(port, available):
    data = port.read_string(available)
    print("Received:", data)

port.connect("on_data", on_data)

if port.open("/dev/ttyUSB0"):
    print("Port opened")
else:
    print("Failed to open port")

GLib.MainLoop().run()
```

See the examples/ folder for more advanced usage with byte-level access, event handling, write-back, and SQGI scripts.

Configure baud, timeout, parity, data bits, stop bits, and nonblocking mode before
opening the port. Close it before changing settings. On POSIX, blocking writes
wait up to the configured timeout for buffer space and can return a partial byte
count; retry any unwritten bytes. Nonblocking mode and a zero timeout return
immediately.

Run the Linux regression tests with `meson test -C builddir --print-errorlogs`.
They use pseudo-terminals and require no serial hardware.

## Device discovery

`GSerial.DeviceManager.new().list_devices()` returns immutable `Device` snapshots.
Each provides `get_path()`, `get_name()`, USB vendor/product IDs, serial number,
port/interface number, and `get_has_permission()`. Pass the returned path to
`Port.open_full()` to receive detailed errors, or the existing `Port.open()` for
a boolean result. Discovery never opens or changes a serial port.

Linux prefers `/dev/serial/by-id` paths and reads USB metadata from sysfs.
Windows uses SetupAPI to discover COM ports. Android enumerates supported USB
serial interfaces through the USB host API, without root or a kernel tty driver.
These are the supported platforms; macOS and iOS backends are not implemented.

On Android, call `manager.request_permission(device.get_path())`, then handle
`permission-result(path, granted, message)` before opening. Call `lookup(path)`
after permission is granted and save its refreshed path. A unique USB serial
number gives a stable `usb://0403:6001/sSERIAL/0` identity. Before permission, or
when the serial number is unavailable or duplicated, the selector is a temporary
Android device ID (`%40...`). Reselect those adapters after unplug/replug; do not
treat temporary IDs or USB VID/PID alone as persistent radio identities. Android
may require a new permission grant after detach. Refresh the list and select the
adapter again if a saved identity cannot be resolved.

Device managers run on their creating thread's GLib context. Keep the manager
alive while requesting permission; `cancel_permission()` suppresses completion
but cannot dismiss an already-visible Android system dialog.

## Android / SQGI / Ooblerg

See [the Android guide](android/README.md) for packaging, the NDK entry point,
emulator regression commands, and RFD900 hardware checks. The
[device-picker example](examples/android/main.nut) connects at 57600 baud.

## 🧪 Status

GSerial is stable and in use in real-world applications. Further contributions, testing, and packaging help are welcome!

## 📜 License

MIT License — free to use, modify, and distribute.

## 🤝 Contributing

Issues and pull requests are welcome! If you have a GTK-based serial tool or hardware project, GSerial might be a great fit. Feedback is appreciated.
