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
* Works on Linux and Windows

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
port.baud = 57600

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

See the examples/ folder for more advanced usage with byte-level access, event handling, and write-back.

## 🧪 Status

GSerial is stable and in use in real-world applications. Further contributions, testing, and packaging help are welcome!

## 📜 License

MIT License — free to use, modify, and distribute.

## 🤝 Contributing

Issues and pull requests are welcome! If you have a GTK-based serial tool or hardware project, GSerial might be a great fit. Feedback is appreciated.