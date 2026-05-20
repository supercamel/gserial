# SQGI Examples

These examples use GSerial through GObject Introspection from SQGI.

When running from this source tree, build GSerial first and point SQGI at the
generated typelib and shared library:

```sh
meson compile -C builddir
export GI_TYPELIB_PATH="$PWD/builddir${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}"
export LD_LIBRARY_PATH="$PWD/builddir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

Examples:

```sh
sqgi examples/sqgi/read_monitor.nut --device=/dev/ttyUSB0 --baud=57600
sqgi examples/sqgi/string_echo.nut --device=/dev/ttyUSB0 --baud=9600
sqgi examples/sqgi/gtk_terminal.nut --device=/dev/ttyUSB0 --baud=9600
```

On Windows, use a COM port such as `--device=COM3`.
