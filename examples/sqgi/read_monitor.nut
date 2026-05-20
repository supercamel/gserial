#!/usr/bin/env sqgi

function usage() {
    print("Usage: sqgi examples/sqgi/read_monitor.nut --device=PATH [--baud=57600]\n")
    print("Example: sqgi examples/sqgi/read_monitor.nut --device=/dev/ttyUSB0 --baud=115200\n")
}

local device = null
local baud = 57600

foreach (arg in vargv) {
    if (arg == "--help" || arg == "-h") {
        usage()
        return 0
    } else if (arg.find("--device=") == 0) {
        device = arg.slice(9)
    } else if (arg.find("--baud=") == 0) {
        baud = arg.slice(7).tointeger()
    }
}

if (device == null) {
    usage()
    return 1
}

local GLib = import("GLib")
local GSerial = import("GSerial", "1.0")

local loop = GLib.MainLoop.new(null, false)
local port = GSerial.Port.new()

port.set_baud(baud)
port.set_timeout(100)

port.connect("connected", function() {
    print("connected to " + device + " at " + baud + "\n")
})

port.connect("disconnected", function() {
    print("\ndisconnected\n")
    loop.quit()
})

port.connect("on_data", function(available) {
    local data = port.read_string(available)
    if (data != null) print(data)
})

if (!port.open(device)) {
    print("failed to open " + device + "\n")
    return 1
}

print("listening; press Ctrl+C to stop\n")
loop.run()
