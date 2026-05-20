#!/usr/bin/env sqgi

function usage() {
    print("Usage: sqgi examples/sqgi/string_echo.nut --device=PATH [--baud=9600]\n")
    print("Echoes every received text chunk back to the serial device.\n")
}

local device = null
local baud = 9600

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
port.set_timeout(500)

port.connect("connected", function() {
    print("echo server connected to " + device + " at " + baud + "\n")
})

port.connect("disconnected", function() {
    print("\ndisconnected\n")
    loop.quit()
})

port.connect("on_data", function(available) {
    local data = port.read_string(available)
    if (data == null) return

    print("received: " + data + "\n")
    port.write_string("Echo: " + data)
})

if (!port.open(device)) {
    print("failed to open " + device + "\n")
    return 1
}

loop.run()
