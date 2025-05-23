#!/usr/bin/env python3

import gi
gi.require_version('GSerial', '1.0')
from gi.repository import GSerial
from gi.repository import GLib

# Create a new port instance and configure parameters
port = GSerial.Port.new()
port.baud = 9600
port.timeout = 500  # Timeout in milliseconds

# Example: Echo received string
def on_data(port, available):
    # Read available bytes as a string
    data = port.read_string(available)
    print("Received string:", data)

    # Optionally, write a response back to the device
    response = "Echo: " + data
    port.write_string(response)

# Print confirmation when the port is connected
def connected(port):
    print("Port connected successfully")

# Handle disconnection
def disconnected(port):
    print("Port disconnected")

# Connect signal handlers
port.connect("on_data", on_data)
port.connect("connected", connected)
port.connect("disconnected", disconnected)

# Try opening the port
if port.open("/dev/ttyUSB0"):
    print("Port opened")
else:
    print("Failed to open port")
    exit(1)

# Run the main loop
loop = GLib.MainLoop()
loop.run()
