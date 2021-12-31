#!/bin/python3

import gi
gi.require_version('GSerial', '1.0')
from gi.repository import GSerial
from gi.repository import GLib


port = GSerial.Port.new()
port.baud = 57600


def connected(port):
    print("Port connected ok")

def disconnected(port):
    print("Port disconnected")

def on_data(port, available):
    data = port.read_bytes(available)

port.connect("on_data", on_data)
port.connect("connected", connected)
port.connect("disconnected", disconnected)

if(port.open("/dev/ttyUSB0")):
    print("opened ok")


loop = GLib.MainLoop()
loop.run()


