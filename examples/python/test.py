#!/bin/python3

import gi
gi.require_version('GSerial', '1.0')
from gi.repository import GSerial

port = GSerial.Port.new()

port.baud = 57600

if(port.open("/dev/ttyUSB0")):
    print("opened ok")

    while(True):
        while(port.bytes_available()):
            print(port.read_string(port.bytes_available()))


