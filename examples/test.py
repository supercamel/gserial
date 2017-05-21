#!/mingw64/bin/python3

print("HERE")
import gi
gi.require_version('gserial', '1.0')
from gi.repository import gserial

print("starting...")
port = gserial.Port()

print("opening port")
port.open("COM3")

if(port.is_open() == False):
    print("failed to open port");
    exit()
else:
    print("port opened OK")

print(port.write("Hello world!".encode("utf-8")))
