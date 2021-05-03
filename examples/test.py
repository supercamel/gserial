#!/mingw64/bin/python3

print("HERE")
import gi
gi.require_version('gserial', '1.0')
from gi.repository import gserial

print("starting...")
port = gserial.Port()

print("opening port")
port.open("/dev/ttyUSB0")

if(port.is_open() == False):
    print("failed to open port");
    exit()
else:
    print("port opened OK")

print(port.write("Hello world!".encode("utf-8")))
while(True):
    if(port.available() > 0):
        print(port.read_bytes(port.available()))



