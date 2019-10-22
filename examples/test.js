#!/usr/bin/gjs

const Lang = imports.lang;
const gserial = imports.gi.gserial;

var port = gserial.Port.new_with_params(gserial.ByteSize.EIGHTBITS, gserial.Parity.NONE, 1, 57600);


port.open("/dev/ttyUSB0");

if(port.is_open()) {
	print("port is open!");
	while(true) {
		port.write("hello world!");

		print("available: " + port.available());
		var b = port.read_bytes_until(" ".charCodeAt(0));
		print(b);
	}
}



