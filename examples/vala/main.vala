
public static int main() {
    var loop = new MainLoop();
    var port = new GSerial.Port();

    port.set_baud(57600);
    port.connected.connect(() => {
        print("Com port connected\n");

        var time = new TimeoutSource(1);
        time.set_callback(() => {
            port.write_string("Hello world\n");
            return true;
        });

        time.attach(loop.get_context());
    });

    port.disconnected.connect(() => {
        print("disconnected\n");
        loop.quit();
    });


    port.on_data.connect((available) => {
        print("received %i bytes\n".printf((int)available));
        var data = port.read_string(available);
		
		for(int i = 0; i < available; i++) {
			stdout.putc((char)data[i]);
		}

        //do something with the bytes here
    });

    port.open("COM3");
    print("starting main loop\n");
    loop.run();
    return 0;
}



