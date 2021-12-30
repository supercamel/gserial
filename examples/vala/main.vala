
public static int main() {
    var loop = new MainLoop();
    var port = new GSerial.Port();

    port.set_baud(57600);
    port.connected.connect(() => {
        print("Com port connected");

        var time = new TimeoutSource(100);
        time.set_callback(() => {
            if(port.bytes_available() > 0) {
                var bytes = port.read_bytes(port.bytes_available());
                // do something with bytes
            }
            return true;
        });

        time.attach(loop.get_context());
    });

    port.disconnected.connect(() => {
        loop.quit();
    });

    port.open("/dev/ttyUSB0");
    print("starting main loop");
    loop.run();
    return 0;
}



