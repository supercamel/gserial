local Gtk=import("Gtk","4.0"), Gio=import("Gio"), GLib=import("GLib"), G=import("GSerial","1.0")
local app=Gtk.Application.new("org.gserial.usb.test",Gio.ApplicationFlags.flags_none)
local manager=G.DeviceManager.new(), port=G.Port.new(), stage="initial", received="", signals=0, disconnects=0
local want="abc\x00\xff123\r\n", started=GLib.get_monotonic_time(), label=null
function require(ok, message) { if(!ok)throw message }
function finish(ok, message) {
    stage="done"; manager.cancel_permission(); port.close()
    local result=(ok?"GSERIAL_USB_TEST_PASS":"GSERIAL_USB_TEST_FAIL")+": "+message
    print(result+"\n"); if(label!=null)label.set_text(result)
}
manager.connect("permission-result",function(path,granted,message){try {
    if(stage=="denied") {
        require(!granted && message.len()>0,"denial result")
        stage="grant";manager.request_permission("gserial-test://loopback")
    } else if(stage=="grant") {
        require(granted,"permission grant")
        local device=manager.lookup(path);require(device.get_has_permission(),"permission metadata")
        port.set_baud(57600);port.set_timeout(100);port.open_full(device.get_path())
        port.set_control_lines(false,false);stage="receive"
        local offset=0
        while(offset<want.len()) {local count=port.write_bytes(want.slice(offset));require(count>0 && count<=3,"partial write count");offset+=count}
    } else throw "unexpected permission callback"
}catch(e){finish(false,e.tostring())}})
port.connect("on-data",function(count){try {
    signals++;received+=port.read_bytes(count)
    if(received.len()>=want.len()) {
        require(received==want,"binary payload")
        // Close/reopen from a data callback exercises both native and Java lifetimes.
        port.close();port.open_full("gserial-test://loopback");stage="detach"
        require(port.write_bytes("\x04")==1,"detach trigger")
    }
}catch(e){finish(false,e.tostring())}})
port.connect("disconnected",function(){disconnects++;if(stage=="detach")finish(true,"enumeration, permission denial/grant, binary partial I/O, control lines, callback reopen, detach")})
app.connect("activate",function(){try {
    local win=Gtk.ApplicationWindow.new(app);win.set_title("GSerial USB regression");win.set_default_size(640,300)
    label=Gtk.Label.new("Running JNI/SQGI regression (simulated adapter)");label.set_wrap(true);win.set_child(label);win.present()
    local devices=manager.list_devices();local found=false
    foreach(device in devices)if(device.get_path()=="gserial-test://loopback") {found=true;require(device.get_vendor_id()==1027 && device.get_product_id()==24577,"USB IDs")}
    require(found,"test backend was packaged")
    local rejected=false;try{port.open_full("gserial-test://loopback")}catch(e){rejected=true};require(rejected,"permission required")
    manager.request_permission("gserial-test://loopback");manager.cancel_permission()
    stage="denied";manager.request_permission("gserial-test://denied")
    sqgi.timeout_add(100,function(){if(stage=="done")return false;if(GLib.get_monotonic_time()-started>10000000){finish(false,"test timeout");return false}return true})
}catch(e){finish(false,e.tostring())}})
app.run(["gserial-usb-test"])
