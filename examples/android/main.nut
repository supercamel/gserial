local Gtk = import("Gtk", "4.0"), Gio = import("Gio"), GSerial = import("GSerial", "1.0")
local manager = GSerial.DeviceManager.new(), port = GSerial.Port.new()
local app = Gtk.Application.new("org.gserial.usb.demo", Gio.ApplicationFlags.flags_none)
local choices = null, status = null, output = null, devices = [], pending = null
function refresh() {
    choices.remove_all(); devices = manager.list_devices()
    foreach (device in devices) choices.append(device.get_path(), device.get_name())
    if (devices.len()) choices.set_active(0)
    status.set_text(devices.len() ? "Select an adapter, then connect" : "No USB serial adapters attached")
}
function open_port(path) {
    try {
        local device = manager.lookup(path)
        port.set_baud(57600); port.set_timeout(100)
        port.open_full(device.get_path())
        status.set_text("Connected · " + device.get_name())
    } catch (error) { status.set_text(error.tostring()) }
}
manager.connect("permission-result", function(path, granted, message) {
    pending = null
    if (granted) open_port(path); else status.set_text(message)
})
port.connect("on-data", function(count) {
    local text = port.read_string(count)
    if (text != null) output.set_text(text)
})
port.connect("disconnected", function() { status.set_text("Disconnected") })
app.connect("activate", function() {
    local win = Gtk.ApplicationWindow.new(app); win.set_title("GSerial USB devices"); win.set_default_size(640,420)
    local box = Gtk.Box.new(Gtk.Orientation.vertical,12)
    box.set_margin_start(20); box.set_margin_end(20); box.set_margin_top(20); box.set_margin_bottom(20)
    choices = Gtk.ComboBoxText.new(); box.append(choices)
    local scan = Gtk.Button.new_with_label("Refresh devices"); scan.connect("clicked", function() { try { refresh() } catch(e) { status.set_text(e.tostring()) } }); box.append(scan)
    local connect = Gtk.Button.new_with_label("Connect · 57600 baud")
    connect.connect("clicked", function() {
        try {
            if (pending != null) return
            local path = choices.get_active_id(); if (path == null) return
            port.close(); pending = path; manager.request_permission(path); status.set_text("Waiting for USB permission")
        } catch (error) { pending = null; status.set_text(error.tostring()) }
    }); box.append(connect)
    local disconnect = Gtk.Button.new_with_label("Disconnect")
    disconnect.connect("clicked", function() { manager.cancel_permission(); pending = null; port.close() }); box.append(disconnect)
    local entry = Gtk.Entry.new(); entry.set_placeholder_text("Text to send (Enter)")
    entry.connect("activate", function() {
        if (!port.is_open()) return
        local data = entry.get_text() + "\n", offset = 0
        while (offset < data.len()) { local count = port.write_string(data.slice(offset)); if (count <= 0) break; offset += count }
        if (offset == data.len()) entry.set_text(""); else status.set_text("Partial write; remaining bytes were not sent")
    }); box.append(entry)
    status = Gtk.Label.new(""); status.set_wrap(true); box.append(status)
    output = Gtk.Label.new(""); output.set_wrap(true); box.append(output)
    win.set_child(box); win.present()
    try { refresh() } catch (error) { status.set_text(error.tostring()) }
})
app.connect("shutdown", function() { manager.cancel_permission(); port.close() })
app.run(["gserial-usb"])
