#!/usr/bin/env sqgi

local default_device = "/dev/ttyUSB0"
local default_baud = 9600

foreach (arg in vargv) {
    if (arg == "--help" || arg == "-h") {
        print("Usage: sqgi examples/sqgi/gtk_terminal.nut [--device=PATH] [--baud=9600]\n")
        return 0
    } else if (arg.find("--device=") == 0) {
        default_device = arg.slice(9)
    } else if (arg.find("--baud=") == 0) {
        default_baud = arg.slice(7).tointeger()
    }
}

local Gio = import("Gio")
local Gtk = import("Gtk", "4.0")
local GSerial = import("GSerial", "1.0")

local app = Gtk.Application.new("org.gserial.sqgi.terminal", Gio.ApplicationFlags.flags_none)
local state = {
    port = null,
    connected = false,
    transcript = ""
}
local W = {}

function set_status(text) {
    W.status.set_text(text)
}

function append_text(text) {
    state.transcript += text
    W.buffer.set_text(state.transcript, -1)
}

function disconnect_port() {
    if (state.port != null && state.port.is_open()) {
        state.port.close()
    } else {
        state.connected = false
        W.connect_button.set_label("Connect")
        W.input.set_sensitive(false)
        set_status("Disconnected")
    }
}

function connect_port() {
    local device = W.device_entry.get_text()
    local baud = W.baud_entry.get_text().tointeger()

    local port = GSerial.Port.new()
    port.set_baud(baud)
    port.set_timeout(100)

    port.connect("connected", function() {
        state.connected = true
        W.connect_button.set_label("Disconnect")
        W.input.set_sensitive(true)
        set_status("Connected")
        append_text("[connected to " + device + " at " + baud + "]\n")
    })

    port.connect("disconnected", function() {
        state.connected = false
        W.connect_button.set_label("Connect")
        W.input.set_sensitive(false)
        set_status("Disconnected")
        append_text("\n[disconnected]\n")
    })

    port.connect("on_data", function(available) {
        local data = port.read_string(available)
        if (data != null) append_text(data)
    })

    state.port = port
    if (!port.open(device)) {
        state.port = null
        set_status("Failed to open " + device)
    }
}

app.connect("activate", function() {
    local win = Gtk.ApplicationWindow.new(app)
    win.set_title("GSerial SQGI Terminal")
    win.set_default_size(720, 520)

    local root = Gtk.Box.new(Gtk.Orientation.vertical, 6)
    root.set_margin_top(6)
    root.set_margin_bottom(6)
    root.set_margin_start(6)
    root.set_margin_end(6)
    win.set_child(root)

    local controls = Gtk.Box.new(Gtk.Orientation.horizontal, 6)
    root.append(controls)

    W.device_entry <- Gtk.Entry.new()
    W.device_entry.set_placeholder_text("/dev/ttyUSB0 or COM3")
    W.device_entry.set_text(default_device)
    W.device_entry.set_hexpand(true)
    controls.append(W.device_entry)

    W.baud_entry <- Gtk.Entry.new()
    W.baud_entry.set_placeholder_text("Baud")
    W.baud_entry.set_text(default_baud.tostring())
    controls.append(W.baud_entry)

    W.connect_button <- Gtk.Button.new_with_label("Connect")
    W.connect_button.connect("clicked", function() {
        if (state.connected) disconnect_port()
        else connect_port()
    })
    controls.append(W.connect_button)

    local textview = Gtk.TextView.new()
    textview.set_monospace(true)
    textview.set_editable(false)
    textview.set_wrap_mode(Gtk.WrapMode.word_char)
    W.buffer <- textview.get_buffer()

    local scroll = Gtk.ScrolledWindow.new()
    scroll.set_child(textview)
    scroll.set_vexpand(true)
    root.append(scroll)

    W.input <- Gtk.Entry.new()
    W.input.set_placeholder_text("Type command and press Enter")
    W.input.set_sensitive(false)
    W.input.connect("activate", function() {
        local text = W.input.get_text()
        if (text != "" && state.port != null && state.port.is_open()) {
            state.port.write_string(text + "\n")
            W.input.set_text("")
        }
    })
    root.append(W.input)

    W.status <- Gtk.Label.new("Disconnected")
    root.append(W.status)

    win.present()
})

local argv = ["gserial-sqgi-terminal"]
foreach (arg in vargv) argv.append(arg)
app.run(argv.len(), argv)
