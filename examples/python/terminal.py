#!/usr/bin/env python3

import gi
gi.require_version("Gtk", "4.0")
gi.require_version("GSerial", "1.0")
from gi.repository import Gtk, GSerial, GLib

class SerialTerminal(Gtk.Application):
    def __init__(self):
        super().__init__(application_id="com.example.GSerialTerminal")
        self.port = None
        self.connected = False

    def do_activate(self):
        self.window = Gtk.ApplicationWindow(application=self)
        self.window.set_title("GSerial Terminal")
        self.window.set_default_size(700, 500)

        self.vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6, margin_top=6, margin_bottom=6, margin_start=6, margin_end=6)
        self.window.set_child(self.vbox)

        # Port controls
        controls = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.vbox.append(controls)

        self.port_entry = Gtk.Entry()
        self.port_entry.set_placeholder_text("/dev/ttyUSB0 or COM3")
        self.port_entry.set_text("/dev/ttyUSB0")  # Default suggestion
        controls.append(self.port_entry)

        self.baud_entry = Gtk.Entry()
        self.baud_entry.set_placeholder_text("Baud (e.g., 9600)")
        self.baud_entry.set_text("9600")
        controls.append(self.baud_entry)

        self.connect_button = Gtk.Button(label="Connect")
        self.connect_button.connect("clicked", self.on_connect_clicked)
        controls.append(self.connect_button)

        # Text output
        self.textview = Gtk.TextView(monospace=True, editable=False, wrap_mode=Gtk.WrapMode.WORD_CHAR)
        self.textbuf = self.textview.get_buffer()
        self.scroll = Gtk.ScrolledWindow()
        self.scroll.set_child(self.textview)
        self.scroll.set_vexpand(True)
        self.vbox.append(self.scroll)

        # Entry for input
        self.entry = Gtk.Entry()
        self.entry.set_placeholder_text("Type command and press Enter")
        self.entry.set_sensitive(False)
        self.entry.connect("activate", self.send_input)
        self.vbox.append(self.entry)

        # Status label
        self.status = Gtk.Label(label="Disconnected")
        self.vbox.append(self.status)

        self.window.present()

    def on_connect_clicked(self, button):
        if self.connected:
            self.disconnect_port()
        else:
            self.connect_port(self.port_entry.get_text(), self.baud_entry.get_text())

    def connect_port(self, device_path, baud_text):
        try:
            baud = int(baud_text)
        except ValueError:
            self.status.set_text("Invalid baud rate")
            return

        self.port = GSerial.Port.new()
        self.port.baud = baud
        self.port.timeout = 100

        self.port.connect("on_data", self.on_data)
        self.port.connect("connected", self.on_connected)
        self.port.connect("disconnected", self.on_disconnected)

        if not self.port.open(device_path):
            self.status.set_text(f"Failed to open {device_path}")
            self.port = None

    def disconnect_port(self):
        self.connected = False
        self.connect_button.set_label("Connect")
        self.entry.set_sensitive(False)
        self.status.set_text("Disconnected")

    def on_connected(self, port):
        self.connected = True
        self.connect_button.set_label("Disconnect")
        self.entry.set_sensitive(True)
        self.status.set_text("Connected")

    def on_disconnected(self, port):
        self.disconnect_port()

    def on_data(self, port, available):
        data = port.read_string(available)
        end = self.textbuf.get_end_iter()
        self.textbuf.insert(end, data)

    def send_input(self, entry):
        text = entry.get_text()
        if text and self.port and self.port.is_open():
            self.port.write_string(text + "\n")
            entry.set_text("")

app = SerialTerminal()
app.run()
