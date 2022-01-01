# gserial

gserial is a very small library for using serial ports in Gtk+ applications. It is written in C using GObjects and is useable in many different languages such as Python, Vala, etc. 

It is compatible with both Windows and Linux. 

# To build

meson --prefix=/usr --libdir=lib builddir

cd builddir

meson compile 
meson install

# Development Status

It's been tested in Linux but not very thoroughly (only 8N1 configuration). I have not compiled for Windows yet (need a tester). 

# Useage

This is a very small and simple library. Refer to the examples for useage.
