# gserial

gserial is a very small serial / COM port library. It is written in C using GObjects and through GObject introspection, it is useable in many different languages such as Python, Vala, etc

It is compatible with both Windows and Linux. 

# To build

meson --prefix=/usr --libdir=lib builddir

cd builddir

meson compile 
meson install

# Development Status

It's been tested in Linux but not very thoroughly (only 8N1 configuration). It hasn't even been compiled for Windows. 

# Useage

This is a very small and simple library. Refer to the examples for useage.
