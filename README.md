# gserial 

gserial is a serial / COM port library for the GObject ecosystem. That means it has automatic bindings for python, gjs, vala, C and probably other languages as well, and an API that is consistent with the style of Gtk. 

# To build

meson --prefix=/usr --libdir=lib builddir 

cd builddir

meson compile
meson install

# Useage

This is a very small and simple library. Refer to the examples for useage. 


