# gserial

gserial is a very small library for using serial ports in Gtk+ applications. It is written in C using GObjects and is useable in many different languages such as Python, Vala, etc. 

It is compatible with both Windows and Linux. 

# Dependencies 

Install meson & ninja to build the project
```
sudo pip3 install meson
sudo pip3 install ninja
```
Install gobject introspection dev package
```
apt install libgirepository1.0-dev
```

# To build

```
meson --prefix=/usr builddir
ninja -C builddir
cd builddir && sudo meson install
```

# Useage

This is a very small and simple library. Refer to the examples for useage.
