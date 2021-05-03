
project('libgserial', 'c',
		version : '1.0.0',
		license : 'MIT')

gnome = import('gnome')
pkg = import('pkgconfig')

dependencies = [
	dependency('glib-2.0'),
	dependency('gobject-2.0'),
]

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : 'libgserial',
		version : '1.0',
		name : 'libgserial',
		filebase : 'libgserial',
		description : 'A serial port library for gtk applications')

install_headers('gserial.h')

gserial_sources = ['gserial.c']
gseriallib = shared_library('gserial',
		gserial_sources,
		include_directories : '.',
		dependencies : dependencies,
		vala_vapi: 'gserial-1.0.vapi',
		install : true)

pkg.generate(gseriallib)

gir = gnome.generate_gir(gseriallib, 
		sources: ['gserial.c', 'gserial.h'],
		namespace: 'gserial',
		nsversion: '1.0',
		identifier_prefix: 'gserial',
		symbol_prefix: 'gserial',
		export_packages: 'libgserial',
		includes: [ 'GObject-2.0', 'GLib-2.0' ],
		header: 'gserial.h',
		install: true)

