project('gserial-1.0', 'c', 'vala', 
	version: '0.1.0',
	license: 'MIT')

gnome = import('gnome')
pkg = import('pkgconfig')

dependencies = [
	dependency('glib-2.0'),
	dependency('gobject-2.0')
]

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : 'libgserial',
		version : '1.0',
		name : 'libgserial',
		filebase : 'libgserial',
		description : 'A com port library for vala/gtk')

gserial_sources= [
	'gserial.vala',
	'gserial_os.c',
	'gserial_os.vapi'
]

gseriallib= shared_library('gserial-1.0',
		gserial_sources,
		vala_header: 'gserial.h',
		vala_vapi: 'gserial-1.0.vapi',
		vala_gir: 'gserial-1.0.gir',
		include_directories : '.',
		dependencies : dependencies,
		install : true,
		install_dir: [true, true, true, true])

pkg.generate(gseriallib)


g_ir_compiler = find_program('g-ir-compiler')
custom_target('gserial typelib', command: [g_ir_compiler, '--output', '@OUTPUT@', meson.current_build_dir() + '/gserial-1.0.gir'],
              output: 'gserial-1.0.typelib',
              depends: gseriallib,
              install: true,
              install_dir: get_option('libdir') / 'girepository-1.0')
