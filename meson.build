project('gserial-1.0', 'c', version: '0.2.0', license: 'MIT')

gnome = import('gnome')
pkg = import('pkgconfig')

dependencies = [
  dependency('glib-2.0'),
  dependency('gobject-2.0')
  ]

gserial_sources = ['gserial.c']

gseriallib = shared_library('gserial-1.0',
  gserial_sources,
  vala_vapi: 'gserial-1.0.vapi',
  vala_gir: 'gserial-1.0.gir',
  include_directories: './',
  dependencies: dependencies,
  install: true
  )

pkg.generate(gseriallib,
  version: '1.0',
  name: 'gserial',
  filebase: 'gserial-1.0',
  description: 'A simple serial port library',
  subdirs: 'gserial-1.0'
  )


headers = ['GSerial-1.0.h']

install_headers(headers, subdir: 'gserial-1.0')

introspection_sources = ['gserial.c', 'GSerial-1.0.h']

g_ir_compiler = find_program('g-ir-compiler')
gserial_gir = gnome.generate_gir(gseriallib,
  sources: introspection_sources,
  nsversion: '1.0',
  namespace: 'GSerial',
  symbol_prefix: 'gserial_',
  identifier_prefix: 'GSerial',
  export_packages: 'gserial',
  includes: ['GObject-2.0'],
  install: true,
  install_dir_gir: get_option('libdir') / 'girepository-1.0',
  install_dir_typelib: get_option('libdir') / 'girepository-1.0'
  )[0]

gnome.generate_vapi('gserial-1.0',
  install: true,
  sources: gserial_gir
  )

