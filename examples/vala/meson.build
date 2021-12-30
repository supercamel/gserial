project('gserial_test', 'vala')

dependencies = [
  dependency('glib-2.0'),
  dependency('gobject-2.0'),
  dependency('gserial-1.0'),
  meson.get_compiler('c').find_library('m', required: false)
]

sources = files('main.vala')

executable('gserial_test', sources, dependencies: dependencies)


