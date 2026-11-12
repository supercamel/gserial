project('gserial-1.0', 'c', version: '0.3.0', license: 'MIT')

gnome = import('gnome')
pkg = import('pkgconfig')

dependencies = [
  dependency('glib-2.0'),
  dependency('gobject-2.0')
  ]

gserial_sources = ['gserial.c', 'gserial-device.c', 'gserial-enumeration.c']
if host_machine.system() == 'android'
  gserial_sources += ['android/gserial-android.c']
  dependencies += [meson.get_compiler('c').find_library('dl')]
  install_headers('android/gserial-android.h', subdir: 'gserial-1.0')
  install_subdir('android/vendor/usb-serial-for-android/java',
    install_dir: get_option('datadir') / 'sqgi-android')
  install_data('android/java/org/gserial/UsbSerialBridge.java',
    install_dir: get_option('datadir') / 'sqgi-android/java/org/gserial')
  install_data('android/java/com/hoho/android/usbserial/BuildConfig.java',
    install_dir: get_option('datadir') / 'sqgi-android/java/com/hoho/android/usbserial')
  install_data('android/vendor/usb-serial-for-android/LICENSE.txt',
    'android/vendor/usb-serial-for-android/source.json',
    install_dir: get_option('datadir') / 'licenses/gserial/usb-serial-for-android')
  if get_option('android_test_backend')
    install_data('tests/android/TestUsbBackend.java',
      install_dir: get_option('datadir') / 'sqgi-android/java/org/gserial')
  endif
elif host_machine.system() == 'windows'
  dependencies += [meson.get_compiler('c').find_library('setupapi'),
                   meson.get_compiler('c').find_library('advapi32'),
                   meson.get_compiler('c').find_library('cfgmgr32')]
endif
install_data('LICENSE', install_dir: get_option('datadir') / 'licenses/gserial')

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
  requires: ['glib-2.0', 'gobject-2.0'],
  subdirs: 'gserial-1.0'
  )


headers = ['GSerial-1.0.h', 'gserial-device.h']

install_headers(headers, subdir: 'gserial-1.0')

introspection_sources = ['gserial.c', 'gserial-device.c'] + headers

g_ir_compiler = find_program('g-ir-compiler')
gserial_gir = gnome.generate_gir(gseriallib,
  sources: introspection_sources,
  nsversion: '1.0',
  namespace: 'GSerial',
  symbol_prefix: 'gserial_',
  identifier_prefix: 'GSerial',
  export_packages: 'gserial-1.0',
  includes: ['GObject-2.0'],
  install: true,
  install_dir_gir: get_option('datadir') / 'gir-1.0',
  install_dir_typelib: get_option('libdir') / 'girepository-1.0'
  )[0]

gnome.generate_vapi('gserial-1.0',
  install: true,
  sources: gserial_gir
  )

if host_machine.system() == 'linux'
  serial_test = executable('test-gserial', 'tests/test-gserial.c',
    include_directories: '.',
    dependencies: dependencies,
    link_with: gseriallib,
    link_args: ['-lutil'],
  )
  test('gserial', serial_test, timeout: 30)
  device_test = executable('test-devices', 'tests/test-devices.c',
    include_directories: '.', dependencies: dependencies, link_with: gseriallib)
  test('devices', device_test, timeout: 10)
endif

