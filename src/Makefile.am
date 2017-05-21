## Process this file with automake to produce Makefile.in
lib_LTLIBRARIES = libgserial-1.0.la

GSERIAL_CFLAGS =          \
	$(GLIB_CFLAGS)          \
	$(GOBJECT_CFLAGS)

GSERIAL_LIBS =            \
    $(GLIB_LIBS)            \
    $(GOBJECT_LIBS)

## Shared library
libgserial_1_0_la_CFLAGS =    \
    $(WARN_CFLAGS)          \
    $(DISABLE_DEPRECATED)   \
    $(GSERIAL_CFLAGS)     \
	-DG_LOG_DOMAIN=\"gserial\"

if OS_WIN32
no_undefined = -no-undefined
endif

libgserial_1_0_la_LDFLAGS = \
	-version-info $(LT_VERSION_INFO)    \
	$(no_undefined)

libgserial_1_0_la_LIBADD =    \
    $(GSERIAL_LIBS)

sources_public_h =          \
    gserial.h

sources_c =                 \
    gserial.c

libgserial_1_0_la_SOURCES =   \
	$(sources_public_h)     \
	$(sources_c)

libgserialincdir = $(includedir)/gserial-1.0
libgserialinc_HEADERS = $(sources_public_h)

## Misc
CLEANFILES =

## GObject Introspection Support
-include $(INTROSPECTION_MAKEFILE)
INTROSPECTION_GIRS =
INTROSPECTION_SCANNER_ARGS = $(INTROSPECTION_FLAGS) --add-include-path=$(srcdir) --add-include-path=$(builddir) --warn-all --warn-error
INTROSPECTION_COMPILER_ARGS = --includedir=$(srcdir) --includedir=$(builddir)

if HAVE_INTROSPECTION
introspection_sources = $(sources_c) $(sources_public_h)

gserial-1.0.gir: libgserial-1.0.la
gserial_1_0_gir_INCLUDES = GObject-2.0 GLib-2.0
gserial_1_0_gir_CFLAGS = $(GSERIAL_CFLAGS) -I$(srcdir) -I$(builddir)
gserial_1_0_gir_LIBS = libgserial-1.0.la
gserial_1_0_gir_FILES = $(introspection_sources)
gserial_1_0_gir_SCANNERFLAGS = --identifier-prefix=gserial --c-include "gserial.h"
INTROSPECTION_GIRS += gserial-1.0.gir

girdir = $(datadir)/gir-1.0
gir_DATA = $(INTROSPECTION_GIRS)

typelibdir = $(libdir)/girepository-1.0
typelib_DATA = $(INTROSPECTION_GIRS:.gir=.typelib)

CLEANFILES += $(gir_DATA) $(typelib_DATA)
endif

if ENABLE_VAPIGEN
-include $(VAPIGEN_MAKEFILE)

gserial-1.0.vapi: gserial-1.0.gir

VAPIGEN_VAPIS = gserial-1.0.vapi

gserial_1_0_vapi_METADATADIRS = $(srcdir)
gserial_1_0_vapi_FILES = gserial-1.0.gir

vapidir = $(datadir)/vala/vapi

endif
