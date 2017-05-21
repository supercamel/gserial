## Process this file with automake to produce Makefile.in

SUBDIRS = src

ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}

doc_DATA =                                      \
	README                                      \
	COPYING                                     \
	AUTHORS                                     \
	NEWS

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = gserial-1.0.pc


release: dist
	scp @PACKAGE@-@VERSION@.tar.gz samuel.cowen@camelsoftware.com:/var/www/camelsoftware.com/gserial/

