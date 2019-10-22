AC_PREREQ([2.63])
AC_INIT([gserial],
        [1.0])

AM_INIT_AUTOMAKE([1.11 foreign subdir-objects parallel-tests])

AC_CONFIG_HEADERS([config.h])
AC_CONFIG_SRCDIR([src/gserial.h])
AC_CONFIG_MACRO_DIR([m4])

AC_PREFIX_DEFAULT('/usr/')

AM_PROG_CC_C_O

AC_LIBTOOL_WIN32_DLL
AM_PROG_LIBTOOL

AM_SILENT_RULES([yes])

# Library dependencies
PKG_CHECK_MODULES(GLIB,     [glib-2.0 >= 2.16])
PKG_CHECK_MODULES(GOBJECT,  [gobject-2.0 >= 2.0])

AC_MSG_CHECKING([for Win32])
case "$host" in
  *-*-mingw*)
    os_win32=yes
    ;;
  *)
    os_win32=no
    ;;
esac
AC_MSG_RESULT([$os_win32])
AM_CONDITIONAL(OS_WIN32, [test $os_win32 = yes])

# Before making a release, the LT_VERSION_INFO string should be modified.
# The string is of the form C:R:A.
# - If interfaces have been changed or added, but binary compatibility has
#   been preserved, change to C+1:0:A+1
# - If binary compatibility has been broken (eg removed or changed interfaces)
#   change to C+1:0:0
# - If the interface is the same as the previous version, change to C:R+1:A
LT_VERSION_INFO=1:0:0
AC_SUBST(LT_VERSION_INFO)

GOBJECT_INTROSPECTION_REQS=1.40.0
GOBJECT_INTROSPECTION_CHECK([$GOBJECT_INTROSPECTION_REQS])

VAPIGEN_CHECK()

GNOME_COMPILE_WARNINGS([maximum])

AC_OUTPUT([
gserial-1.0.pc
Makefile
src/Makefile
])

echo
echo $PACKAGE v$VERSION
echo
echo Prefix............... : $prefix
echo Introspection support : ${found_introspection}
echo

