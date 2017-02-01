dnl Autoconf file for building rmp440 codels library.

AC_PREREQ(2.59)

AC_INIT([rmp440-genom3],[0.1],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acrmp440.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT
AC_PROG_CC


dnl External packages
PKG_CHECK_MODULES(requires, [
  genom3 >= 2.99.4
  gyro-libs >= 3.0.99.0
  rmp440-libs, gyro-libs, felib
])

AC_PATH_PROG(GENOM3, [genom3], AC_MSG_ERROR([genom3 tool not found], 2))


dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/rmp440.gen])


dnl Output
AC_CONFIG_FILES([
	rmp440-genom3.pc
	rmp440-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES