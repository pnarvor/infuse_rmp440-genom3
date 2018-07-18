dnl Autoconf file for building rmp440 codels library.

AC_PREREQ(2.59)

AC_INIT([rmp440-genom3],[0.1.99],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acrmp440.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CC
AC_PROG_CXX


dnl External packages
PKG_CHECK_MODULES(requires, [
  genom3 >= 2.99.4
  gyro-libs >= 3.0.99.0
  openrobots-idl >= 1.1
  infuse_types-genom3 >= 0.1
  rmp440-libs >= 0.5, gyro-libs, felib, MTI-clients, cdff_types
])

AC_PATH_PROG(GENOM3, [genom3], AC_MSG_ERROR([genom3 tool not found], 2))


dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/rmp440.gen])

dnl Doc
AM_MISSING_PROG([ASCIIDOCTOR], [asciidoctor])

dnl Output
AC_CONFIG_FILES([
	rmp440-genom3.pc
	rmp440-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
