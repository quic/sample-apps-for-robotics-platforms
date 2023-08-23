
#                                               -*- Autoconf -*-
# configure.ac -- Autoconf script for sensors
#
# Process this file with autoconf to produce a configure script
# Requires autoconf tool later than 2.61
AC_PREREQ(2.61)

# Initialize the sensors package version 1.0.0
AC_INIT([sensors-client], 1.0.0)

# Does not strictly follow GNU Coding standards
AM_INIT_AUTOMAKE([foreign])

# Disables auto rebuilding of configure, Makefile.ins
AM_MAINTAINER_MODE

# Verifies the --srcdir is correct by checking for the path
AC_CONFIG_SRCDIR([src])

# defines some macros variable to be included by source
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET

# Checks for libraries

AC_ARG_WITH(sanitized-headers,
        AS_HELP_STRING([--with-sanitized-headers=DIR],
                [Specify the location of the sanitized Linux headers]),
        [CPPFLAGS="$CPPFLAGS -idirafter $withval"])

AC_ARG_WITH([glib],
       AC_HELP_STRING([--with-glib],
          [enable glib, building HLOS systems which use glib]))

if (test "x${with_glib}" = "xyes"); then
         GLIB_CFLAGS="$GLIB_CFLAGS $GTHREAD_CFLAGS"
         GLIB_LIBS="$GLIB_LIBS $GTHREAD_LIBS"

         AC_SUBST(GLIB_CFLAGS)
         AC_SUBST(GLIB_LIBS)
fi

AM_CONDITIONAL(USE_GLIB, test "x${with_glib}" = "xyes")

AC_ARG_WITH([common_includes],
      AC_HELP_STRING([--with-common-includes=@<:@dir@:>@],
         [Specify the location of the common headers]),
            [common_incdir=$withval],
               with_common_includes=no)

if test "x$with_common_includes" != "xno"; then
   CFLAGS="${CFLAGS} -I${common_incdir}"
fi

AC_SUBST([CFLAGS])
AC_SUBST([CPPFLAGS])
AC_SUBST([CC])


AC_CONFIG_FILES([      \
      Makefile
      ])
AC_OUTPUT
