#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.64])
AC_INIT([TuneL common library],
        [0.0],
        [martin.antony@yahoo.fr],
        [tlcommon])
AC_CONFIG_SRCDIR([src/socket.c])
AC_CONFIG_AUX_DIR([build/aux])
AC_CONFIG_MACRO_DIR([build/m4])
AM_INIT_AUTOMAKE([1.11.1 -Wall -Werror foreign])
AM_PROG_AR
AC_CONFIG_HEADERS([config.h])

AM_SILENT_RULES([yes])

# Checks for programs.
LT_PREREQ([2.2.0])
LT_INIT
AC_PROG_CC
AC_PROG_CXX

# basic setup

#
# revision of the library
# 
# CURRENT:REVISION:AGE
#
# Please remeber to bump library version before releases
#   Remeberer:
# =====================================================================
# |   If you have...                              |   Bump version to |
# =====================================================================
# | Not changed the interface (bug fixes)         | CURRENT:REV+1:AGE |
# ---------------------------------------------------------------------
# | Augmented the interface (new functions)       | CURRENT+1:0:AGE+1 |
# ---------------------------------------------------------------------
# | Broken old interface (e.g. removed functions) | CURRENT+1:0:0     |
# ---------------------------------------------------------------------
TL_COMMON_LTVERSION="0:0:0"
AC_SUBST([TL_COMMON_LTVERSION])

pkgconfigdir="$libdir/pkgconfig"
tl_includedir="$includedir/tunel"
tl_include_commondir="$tl_includedir/common"
AC_SUBST(pkgconfigdir)
AC_SUBST(tl_includedir)
AC_SUBST(tl_include_commondir)

# Checks for libraries.

# AX_REQUIRE_HEADER(HEADER, IF-PRESENT)
# ----------------------------
# Check presence of HEADER and exit if not present, or execute IF-PRESENT if
# prsent
AC_DEFUN([AX_REQUIRE_HEADER],
         [AC_CHECK_HEADERS([$1],
                           [$2],
                           [AC_MSG_ERROR([[$1 not found]])])])

# AX_REQUIRE_LIBRARY(LIBRARY, FUNC, IF-PRESENT)
# ----------------------------
# Check presence of HEADER and exit if not present, or execute IF-PRESENT if
# present
AC_DEFUN([AX_REQUIRE_LIB],
         [AC_CHECK_LIB([$1], [$2],
                       [$3],
                       [AC_MSG_ERROR([[$1 not found]])])])


# PThread
AX_REQUIRE_HEADER([pthread.h])
AX_REQUIRE_LIB([pthread], [pthread_create])

# Wrappers around PKG_CHECK_MODULES() that filling the TL_COMMON_C*FLAGS and
# TL_COMMON_LIBS variables according to the PKG's C*FLAGS and LIBS
AC_DEFUN([AX_PKG_CHECK_MODULES_C],
         [PKG_CHECK_MODULES([$1],
                            [$2])
          TL_COMMON_CFLAGS="${TL_COMMON_CFLAGS} ${$1_CFLAGS}"
          # har har, C++ needs them too.
          TL_COMMON_CXXFLAGS="${TL_COMMON_CXXFLAGS} ${$1_CFLAGS}"
          TL_COMMON_LIBS="${TL_COMMON_LIBS} ${$1_LIBS}"
         ])
AC_DEFUN([AX_PKG_CHECK_MODULES_CXX],
         [PKG_CHECK_MODULES([$1],
                            [$2])
          TL_COMMON_CXXFLAGS="${TL_COMMON_CXXFLAGS} ${$1_CFLAGS}"
          TL_COMMON_LIBS="${TL_COMMON_LIBS} ${$1_LIBS}"
         ])

TL_COMMON_CFLAGS=
TL_COMMON_CXXFLAGS=
TL_COMMON_LIBS=

AX_PKG_CHECK_MODULES_C([LIBCURL],    [libcurl])
AX_PKG_CHECK_MODULES_C([SCEUTILS],   [sceutils])
AX_PKG_CHECK_MODULES_C([SCECORE],    [scecore])
#AX_PKG_CHECK_MODULES_C([GL],         [gl])
AX_PKG_CHECK_MODULES_CXX([BULLET],   [bullet])

AC_SUBST([TL_COMMON_CFLAGS])
AC_SUBST([TL_COMMON_CXXFLAGS])
AC_SUBST([TL_COMMON_LIBS])

# Checks for header files.
AC_CHECK_HEADERS([stdlib.h \
                  string.h \
                  stdarg.h \
                  unistd.h \
                  limits.h \
                  arpa/inet.h \
                  netinet/in.h \
                  sys/socket.h \
                  sys/time.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_C_CONST

# Checks for library functions.
AC_FUNC_STRTOD
AC_CHECK_FUNCS([memset \
                strstr \
                strchr \
                select \
                socket \
                strerror \
                strtol])

# Tuning
AC_ARG_ENABLE([debug],
              AC_HELP_STRING([--enable-debug], [enable debugging [[default=yes]]]),
              [enable_debug="$enableval"],
              [enable_debug="yes"])
AC_MSG_CHECKING([[whether to enable debugging]])
if test "x$enable_debug" = "xyes"; then
  AC_DEFINE([TL_DEBUG], [1], [is debugging enabled])
  # FIXME: please only use TL_DEBUG, not DEBUG
  DEBUG_CPPFLAGS="-DTL_DEBUG -DDEBUG"
  CPPFLAGS="$CPPFLAGS $DEBUG_CPPFLAGS"
  AC_MSG_RESULT([[yes]])
else
  DEBUG_CFLAGS=
  AC_MSG_RESULT([[no]])
fi
AC_SUBST([DEBUG_CFLAGS])

AC_ARG_ENABLE([paranoia],
              AC_HELP_STRING([--enable-paranoia],
                             [enable excessively strict compiler options \
                              (works only with GCC and compatible) [[default=no]]]),
              [enable_paranoia="$enableval"],
              [enable_paranoia="no"])
AC_MSG_CHECKING([[whether to enable paranoiac compiler options]])
if test "x$enable_paranoia" = "xyes"; then
    CFLAGS="$CFLAGS  \
        -Wall -W -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int \
        -Werror-implicit-function-declaration -Wmain -Wparentheses -Wsequence-point \
        -Wreturn-type -Wswitch -Wtrigraphs -Wunused -Wuninitialized -Wunknown-pragmas \
        -Wfloat-equal -Wundef -Wshadow -Wpointer-arith -Wbad-function-cast \
        -Wwrite-strings -Wconversion -Wsign-compare -Waggregate-return \
        -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations \
        -Wmissing-noreturn -Wformat -Wmissing-format-attribute \
        -Wno-deprecated-declarations -Wpacked -Wredundant-decls -Wnested-externs \
        -Winline -Wlong-long -Wunreachable-code -pedantic"
  AC_MSG_RESULT([[yes]])
else
  AC_MSG_RESULT([[no]])
fi

#TL_CHECK_PHY_DEBUG

# output
AC_CONFIG_FILES([Makefile
                 tlcommon.pc
                 src/Makefile
                ])
AC_OUTPUT


echo "------------------------------------------"
echo "Configuration choices:"
echo "* Debugging enabled               : $enable_debug"
echo "* Paranoiac compiler options      : $enable_paranoia"
echo "* Base installation directory     : $prefix"
echo ""
echo "Now type 'make' to build $PACKAGE_NAME"
