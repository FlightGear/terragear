dnl Process this file with aclocal ; automake -a ; autoconf to produce a 
dnl working configure script.

AC_INIT
AC_CONFIG_SRCDIR([src/Lib/DEM/dem.cxx])

# Require at least automake 2.52
AC_PREREQ(2.59)

dnl Initialize the automake stuff
AM_INIT_AUTOMAKE(TerraGear, 0.9.8-wcsp)

dnl Checks for programs.
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_CXX
AC_PROG_RANLIB
AC_PROG_INSTALL
AC_PROG_LN_S

# specify the simgear location
AC_ARG_WITH(simgear, [  --with-simgear=PREFIX   Specify the prefix path to simgear])

if test "x$with_simgear" != "x" ; then
    echo "SimGear prefix path is $with_simgear"
    EXTRA_DIRS="${EXTRA_DIRS} $with_simgear"
fi

dnl set the $host variable based on local machine/os
AC_CANONICAL_HOST

case "${host}" in
*-*-irix*)
    AC_DEFINE([_SOCKLEN_T], 1, [Prevent a nasty bug in IRIX 6.5.17])
    if test "$CXX" = "CC"; then
        AR="CC -ar"
        ARFLAGS="-o"
        CXXFLAGS="$CXXFLAGS -I$with_simgear/include/simgear/compatibility"
    else
        AR="ar"
        ARFLAGS="cru"
    fi
    ;;
*)
    AR="ar"
    ARFLAGS="cru"
    ;;
esac
AC_SUBST(AR)
AC_SUBST(ARFLAGS)

dnl Add gts/glib includes, this will probably need to be made more
dnl flexible in the future.
# AC_CHECK_PROG(GLIB, glib-config, yes, no)
# AC_CHECK_PROG(GTS, gts-config, yes, no)
# 
# if test "$GLIB" = "no"; then
#   echo
#   echo "Unable to find glib-config."
#   echo
#   echo "This program is needed to determine the compiler flags needed for"
#   echo "the glib library. Please make sure this linrary is installed and"
#   echo "the program is in the search path."
#   echo
#   echo "Please read README.gts" for more details.
#   echo
#   exit 1
# fi
# 
# if test "$GTS" = "no"; then
#   echo
#   echo "Unable to find gts-config."
#   echo
#   echo "This program is needed to determine the compiler flags needed for"
#   echo "the gts library. Please make sure this linrary is installed and"
#   echo "the program is in the search path."
#   echo
#   echo "Please read README.gts" for more details.
#   echo
#   exit 1
# 
# fi

# SUPPORT_FLAGS="`gts-config --cflags` `glib-config --cflags`"
# CPPFLAGS="$CPPFLAGS $SUPPORT_FLAGS"

dnl Specify if we want logging (testing build) or not (release build)
# set logging default value
# with_logging=yes
AC_ARG_WITH(logging, [  --with-logging          Include logging output (default)])
if test "x$with_logging" = "xno" ; then
    AC_DEFINE([FG_NDEBUG], 1, [Define for no logging output])
fi


dnl Let the Win32 user specify if they want to build with the SGI
dnl opengl.dll as opposed to the more standard openg32.dll
AC_ARG_WITH(sgi-opengl, [  --with-sgi-opengl       Build against SGI's opengl.dll glu.dll and glut.dll])

# specify gdal support
AC_DEFINE([HAVE_OGR],[],[Use OGR])

AC_ARG_WITH(gdal,
            AC_HELP_STRING([--with-gdal[=path/gdal-config]],
                           [enable GDAL/OGR support (gdal-config with path,
                            e.g. '--with-gdal=/usr/local/bin/gdal-config',
                            default: enabled if gdal-config is in PATH)]))

AC_ARG_ENABLE(ogrdecode,
              AC_HELP_STRING([--disable-ogrdecode],
                             [enable ogrdecode tool (default depending on OGR availability)]))

if test "$with_gdal" != "no" ; then
  if test "`basename xx/$with_gdal`" = "gdal-config" ; then
    # gdal-config was specified
    GDAL_CONFIG="$with_gdal"
    with_gdal="yes"
  else
    # gdal-config was not specified, but GDAL/OGR wasn't disabled
    AC_PATH_PROG(GDAL_CONFIG, gdal-config, no)
    if test "$GDAL_CONFIG" = "no" ; then
      with_gdal="no"
    fi
  fi

  GDAL_LIBS=
  GDAL_CFLAGS=
  USE_GDAL=
  USE_OGR=
  
  AC_MSG_CHECKING([for GDAL...])
  if test -x "$GDAL_CONFIG"; then
    AC_MSG_RESULT(yes)
    GDAL_LIBS=`"$GDAL_CONFIG" --libs`
    GDAL_DEP_LIBS=`"$GDAL_CONFIG" --dep-libs`
    GDAL_CFLAGS=`"$GDAL_CONFIG" --cflags`
    have_gdal=yes
    AC_MSG_CHECKING([for OGR...])
    if test `"$GDAL_CONFIG" --ogr-enabled` = "yes" ; then
      AC_MSG_RESULT(yes)
      have_ogr=yes
    else
      AC_MSG_RESULT(no)
    fi
  else
    AC_MSG_RESULT(no)
  fi

  GDAL_LIBS="$GDAL_LIBS $GDAL_DEP_LIBS"

  AC_SUBST(GDAL_LIBS)
  AC_SUBST(GDAL_CFLAGS)
fi

if test x$have_ogr != xyes -a x$enable_ogrdecode = xyes; then
  AC_MSG_ERROR([ogrdecode was forcefully enabled but OGR was not found.
                Please make sure that you have GDAL installed with OGR enabled
                and gdal-config in your PATH resp. the path to gdal-config provided
                using --with-gdal.])
fi

AM_CONDITIONAL(WANT_OGRDECODE, test x$have_ogr = xyes -a x$enable_ogrdecode != xno)

dnl Check for MS Windows environment
AC_CHECK_HEADER(windows.h)

dnl extra library and include directories
EXTRA_DIRS="/usr/local/plib /usr/X11R6"

if test -d /opt/X11R6 ; then
     EXTRA_DIRS="$EXTRA_DIRS /opt/X11R6"
fi

if test "x$ac_cv_header_windows_h" = "xyes" ; then
    EXTRA_DIRS="${EXTRA_DIRS} `pwd`/Win32"
fi
wi_EXTRA_DIRS(no, ${EXTRA_DIRS})

dnl Using AM_CONDITIONAL is a step out of the protected little 
dnl automake fold so it is potentially dangerous.  But, we are
dnl beginning to run into cases where the standard checks are not
dnl enough.  AM_CONDITIONALS are then referenced to conditionally
dnl build a Makefile.in from a Makefile.am which lets us define custom
dnl includes, compile alternative source files, etc.

dnl Check for X11 (fancy)
AC_PATH_XTRA

dnl Checks for libraries.

null_LIBS="$LIBS"
LIBS=""

dnl search for network related libraries
AC_SEARCH_LIBS(inet_addr, xnet)
AC_SEARCH_LIBS(socket, socket)
AC_SEARCH_LIBS(main, nsl)

network_LIBS="$LIBS"
LIBS=""

dnl check for some default libraries
AC_SEARCH_LIBS(cos, m)

base_LIBS="$LIBS"
# support_LIBS="`gts-config --libs` `glib-config --libs`"

AC_SEARCH_LIBS(XCreateWindow, X11)
AC_SEARCH_LIBS(XShmCreateImage, Xext)
AC_SEARCH_LIBS(XGetExtensionVersion, Xi)
AC_SEARCH_LIBS(IceOpenConnection, ICE)
AC_SEARCH_LIBS(SmcOpenConnection, SM)
AC_SEARCH_LIBS(XtMalloc, Xt)
AC_SEARCH_LIBS(XmuLookupStandardColormap, Xmu)

dnl check for OpenGL related libraries

if test "x$ac_cv_header_windows_h" != "xyes" ; then
    dnl Reasonable stuff for non-windoze variants ... :-)

    AC_SEARCH_LIBS(glNewList, [ GL GLcore MesaGL ])
    if test "x$ac_cv_search_glNewList" = "x-lGLcore"; then
        dnl if GLcore found, then also check for GL
        AC_SEARCH_LIBS(glXCreateContext, GL)
    fi

    dnl if using mesa, check for xmesa.h
    if test "x$ac_cv_lib_MesaGL_glNewList" = "xyes" ; then
	AC_CHECK_HEADER(GL/xmesa.h)
	if test "x$ac_cv_header_GL_xmesa_h" = "xyes"; then
	    AC_DEFINE([XMESA], 1, [Define for Mesa FX mode])
            AC_DEFINE([FX], 1, [Define for Mesa FX mode])
        fi
    fi

    AC_SEARCH_LIBS(gluLookAt, [ GLU MesaGLU ])
    AC_SEARCH_LIBS(glutGetModifiers, [ glut freeglut ])

else
    dnl Win32 is a little wierd because it has to try to handle the various
    dnl winbloze-isms.  We'll just do this manually for now.

    echo Win32 specific hacks...
    AC_DEFINE([WIN32], 1, [Define if building on a Win32 platform])
    AC_DEFINE([NOMINMAX], 1, [Define for Win32 platforms])

    dnl just define these to true and hope for the best
    ac_cv_lib_glut_glutGetModifiers="yes"
    ac_cv_lib_glut_glutGameModeString="yes"

    if test "x$with_sgi_opengl" = "xyes" ; then
        echo "Building with glut.dll, glu.dll, and opengl.dll"
	WIN32_GLUT=glut
	WIN32_GLU=glu
	WIN32_OPENGL=opengl
    else 
	echo "Building with glut32.dll, glu32.dll, and opengl32.dll"
	WIN32_GLUT=glut32
	WIN32_GLU=glu32
	WIN32_OPENGL=opengl32
    fi

    LIBS="$LIBS -l${WIN32_GLUT} -l${WIN32_GLU} -l${WIN32_OPENGL}"
    LIBS="$LIBS -luser32 -lgdi32"
    echo "Will link apps with $LIBS"
fi

if test "x$ac_cv_lib_glut_glutGetModifiers" = "xno"; then
    echo 
    echo "Unable to find the necessary OpenGL or GLUT libraries."
    echo "See config.log for automated test details and results ..."
    exit 1
fi

if test "x$ac_cv_lib_glut_glutGameModeString" = "xno"; then
    echo
    echo "Your version of glut doesn't support game mode."
    echo "You need to fetch and install the latest version of glut from:"
    echo
    echo "    http://reality.sgi.com/opengl/glut3/glut3.html"
    exit 1
fi

AC_LANG_PUSH(C++)
# needed for plib 1.8.x or later
AC_CHECK_LIB(plibul, ulInit,,,)

opengl_LIBS="$LIBS"
LIBS="$base_LIBS"

AC_SUBST(base_LIBS)
AC_SUBST(opengl_LIBS)
AC_SUBST(network_LIBS)
AC_SUBST(support_LIBS)

AM_CONDITIONAL(HAVE_XWINDOWS, test "x$ac_cv_lib_X11_XCreateWindow" = "xyes" )

dnl Check for "plib" without which we cannot go on
AC_CHECK_HEADER(plib/sg.h)
if test "x$ac_cv_header_plib_sg_h" != "xyes"; then
    echo
    echo "You *must* have the plib library installed on your system to build"
    echo "TerraGear!"
    echo
    echo "Please see README.plib for more details."
    echo
    echo "configure aborted."
    exit
fi

dnl Check for "libnewmat" without which we cannot build airport surfaces
AC_CHECK_HEADER(newmat/newmat.h)
AM_CONDITIONAL(HAVE_NEWMAT, test "x$ac_cv_header_newmat_newmat_h" = "xyes" )
AC_LANG_POP

# AC_CHECK_HEADER(gts.h)
# if test "x$ac_cv_header_gts_h" != "xyes"; then
#     echo
#     echo "You *must* have the gts library installed on your system to build"
#     echo "TerraGear!"
#     echo
#     echo "Please see README.gts for more details."
#     echo
#     echo "configure aborted."
#     exit
# fi

dnl Check if Generic Polygon Clipping library is installed
dnl (from http://www.cs.man.ac.uk/aig/staff/alan/software/)
AC_CHECK_HEADERS( gpc.h )
if test "x$ac_cv_header_gpc_h" != "xyes"; then
    echo
    echo "You need to have the GPC library installed on your system to"
    echo "build some of the scenery generation tools, otherwise you won't"
    echo "be able to create scenery."
    echo
    echo "Please see README.gpc for more details."
    echo
    echo "configure aborted."
    exit
fi

dnl Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS( \
	fcntl.h getopt.h malloc.h memory.h stdlib.h sys/param.h sys/stat.h \
        sys/time.h sys/timeb.h unistd.h windows.h winbase.h values.h )

# Check for system installed zlib
AC_CHECK_HEADER(zlib.h)
if test "x$ac_cv_header_zlib_h" != "xyes"; then
    echo "no zlib found, please install before continuing."
    exit
fi

# Check for the presence of SimGear
AC_CHECK_HEADER(simgear/version.h)
if test "x$ac_cv_header_simgear_version_h" != "xyes"; then
    echo
    echo "You *must* have the SimGear support library installed on your system"
    echo "to build TerraGear!"
    echo
    echo "Please see README.SimGear for more details."
    echo
    echo "configure aborted."
    exit
fi

AC_MSG_CHECKING(for proper simgear version)

AC_RUN_IFELSE([AC_LANG_SOURCE([[
#include <stdio.h>
#include <simgear/version.h>

#if !defined(SIMGEAR_VERSION)
#error simgear version too old, please upgrade.
#endif

#define STRINGIFY(X) XSTRINGIFY(X)
#define XSTRINGIFY(X) #X

#define MIN_MAJOR 0
#define MIN_MINOR 3
#define MIN_MICRO 8

int main() {
    int major, minor, micro;

    printf("%d.%d.%d or greater... ", MIN_MAJOR, MIN_MINOR, MIN_MICRO);

    sscanf( STRINGIFY(SIMGEAR_VERSION), "%d.%d.%d", &major, &minor, &micro );

    if ( major < MIN_MAJOR ) {
	 return -1;
    } else if ( major == MIN_MAJOR && minor < MIN_MINOR ) {
	 return -1;
    } else if ( major == MIN_MAJOR && minor == MIN_MINOR && micro < MIN_MICRO ){
	 return -1;
    }

    return 0;
}

]])],[AC_MSG_RESULT(yes)],[AC_MSG_RESULT(wrong version);
   AC_MSG_ERROR([Install latest simgear first...])],[AC_MSG_RESULT(yes)
])

dnl Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_TYPE_SIZE_T
AC_HEADER_TIME
AC_STRUCT_TM

dnl check for socklen_t (in Unix98)
AC_MSG_CHECKING(for socklen_t)
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include <sys/types.h>
#include <sys/socket.h>
int accept (int, struct sockaddr *, socklen_t *);
]], [[]])],[
AC_MSG_RESULT(yes)],[
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include <sys/types.h>
#include <sys/socket.h>
int accept (int, struct sockaddr *, size_t *);
]], [[]])],[
AC_MSG_RESULT(size_t)
AC_DEFINE([socklen_t], size_t, [Define for socklen_t])],[
AC_MSG_RESULT(int)
AC_DEFINE([socklen_t], int, [Define for socklen_t])])])

dnl Checks for library functions.
AC_TYPE_SIGNAL
AC_FUNC_VPRINTF
AC_CHECK_FUNCS( ftime gettimeofday timegm memcpy bcopy mktime strstr rand \
	random setitimer getitimer signal GetLocalTime rint getrusage )

AM_CONFIG_HEADER(src/Include/config.h)

AC_CONFIG_FILES([ \
	VERSION \
	Makefile \
	src/Makefile \
	src/Include/Makefile \
	src/Airports/Makefile \
	src/Airports/GenAirports/Makefile \
	src/Airports/Utils/Makefile \
	src/BuildTiles/Makefile \
	src/BuildTiles/Clipper/Makefile \
	src/BuildTiles/GenOutput/Makefile \
	src/BuildTiles/Match/Makefile \
	src/BuildTiles/Triangulate/Makefile \
	src/BuildTiles/Main/Makefile \
	src/BuildTiles/Osgb36/Makefile \
	src/BuildTiles/Parallel/Makefile \
	src/Lib/Makefile \
	src/Lib/Array/Makefile \
	src/Lib/DEM/Makefile \
	src/Lib/e00/Makefile \
	src/Lib/Geometry/Makefile \
	src/Lib/HGT/Makefile \
	src/Lib/landcover/Makefile \
	src/Lib/Optimize/Makefile \
	src/Lib/Output/Makefile \
	src/Lib/Polygon/Makefile \
	src/Lib/poly2tri/Makefile \
	src/Lib/shapelib/Makefile \
	src/Lib/TriangleJRS/Makefile \
	src/Lib/vpf/Makefile \
	src/Prep/Makefile \
	src/Prep/ArrayFit/Makefile \
	src/Prep/DemChop/Makefile \
	src/Prep/DemInfo/Makefile \
	src/Prep/DemRaw2ascii/Makefile \
	src/Prep/E00Lines/Makefile \
	src/Prep/GSHHS/Makefile \
	src/Prep/MergerClipper/Makefile \
	src/Prep/Photo/Makefile \
	src/Prep/OGRDecode/Makefile \
	src/Prep/ShapeFile/Makefile \
	src/Prep/TGVPF/Makefile \
	src/Prep/Terra/Makefile \
	src/Prep/TerraFit/Makefile \
	src/Prep/Tower/Makefile \
	src/Prep/UserDef/Makefile \
	src/Utils/Makefile \
	src/Utils/cdrom/Makefile \
	src/Utils/download-map/Makefile \
])
AC_OUTPUT

echo ""
echo "Configure Summary"
echo "================="

echo "Prefix: $prefix"

if test "x$with_logging" != "x"; then
    echo "Debug messages: $with_logging"
else
    echo "Debug messages: yes"
fi

if test "x$ac_cv_header_newmat_newmat_h" != "xyes"; then
    echo
    echo "You must have the newmat library installed on your system to build"
    echo "the GenAirport utility. This program will not be built."
    echo
    echo "Please see README.newmat for more details."
    echo
fi

dnl :mode=text:indentSize=2:
