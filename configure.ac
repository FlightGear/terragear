dnl Process this file with aclocal ; automake -a ; autoconf to produce a 
dnl working configure script.
dnl
dnl $Id$

AC_INIT
AC_CONFIG_SRCDIR([src/Lib/DEM/dem.cxx])

# Require at least automake 2.52
AC_PREREQ(2.52)

dnl Initialize the automake stuff
AM_INIT_AUTOMAKE(TerraGear, 0.0.5)

dnl Checks for programs.
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_CXX
AC_PROG_RANLIB
AC_PROG_INSTALL
AC_PROG_LN_S

AR="ar"
OS=`uname -s`
if test "$OS" = "IRIX" -o "$OS" = "IRIX64"; then
    if test "$CXX" = "CC"; then
        AR="CC -ar"
        AC_SUBST(AR)
    fi
fi

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

AC_CHECK_LIB(m, cos)

base_LIBS="$LIBS"

AC_CHECK_LIB(socket, socket)
AC_CHECK_LIB(X11, XCreateWindow)
AC_CHECK_LIB(Xext, XShmCreateImage)
AC_CHECK_LIB(Xi, XGetExtensionVersion)
AC_CHECK_LIB(ICE, IceOpenConnection)
AC_CHECK_LIB(SM, SmcOpenConnection)
AC_CHECK_LIB(Xt, XtMalloc)
AC_CHECK_LIB(Xmu, XmuLookupStandardColormap)

dnl check for OpenGL related libraries

if test "x$ac_cv_header_windows_h" != "xyes" ; then
    dnl Reasonable stuff for non-windoze variants ... :-)

    AC_CHECK_LIB(GLcore, glNewList)
    if test "x$ac_cv_lib_GLcore_glNewList" = "xno" ; then
	dnl if no GLcore, check for GL
	AC_CHECK_LIB(GL, glNewList)
	if test "x$ac_cv_lib_GL_glNewList" = "xno" ; then
	    dnl if no GL, check for MesaGL
	    AC_CHECK_LIB(MesaGL, glNewList)
	fi
    else
	dnl if GLcore found, then also check for GL
	AC_CHECK_LIB(GL, glXCreateContext)
    fi

    dnl if using mesa, check for xmesa.h
    if test "x$ac_cv_lib_MesaGL_glNewList" = "xyes" ; then
	AC_CHECK_HEADER(GL/xmesa.h)
	if test "x$ac_cv_header_GL_xmesa_h" = "xyes"; then
	    AC_DEFINE([XMESA], 1, [Define for Mesa FX mode])
            AC_DEFINE([FX], 1, [Define for Mesa FX mode])
        fi
    fi

    AC_CHECK_LIB(GLU, gluLookAt)
    if test "x$ac_cv_lib_GLU_gluLookAt" = "xno" ; then
	dnl if no GLU, check for MesaGLU
	AC_CHECK_LIB(MesaGLU, gluLookAt)
    fi

    dnl check for glut
    AC_CHECK_LIB(glut, glutGetModifiers)

    dnl test for glutGameModeString, but avoid adding glut a second time into
    dnl the list of libraries
    save_LIBS="$LIBS"
    AC_CHECK_LIB(glut, glutGameModeString)
    LIBS="$save_LIBS"
else
    dnl Win32 is a little wierd because it has to try to handle the various
    dnl winbloze-isms.  We'll just do this manually for now.

    echo Win32 specific hacks...
    AC_DEFINE([WIN32], 1, [Define if building on a Win32 platform])

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

opengl_LIBS="$LIBS"
LIBS="$base_LIBS"

AC_SUBST(base_LIBS)
AC_SUBST(opengl_LIBS)

# needed for plib 1.3.x and later 
AC_CHECK_LIB(plibul, ulInit,,,)

AM_CONDITIONAL(HAVE_XWINDOWS, test "x$ac_cv_lib_X11_XCreateWindow" = "xyes" )

AC_LANG_PUSH(C++)
dnl Check for "plib" without which we cannot go on
AC_CHECK_HEADER(plib/pu.h)
if test "x$ac_cv_header_plib_pu_h" != "xyes"; then
    echo
    echo "You *must* have the plib library installed on your system to build"
    echo "the FGFS simulator!"
    echo
    echo "Please see README.plib for more details."
    echo
    echo "configure aborted."
    exit
fi
AC_LANG_POP

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
    echo "to build the FGFS simulator!"
    echo
    echo "Please see README.SimGear for more details."
    echo
    echo "configure aborted."
    exit
fi

AC_MSG_CHECKING(for proper simgear version)

AC_TRY_RUN([
#include <stdio.h>
#include <simgear/version.h>

#if !defined(SIMGEAR_VERSION)
#error simgear version too old, please upgrade.
#endif

#define STRINGIFY(X) XSTRINGIFY(X)
#define XSTRINGIFY(X) #X

#define MIN_MAJOR 0
#define MIN_MINOR 0
#define MIN_MICRO 13

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

],
  AC_MSG_RESULT(yes),
  [AC_MSG_RESULT(wrong version);
   AC_MSG_ERROR([Install latest simgear first...])],
  AC_MSG_RESULT(yes)
)

dnl Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_TYPE_SIZE_T
AC_HEADER_TIME
AC_STRUCT_TM

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
	src/Lib/landcover/Makefile \
	src/Lib/Optimize/Makefile \
	src/Lib/Output/Makefile \
	src/Lib/Polygon/Makefile \
	src/Lib/poly2tri/Makefile \
	src/Lib/shapelib/Makefile \
	src/Lib/TriangleJRS/Makefile \
	src/Lib/vpf/Makefile \
	src/Prep/Makefile \
	src/Prep/DemChop/Makefile \
	src/Prep/DemInfo/Makefile \
	src/Prep/DemRaw2ascii/Makefile \
	src/Prep/E00Lines/Makefile \
	src/Prep/GSHHS/Makefile \
	src/Prep/MergerClipper/Makefile \
	src/Prep/Photo/Makefile \
	src/Prep/ShapeFile/Makefile \
	src/Prep/TGVPF/Makefile \
	src/Prep/UserDef/Makefile \
	src/Utils/Makefile \
	src/Utils/cdrom/Makefile \
	src/Utils/download-map/Makefile \
	src/Utils/TerraSync/Makefile \
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

if test "$OS" = "IRIX" -o "$OS" = "IRIX64"; then
    if test "$CXX" = "CC"; then
        echo "\nRemember to run: sh ../FlightGear-0.x.x/irix-hack.sh!\07\n"
    fi
fi

