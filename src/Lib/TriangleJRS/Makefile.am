# CPPFLAGS is a list of definitions used to compile an object code version
#   of Triangle (triangle.o) to be called by another program.  The file
#   "triangle.h" contains detailed information on how to call triangle.o.
#
# The -DTRILIBRARY should always be used when compiling Triangle into an
#   object file.
#
# An example DEFS line is:
#
#   CPPFLAGS = -DTRILIBRARY -DREDUCED -DCDT_ONLY

CPPFLAGS += -DTRILIBRARY

noinst_LIBRARIES = libTriangleJRS.a

libTriangleJRS_a_SOURCES = \
	triangle.c triangle.h \
	tri_support.c tri_support.h

if HAVE_XWINDOWS

bin_PROGRAMS = showme
showme_SOURCES = showme.c
showme_LDADD = -lX11

endif
