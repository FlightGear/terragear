EXTRA_DIST = README makefile.orig data_1

# if OLD_AUTOMAKE
# CFLAGS += -DSTANDALONE
# else
# AM_CFLAGS = -DSTANDALONE
# endif

noinst_LIBRARIES = libpoly2tri.a

libpoly2tri_a_SOURCES = \
	construct.c \
	interface.h \
        misc.c \
        monotone.c \
	tri.c \
	triangulate.h

# INCLUDES += -I$(top_srcdir) -I$(top_srcdir)/Lib
