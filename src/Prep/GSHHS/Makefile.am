bin_PROGRAMS = gshhs debug

gshhs_SOURCES = \
	main.cxx \
	gshhs.h \
	gshhs_split.cxx gshhs_split.hxx \
	simple_clip.hxx simple_clip.cxx

gshhs_LDADD = \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgbucket -lsgmisc -lz -lgpc

debug_SOURCES = \
	debug.cxx

debug_LDADD = \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgbucket -lsgmisc -lz -lgpc

INCLUDES += -I$(top_srcdir)/src -I$(top_srcdir)/src/Lib
