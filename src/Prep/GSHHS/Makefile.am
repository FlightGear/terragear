bin_PROGRAMS = gshhs debug

gshhs_SOURCES = \
	main.cxx \
	gshhs.h \
	gshhs_split.cxx gshhs_split.hxx \
	simple_clip.hxx simple_clip.cxx

gshhs_LDADD = \
	$(top_builddir)/Lib/Polygon/libPolygon.a \
	$(top_builddir)/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgbucket -lsgmisc -lz -lgpc

debug_SOURCES = \
	debug.cxx

debug_LDADD = \
	$(top_builddir)/Lib/Polygon/libPolygon.a \
	$(top_builddir)/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgbucket -lsgmisc -lz -lgpc

INCLUDES += -I$(top_builddir) -I$(top_builddir)/Lib -I$(top_builddir)/Tools/Lib
