bin_PROGRAMS = e00lines

e00lines_SOURCES = main.cxx

e00lines_LDADD = \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/e00/libe00.a \
	-lsgdebug -lsgbucket -lsgmisc -lsgmath -lsgio -lz -lgpc

INCLUDES += -I$(top_srcdir)/src/Lib
