bin_PROGRAMS = shape-decode

shape_decode_SOURCES = main.cxx

shape_decode_LDADD = \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/shapelib/libshape.a \
	-lsgdebug -lsgbucket -lsgmisc -lz -lgpc

INCLUDES += -I$(top_srcdir)/src/Lib
