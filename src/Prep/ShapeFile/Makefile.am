bin_PROGRAMS = shape-decode noaa-decode

shape_decode_SOURCES = shape-decode.cxx

noaa_decode_SOURCES = shape-decode.cxx

LDADD = \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/Output/libOutput.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/shapelib/libshape.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	-lsgdebug -lsgbucket -lsgmath -lsgmisc -lsgstructure -lsgxml \
	-lgenpolyclip -lz

INCLUDES = -I$(top_srcdir)/src/Lib
