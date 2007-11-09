bin_PROGRAMS = ogr-decode

ogr_decode_SOURCES = ogr-decode.cxx

ogr_decode_CXXFLAGS=$(GDAL_CFLAGS)

ogr_decode_LDADD = \
	$(GDAL_LIBS) \
        $(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	-lsgdebug -lsgbucket -lsgmath -lsgmisc -lsgstructure -lsgxml \
	-lgenpolyclip -lz

INCLUDES = -I$(top_srcdir)/src/Lib
