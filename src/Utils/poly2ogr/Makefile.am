bin_PROGRAMS = poly2ogr

poly2ogr_SOURCES = poly2ogr.cxx
poly2ogr_CXXFLAGS=$(GDAL_CFLAGS)
poly2ogr_LDADD = \
	$(GDAL_LIBS) \
        $(top_builddir)/src/Lib/Polygon/libPolygon.a \
        -lsgio -lsgbucket -lsgmath -lsgstructure -lsgmisc -lsgdebug -lz -lplibul

INCLUDES = -I$(top_srcdir)/src/Lib
