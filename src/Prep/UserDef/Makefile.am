bin_PROGRAMS = tguserdef

tguserdef_SOURCES = tguserdef.cxx

tguserdef_LDADD = \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	-lsgbucket -lsgmisc -lsgmath -lsgio -lsgxml -lsgdebug -lgenpolyclip -lz

INCLUDES = -I$(top_srcdir)/src/Lib
