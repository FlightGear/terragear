bin_PROGRAMS = tguserdef

tguserdef_SOURCES = tguserdef.cxx

tguserdef_LDADD = \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/Output/libOutput.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	-lsgbucket -lsgmath -lsgprops -lsgmisc -lsgio -lsgstructure -lsgxml \
	-lsgdebug \
	-lOpenThreads \
	-lgenpolyclip -lz

INCLUDES = -I$(top_srcdir)/src/Lib
