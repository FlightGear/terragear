noinst_LIBRARIES = libClipper.a

libClipper_a_SOURCES = clipper.cxx clipper.hxx priorities.cxx priorities.hxx
dist_pkgdata_DATA = default_priorities.txt
noinst_PROGRAMS = testclipper

testclipper_SOURCES = testclipper.cxx

testclipper_LDADD = \
	$(top_builddir)/src/BuildTiles/Clipper/libClipper.a \
	$(top_builddir)/src/BuildTiles/Osgb36/libOsgb36.a \
	$(top_builddir)/src/BuildTiles/Triangulate/libTriangulate.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/landcover/liblandcover.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	-lsgbucket -lsgdebug -lsgmath -lsgmisc -lsgstructure -lsgxml \
	-lgenpolyclip -lz 

INCLUDES = -I$(top_srcdir)/src/Lib -I$(top_srcdir)/src/BuildTiles

