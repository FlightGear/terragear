noinst_LIBRARIES = libClipper.a

libClipper_a_SOURCES = clipper.cxx clipper.hxx

noinst_PROGRAMS = testclipper

testclipper_SOURCES = testclipper.cxx

testclipper_LDADD = \
	$(top_builddir)/src/Construct/Clipper/libClipper.a \
	$(top_builddir)/src/Construct/Osgb36/libOsgb36.a \
	$(top_builddir)/src/Construct/Triangulate/libTriangulate.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/landcover/liblandcover.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	-lsgbucket -lsgdebug -lsgmisc -lz -lgpc

INCLUDES += -I$(top_srcdir)/src/Lib -I$(top_srcdir)/src/Construct
