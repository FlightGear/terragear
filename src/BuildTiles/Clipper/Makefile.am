noinst_LIBRARIES = libClipper.a

libClipper_a_SOURCES = clipper.cxx clipper.hxx

noinst_PROGRAMS = testclipper

testclipper_SOURCES = testclipper.cxx

testclipper_LDADD = \
	$(top_builddir)/src/Construct/Clipper/libClipper.a \
	$(top_builddir)/src/Construct/Triangulate/libTriangulate.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgmisc -lz -lgpc

INCLUDES += -I$(top_builddir)/src/Lib
