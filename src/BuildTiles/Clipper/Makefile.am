noinst_LIBRARIES = libClipper.a

libClipper_a_SOURCES = clipper.cxx clipper.hxx

noinst_PROGRAMS = testclipper

testclipper_SOURCES = testclipper.cxx

testclipper_LDADD = \
	$(top_builddir)/Construct/Clipper/libClipper.a \
	$(top_builddir)/Construct/Triangulate/libTriangulate.a \
	$(top_builddir)/Lib/Polygon/libPolygon.a \
	$(top_builddir)/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgmisc -lz -lgpc

INCLUDES += -I$(top_builddir) \
	-I$(top_builddir)/Lib
