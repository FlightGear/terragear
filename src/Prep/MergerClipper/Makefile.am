noinst_LIBRARIES = libMergerClipper.a

libMergerClipper_a_SOURCES = merger.cxx merger.hxx

noinst_PROGRAMS = testmerger

testmerger_SOURCES = testmerger.cxx

testmerger_LDADD = \
	$(top_builddir)/src/Prep/MergerClipper/libMergerClipper.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgbucket -lsgmisc -lz -lgpc

INCLUDES += -I$(top_srcdir)/src/Lib
