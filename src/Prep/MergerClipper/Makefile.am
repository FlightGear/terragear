noinst_LIBRARIES = libMergerClipper.a

libMergerClipper_a_SOURCES = merger.cxx merger.hxx

noinst_PROGRAMS = testmerger

testmerger_SOURCES = testmerger.cxx

testmerger_LDADD = \
	$(top_builddir)/Prep/MergerClipper/libMergerClipper.a \
	$(top_builddir)/Lib/Polygon/libPolygon.a \
	$(top_builddir)/Lib/poly2tri/libpoly2tri.a \
	-lsgdebug -lsgbucket -lsgmisc -lz -lgpc

INCLUDES += -I$(top_builddir) -I$(top_builddir)/Lib
