bin_PROGRAMS = fgfs-construct fgfs-master

fgfs_construct_SOURCES = \
	construct.cxx construct.hxx \
	main.cxx

fgfs_construct_LDADD = \
	$(top_builddir)/src/Construct/Clipper/libClipper.a \
	$(top_builddir)/src/Construct/GenOutput/libGenOutput.a \
	$(top_builddir)/src/Construct/Combine/libCombine.a \
	$(top_builddir)/src/Construct/Match/libMatch.a \
	$(top_builddir)/src/Construct/Triangulate/libTriangulate.a \
	$(top_builddir)/src/Lib/Array/libArray.a \
	$(top_builddir)/src/Lib/Build/libBuild.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/Triangle/libTriangle.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lplibsg -lz -lgpc

fgfs_master_SOURCES = master.cxx

fgfs_master_LDADD = -lsgbucket -lsgmisc

INCLUDES += \
	-I$(top_builddir)/src \
	-I$(top_builddir)/src/Lib \
	-I$(top_builddir)/src/Construct
