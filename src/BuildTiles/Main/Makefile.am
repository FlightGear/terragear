bin_PROGRAMS = fgfs-construct fgfs-master

fgfs_construct_SOURCES = \
	construct.cxx construct.hxx \
	main.cxx

fgfs_construct_LDADD = \
	$(top_builddir)/Construct/Clipper/libClipper.a \
	$(top_builddir)/Construct/GenOutput/libGenOutput.a \
	$(top_builddir)/Construct/Combine/libCombine.a \
	$(top_builddir)/Construct/Match/libMatch.a \
	$(top_builddir)/Construct/Triangulate/libTriangulate.a \
	$(top_builddir)/Lib/Array/libArray.a \
	$(top_builddir)/Lib/Build/libBuild.a \
	$(top_builddir)/Lib/Polygon/libPolygon.a \
	$(top_builddir)/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/Lib/Triangle/libTriangle.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lz -lgpc

fgfs_master_SOURCES = master.cxx

fgfs_master_LDADD = -lsgbucket -lsgmisc

INCLUDES += \
	-I$(top_builddir) \
	-I$(top_builddir)/Lib \
	-I$(top_builddir)/Construct
