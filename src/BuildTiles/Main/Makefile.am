bin_PROGRAMS = fgfs-construct fgfs-master

fgfs_construct_SOURCES = \
	construct.cxx construct.hxx \
	main.cxx

fgfs_construct_LDADD = \
	$(top_builddir)/src/BuildTiles/Clipper/libClipper.a \
	$(top_builddir)/src/BuildTiles/GenOutput/libGenOutput.a \
	$(top_builddir)/src/BuildTiles/Match/libMatch.a \
	$(top_builddir)/src/BuildTiles/Osgb36/libOsgb36.a \
	$(top_builddir)/src/BuildTiles/Triangulate/libTriangulate.a \
	$(top_builddir)/src/Lib/Array/libArray.a \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/Optimize/libOptimize.a \
	$(top_builddir)/src/Lib/Output/libOutput.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/landcover/liblandcover.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	-lsgbucket -lsgio -lsgmath -lsgmisc -lsgdebug -lplibsg -lz -lgpc

fgfs_master_SOURCES = master.cxx

fgfs_master_LDADD = -lsgbucket -lsgmisc

INCLUDES += \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/src/Lib \
	-I$(top_srcdir)/src/BuildTiles
