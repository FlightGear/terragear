bin_PROGRAMS = tgvpf

tgvpf_SOURCES = tgvpf.cxx

tgvpf_LDADD = \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	$(top_builddir)/src/Lib/vpf/libvpf.a \
	-lsgdebug -lsgbucket -lsgmisc -lsgmath -lsgio -lsgxml -lgenpolyclip -lz

INCLUDES = -I$(top_srcdir)/src/Lib
