EXTRA_DIST = Makefile.gcc-2.95 Makefile.orig

bin_PROGRAMS = terra xterra

terra_SOURCES = \
	Array.h Geom.h GreedyInsert.cc GreedyInsert.h Heap.cc Heap.h \
	Map.cc Map.h Mask.cc \
	Mask.h Quadedge.cc Quadedge.h Subdivision.cc Subdivision.h Vec2.h \
	Vec3.h cmdline.cc glHacks.cc glHacks.h greedy.cc gui.cc gui.h \
	output.cc terra.cc terra.h version.h

terra_LDADD = $(opengl_LIBS)

xterra_SOURCES = \
	Array.h Geom.h GreedyInsert.cc GreedyInsert.h Heap.cc Heap.h \
	Map.cc Map.h Mask.cc \
	Mask.h Quadedge.cc Quadedge.h Subdivision.cc Subdivision.h Vec2.h \
	Vec3.h cmdline.cc glHacks.cc glHacks.h greedy.cc gui.cc gui.h \
	output.cc terra.h version.h xterra.cc

xterra_LDADD = $(opengl_LIBS)
