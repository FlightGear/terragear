noinst_LIBRARIES = libvpf.a

libvpf_a_SOURCES = \
	component.hxx \
	coverage.hxx \
	database.hxx \
	feature.hxx \
	label.hxx \
	library.hxx \
	contour.hxx \
	line.hxx \
	polygon.hxx \
	property.hxx \
	table.hxx \
	tablemgr.hxx \
	tile.hxx \
	value.hxx \
	vpf.hxx \
	vpfbase.hxx \
	vpfbase.cxx \
	value.cxx \
	table.cxx \
	tablemgr.cxx \
	component.cxx \
	database.cxx \
	library.cxx \
	coverage.cxx \
	feature.cxx \
	line.cxx \
	contour.cxx \
	polygon.cxx \
	label.cxx \
	property.cxx \
        tile.cxx \
	vpf-summary.cxx \
	vpf-dump.cxx \
	vpf-topology.cxx

noinst_PROGRAMS = vpf-dump vpf-topology vpf-summary

vpf_dump_SOURCES = vpf-dump.cxx
vpf_dump_LDADD = libvpf.a

vpf_topology_SOURCES = vpf-topology.cxx
vpf_topology_LDADD = libvpf.a

vpf_summary_SOURCES = vpf-summary.cxx
vpf_summary_LDADD = libvpf.a

