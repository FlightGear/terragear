EXTRA_DIST = \
	PATCH-nurbs++-3.0.11-with-gcc-3.4.x \
	README README README.cygwin README.gpc README.gts README.howto \
	README.nurbs++ README.plib README.SimGear \
	TerraGear.dsp TerraGear.dsw
	acinclude.m4 \
        autogen.sh

SUBDIRS = src

dist-hook:
	(cd $(top_srcdir); $(HOME)/Projects/FlightGear-0.9/admin/am2dsp.pl)
