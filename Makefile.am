EXTRA_DIST = \
	README README.howto README.gpc \
	TerraGear.dsp TerraGear.dsw
	acinclude.m4 \
        autogen.sh

SUBDIRS = src

dist-hook:
	(cd $(top_srcdir); $(HOME)/Projects/FlightGear-0.9/admin/am2dsp.pl)
