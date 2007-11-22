EXTRA_DIST = \
	README README README.cygwin README.gpc README.gts README.howto \
	README.plib README.SimGear \
	TerraGear.dsp TerraGear.dsw
	acinclude.m4 \
        autogen.sh

SUBDIRS = src

dist-hook:
	(cd $(top_srcdir); ./am2dsp.pl)
