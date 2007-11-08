bin_SCRIPTS = terrafit.py
EXTRA_DIST = terrafit.README terrafit.py.in

terrafit.py: $(srcdir)/terrafit.py.in
	echo ${bindir}
	sed -e "s%__bindir__%${bindir}%g" $(srcdir)/terrafit.py.in > terrafit.py