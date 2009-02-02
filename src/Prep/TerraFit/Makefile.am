bin_SCRIPTS = terrafit.py
bin_PROGRAMS = terrafit

terrafit_SOURCES = terrafit.cc
terrafit_LDADD = $(top_builddir)/src/Prep/Terra/libTerra.a \
                 $(top_builddir)/src/Lib/Array/libArray.a \
                 -lsgbucket -lsgstructure -lsgmisc -lsgdebug -lz -lplibul

EXTRA_DIST = terrafit.README terrafit.py.in

terrafit.py: $(srcdir)/terrafit.py.in
	echo ${bindir}
	sed -e "s%__bindir__%${bindir}%g" $(srcdir)/terrafit.py.in > terrafit.py

INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/Lib
