noinst_PROGRAMS = arrayfit

arrayfit_SOURCES = arrayfit.cxx

arrayfit_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lsgxml -lz

INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/Lib
