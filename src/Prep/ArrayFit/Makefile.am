bin_PROGRAMS = arrayfit demo

arrayfit_SOURCES = arrayfit.cxx

arrayfit_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lsgxml -lz -lgts

demo_SOURCES = demo.cxx

demo_LDADD = -lz -lgts

INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/Lib
