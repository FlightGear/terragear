bin_PROGRAMS = arrayfit demo

arrayfit_SOURCES = arrayfit.cxx

arrayfit_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lsgxml \
	$(support_LIBS) -lz

demo_SOURCES = demo.cxx

demo_LDADD = $(support_LIBS) -lz

INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/Lib

