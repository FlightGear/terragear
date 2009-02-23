noinst_LIBRARIES = libHGT.a

libHGT_a_SOURCES = hgt.cxx hgt.hxx srtmbase.cxx srtmbase.hxx

noinst_PROGRAMS = testhgt

testhgt_SOURCES = testhgt.cxx

testhgt_LDADD = \
	$(top_builddir)/src/Lib/HGT/libHGT.a \
	-lsgbucket -lsgmisc -lsgdebug -lz -lplibul

INCLUDES = -I$(top_srcdir)/src

