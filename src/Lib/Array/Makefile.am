noinst_LIBRARIES = libArray.a

libArray_a_SOURCES = array.cxx array.hxx

noinst_PROGRAMS = testarray testgts

testarray_SOURCES = testarray.cxx

testarray_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lsgxml -lz -lgts

testgts_SOURCES = testgts.cxx

testgts_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lsgxml -lz -lgts

INCLUDES = -I$(top_srcdir)/src
