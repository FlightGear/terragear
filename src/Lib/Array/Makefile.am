noinst_LIBRARIES = libArray.a

libArray_a_SOURCES = array.cxx array.hxx

# noinst_PROGRAMS = testarray testgts
bin_PROGRAMS = testarray

testarray_SOURCES = testarray.cxx
testarray_LDADD = libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lsgxml \
	$(support_LIBS) -lz 

INCLUDES = -I$(top_srcdir)/src/Lib
