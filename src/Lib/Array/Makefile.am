noinst_LIBRARIES = libArray.a

libArray_a_SOURCES = array.cxx array.hxx

noinst_PROGRAMS = testarray

testarray_SOURCES = testarray.cxx

testarray_LDADD = \
	$(top_builddir)/Lib/Array/libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lz

INCLUDES += \
	-I$(top_builddir) \
	-I$(top_builddir)/Lib \
	-I$(top_builddir)/Tools/Lib \
	-I$(top_builddir)/Tools/Construct
