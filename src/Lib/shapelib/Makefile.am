EXTRA_DIST = ChangeLog dbf_api.html makeshape.sh shape.errata \
	shapelib.html shp_api.html stream1.sh stream2.sh

noinst_LIBRARIES = libshape.a

libshape_a_SOURCES = \
	dbfopen.c shpopen.c shapefil.h 

noinst_PROGRAMS = shpcreate shpadd shpdump \
	dbfcreate dbfadd dbfdump \
	shptest shputils

shpcreate_SOURCES = shpcreate.c
shpcreate_LDADD = libshape.a

shpadd_SOURCES = shpadd.c
shpadd_LDADD = libshape.a
 
shpdump_SOURCES = shpdump.c
shpdump_LDADD = libshape.a
 
dbfcreate_SOURCES = dbfcreate.c
dbfcreate_LDADD = libshape.a
 
dbfadd_SOURCES = dbfadd.c
dbfadd_LDADD = libshape.a
 
dbfdump_SOURCES = dbfdump.c
dbfdump_LDADD = libshape.a
 
shptest_SOURCES = shptest.c
shptest_LDADD = libshape.a
 
shputils_SOURCES = shputils.c
shputils_LDADD = libshape.a

INCLUDES = -I$(top_srcdir)/src
