noinst_LIBRARIES = libe00.a

libe00_a_SOURCES = e00.cxx e00.hxx

noinst_PROGRAMS = teste00

teste00_SOURCES = teste00.cxx
teste00_LDADD = libe00.a -lsgmisc -lsgdebug -lsgxml -lz

INCLUDES = \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/src/Lib
