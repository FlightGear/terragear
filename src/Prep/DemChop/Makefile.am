#---------------------------------------------------------------------------
# Makefile
#
# Written by Curtis Olson, started March 1999.
#
# Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
# $Id: Makefile.am,v 1.9 2005-11-08 16:27:36 curt Exp $
#---------------------------------------------------------------------------


bin_PROGRAMS = demchop hgtchop fillvoids testassem

if HAVE_LIBTIFF
bin_PROGRAMS+=srtmchop
endif

demchop_SOURCES = \
	demchop.cxx point2d.hxx

demchop_LDADD = \
	$(top_builddir)/src/Lib/DEM/libDEM.a \
	-lsgbucket -lsgmisc -lsgdebug -lsgxml -lz
	$(base_LIBS)

hgtchop_SOURCES = \
	hgtchop.cxx point2d.hxx

hgtchop_LDADD = \
	$(top_builddir)/src/Lib/HGT/libHGT.a \
	-lsgbucket -lsgmisc -lsgdebug -lsgxml -lz -lplibul
	$(base_LIBS)

if HAVE_LIBTIFF
srtmchop_SOURCES = \
	srtmchop.cxx point2d.hxx

srtmchop_LDADD = \
	$(top_builddir)/src/Lib/HGT/libHGT.a \
	-lsgbucket -lsgmisc -lsgdebug -lsgxml -lz -lplibul -ltiff
	$(base_LIBS)
endif

fillvoids_SOURCES = \
	fillvoids.cxx

fillvoids_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	-lsgbucket -lsgmisc -lsgdebug -lsgxml -lz
	$(base_LIBS)

testassem_SOURCES = \
	testassem.cxx

testassem_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	-lsgbucket -lsgmath -lsgmisc -lsgdebug -lsgxml -lz
	$(base_LIBS)

INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/Lib

# We can't build this with "-O2" (optimization) since this causes a seg fault
# I haven't found a way to strip this out of the CXXFLAGS, so I'm just
# setting it to "-g"
# CXXFLAGS = -g 


#---------------------------------------------------------------------------
