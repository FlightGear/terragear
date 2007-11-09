#---------------------------------------------------------------------------
# Makefile
#
# Written by Ralf Gerlich, started Nov 2007
#
# Copyright (C) 2007  Ralf Gerlich - http://www.custom-scenery.org/
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
# $Id$
#---------------------------------------------------------------------------

bin_PROGRAMS = gdalchop

gdalchop_SOURCES = \
	gdalchop.cxx

gdalchop_CXXFLAGS=$(GDAL_CFLAGS)

gdalchop_LDADD = \
	$(GDAL_LIBS) -lsgbucket -lsgmisc -lsgdebug -lsgxml -lz
	$(base_LIBS)

INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/Lib

#---------------------------------------------------------------------------
