#---------------------------------------------------------------------------
# Makefile
#
# Written by Curtis Olson, started January 1998.
#
# Copyright (C) 1998  Curtis L. Olson  - curt@me.umn.edu
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


EXTRA_DIST = chop.pl trans.pl

bin_PROGRAMS = photo wgs84offset

photo_SOURCES = photo.cxx

photo_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/Output/libOutput.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	-lsgbucket -lsgdebug -lsgio -lsgmath -lsgmisc -lz -lgenpolyclip \
	$(base_LIBS)

wgs84offset_SOURCES = wgs84offset.cxx

wgs84offset_LDADD = -lsgmath -lsgdebug

INCLUDES += \
	-I$(top_srcdir) \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/src/Lib \
	-I$(top_srcdir)/src/BuildTiles
