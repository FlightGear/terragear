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


bin_PROGRAMS = genapts testnurbs

genapts_SOURCES = \
	apt_surface.hxx apt_surface.cxx \
	build.cxx build.hxx \
	convex_hull.cxx convex_hull.hxx \
	global.hxx \
	lights.hxx lights.cxx \
	main.cxx \
	point2d.cxx point2d.hxx \
	poly_extra.cxx poly_extra.hxx \
	runway.cxx runway.hxx \
	rwy_common.cxx rwy_common.hxx \
	rwy_nonprec.cxx rwy_nonprec.hxx \
	rwy_prec.cxx rwy_prec.hxx \
	rwy_simple.cxx rwy_simple.hxx \
	rwy_visual.cxx rwy_visual.hxx \
	taxiway.cxx taxiway.hxx \
	texparams.hxx

genapts_LDADD = \
	$(top_builddir)/src/Lib/Array/libArray.a \
	$(top_builddir)/src/Lib/Polygon/libPolygon.a \
	$(top_builddir)/src/Lib/poly2tri/libpoly2tri.a \
	$(top_builddir)/src/Lib/Geometry/libGeometry.a \
	$(top_builddir)/src/Lib/Output/libOutput.a \
	$(top_builddir)/src/Lib/TriangleJRS/libTriangleJRS.a \
	-lsgbucket -lsgdebug -lsgio -lsgmath -lsgmisc -lsgxml -lgenpolyclip \
        -lnurbsf -lmatrixI -lmatrixN -lmatrix \
	-lz \
	$(base_LIBS)

testnurbs_SOURCES = testnurbs.cxx
testnurbs_LDADD = -lnurbsf -lmatrixI -lmatrixN -lmatrix

INCLUDES = \
	-I$(top_srcdir) \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/src/Lib \
	-I$(top_srcdir)/src/BuildTiles
