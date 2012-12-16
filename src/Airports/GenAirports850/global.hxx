// global.hxx -- kind of dumb but oh well...
//
// Written by Curtis Olson, started February 2002.
//
// Copyright (C) 2002  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: global.hxx,v 1.9 2005-10-31 18:43:27 curt Exp $
//


#ifndef _GEN_AIRPORT_GLOBAL_HXX
#define _GEN_AIRPORT_GLOBAL_HXX


extern int nudge;

// Each polygon vertex is snapped to a grid with this resolution (~1cm by default)
extern double gSnap;

extern double slope_max;
extern double slope_eps;

#endif
