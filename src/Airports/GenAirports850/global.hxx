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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: global.hxx,v 1.9 2005-10-31 18:43:27 curt Exp $
//


#ifndef _GEN_AIRPORT_GLOBAL_HXX
#define _GEN_AIRPORT_GLOBAL_HXX


extern int nudge;

// Final grid size for airport surface (in meters)
const double coarse_grid = 300.0;

// compared to the average surface elevation, clamp all values within
// this many meters of the average
const double max_clamp = 100.0;

// maximum slope (rise/run) allowed on an airport surface
extern double slope_max; // = 0.02; 
const double slope_eps = 0.00001;

// nurbs query/search epsilon
const double nurbs_eps = 0.0000001;

#endif // _GEN_AIRPORT_GLOBAL_HXX
