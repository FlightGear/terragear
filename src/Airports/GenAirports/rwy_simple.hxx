// rwy_simple.hxx -- Build a simple (non-marked) runway
//
// Written by Curtis Olson, started February 2002.
//
// Copyright (C) 2002  Curtis L. Olson  - curt@flightgear.org
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
// $Id$
//


#ifndef _RWY_SIMPLE_HXX
#define _RWY_SIMPLE_HXX


#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>

#include "runway.hxx"
#include "texparams.hxx"


// generate a simple runway.  The routine modifies rwy_polys,
// texparams, and accum
void gen_simple_rwy( const FGRunway& rwy_info,
                     double alt_m,
                     const string& material,
		     superpoly_list *rwy_polys,
		     texparams_list *texparams,
		     FGPolygon *accum );


#endif // _RWY_SIMPLE_HXX
