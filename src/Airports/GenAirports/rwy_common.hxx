// rwy_common.hxx -- Common runway generation routines
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


#ifndef _RWY_COMMON_HXX
#define _RWY_COMMON_HXX


#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>

#include "runway.hxx"
#include "texparams.hxx"


void gen_number_block( const FGRunway& rwy_info,
			      const string& material,
			      TGPolygon poly, double heading, int num,
			      double start_pct, double end_pct,
			      superpoly_list *rwy_polys,
			      texparams_list *texparams,
			      TGPolygon *accum );

// generate a section of runway
void gen_runway_section( const FGRunway& rwy_info,
			 const TGPolygon& runway,
			 double startl_pct, double endl_pct,
			 double startw_pct, double endw_pct,
			 double heading,
			 const string& prefix,
			 const string& material,
			 superpoly_list *rwy_polys,
			 texparams_list *texparams,
			 TGPolygon *accum  );


#endif // _RWY_COMMON_HXX
