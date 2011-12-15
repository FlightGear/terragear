// rwy_common.hxx -- Common runway generation routines
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
// $Id: rwy_common.hxx,v 1.5 2004-11-19 22:25:49 curt Exp $
//


#ifndef _RWY_COMMON_HXX
#define _RWY_COMMON_HXX


#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>

#include "runway.hxx"


void gen_number_block( const TGRunway& rwy_info,
                       const std::string& material,
                       TGPolygon poly, double heading, int num,
                       double start_pct, double end_pct,
                       superpoly_list *rwy_polys,
                       texparams_list *texparams,
                       TGPolygon *accum );

// generate the runway stopway
void gen_runway_stopway( const TGRunway& rwy_info,
                         const TGPolygon& runway_a,
                         const TGPolygon& runway_b,
                         const std::string& prefix,
                         superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         TGPolygon* accum );

// generate a section of runway
void gen_runway_section( const TGRunway& rwy_info,
			 const TGPolygon& runway,
			 double startl_pct, double endl_pct,
			 double startw_pct, double endw_pct,
                         double minu, double maxu, double minv, double maxv,
			 double heading,
			 const std::string& prefix,
			 const std::string& material,
			 superpoly_list *rwy_polys,
			 texparams_list *texparams,
			 TGPolygon *accum  );


#endif // _RWY_COMMON_HXX
