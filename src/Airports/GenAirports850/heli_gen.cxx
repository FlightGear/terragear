// rwy_gen.cxx -- Build a runway
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

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>

#include "rwy_common.hxx"

#include <stdlib.h>

using std::string;


void gen_heli( const TGRunway& rwy_info,
                        double alt_m,
			const string& material,
			superpoly_list *rwy_polys,
			texparams_list *texparams,
			TGPolygon *accum )
{
    SG_LOG( SG_GENERAL, SG_INFO, "Building helipad = " << rwy_info.rwy_no1 );

    //
    // Generate the basic helipad outlines
    //
    Point3D helicenter = Point3D( rwy_info.lon, rwy_info.lat, 0.0);

    TGPolygon helipad = gen_wgs84_area( helicenter, rwy_info.length, 0, 0, rwy_info.width, rwy_info.heading,
                                        alt_m, false);

    double start1_pct = 0.0;
    double end1_pct = 0.0;
    double maxsize = rwy_info.width - rwy_info.length;

    if (maxsize <= 0)
        maxsize = rwy_info.width;
    else if (maxsize > 0)
        maxsize = rwy_info.length;

    double percent = (maxsize / rwy_info.length) /2;

    start1_pct = 0.5 - percent;
    end1_pct = 0.5 + percent;
    gen_runway_section( rwy_info, helipad,
                        start1_pct, end1_pct,
                        0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
                        rwy_info.heading,
                        material, "heli",
                        rwy_polys, texparams, accum );
}
