// rwy_simple.cxx -- Build a simple (non-marked) runway
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
#include <Polygon/superpoly.hxx>
#include "texparams.hxx"
#include "runway.hxx"

using std::string;


// generate a simple runway.  The routine modifies rwy_polys,
// texparams, and accum
void Runway::gen_simple_rwy( double alt_m,
                     const string& material,
		     superpoly_list *rwy_polys,
		     texparams_list *texparams,
		     TGPolygon *accum )
{
    int i;

    TGPolygon runway = gen_runway_w_mid( alt_m, 0.0, 0.0 );

    TGPolygon runway_half;

for ( int rwhalf=0; rwhalf<2; ++rwhalf ){

    if (rwhalf == 0) {

    //Create the first half of the runway (first entry in apt.dat)
    runway_half.erase();
    runway_half.add_node( 0, runway.get_pt(0, 3) );
    runway_half.add_node( 0, runway.get_pt(0, 4) );
    runway_half.add_node( 0, runway.get_pt(0, 5) );
    runway_half.add_node( 0, runway.get_pt(0, 2) );
    }

    else if (rwhalf == 1) {

    //Create the second runway half from apt.dat
    runway_half.erase();
    runway_half.add_node( 0, runway.get_pt(0, 0) );
    runway_half.add_node( 0, runway.get_pt(0, 1) );
    runway_half.add_node( 0, runway.get_pt(0, 2) );
    runway_half.add_node( 0, runway.get_pt(0, 5) );
    }

    // we add 0.5m to the length for texture overlap.  This puts the
    // lines on the texture back to the edge of the runway where they
    // belong.
    double length = rwy.length / 2.0 + 0.5;
    double start1_pct = 0.0;
    double end1_pct = 0.0;
    double heading = 0.0;

    if (rwhalf == 0) {
            heading = rwy.heading + 180.0;
    }
    else if (rwhalf == 1) {
            heading = rwy.heading;
    }
    SG_LOG( SG_GENERAL, SG_INFO, "runway marking = " << rwy.marking[rwhalf] );

    // Displaced threshold if it exists
    if ( rwy.threshold[rwhalf] > 0.0 ) {
        SG_LOG( SG_GENERAL, SG_INFO, "Displaced threshold for RW side " << rwhalf << " is "
                << rwy.threshold[rwhalf] );

        start1_pct = end1_pct;
        end1_pct = start1_pct + ( rwy.threshold[rwhalf] / length );
        Runway::gen_runway_section( runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, "",
                            rwy_polys, texparams, accum );
    }
        // Generate runway
        Runway::gen_runway_section( runway_half,
                            0, 1,
                            0.0, 1.0,
                            0.0, 0.28, 0.0, 1.0,
                            heading,
                            material, "",
                            rwy_polys, texparams, accum );

}

}
