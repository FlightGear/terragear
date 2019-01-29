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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//


#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>

#include "runway.hxx"
#include "debug.hxx"

using std::string;

// generate a simple runway
void Runway::gen_simple_rwy( tgpolygon_list& rwy_polys,
                             tgcontour_list& slivers,
                             tgAccumulator& accum )
{
    tgContour runway = gen_runway_w_mid( 0.0, 0.0 );
    tgPolygon runway_half;
    std::string empty = "";

    for ( int rwhalf=0; rwhalf<2; ++rwhalf ) {
        double length = rwy.length / 2.0;
        double end_pct = 0.0;
        double heading = 0.0;

        if (rwhalf == 0) {
            heading = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180.0);

            //Create the first half of the runway (first entry in apt.dat)
            runway_half.Erase();
            runway_half.AddNode( 0, runway.GetNode(3) );
            runway_half.AddNode( 0, runway.GetNode(4) );
            runway_half.AddNode( 0, runway.GetNode(5) );
            runway_half.AddNode( 0, runway.GetNode(2) );
        }
        else {
            heading = rwy.heading;

            //Create the second runway half from apt.dat
            runway_half.Erase();
            runway_half.AddNode( 0, runway.GetNode(0) );
            runway_half.AddNode( 0, runway.GetNode(1) );
            runway_half.AddNode( 0, runway.GetNode(2) );
            runway_half.AddNode( 0, runway.GetNode(5) );
        }

        TG_LOG( SG_GENERAL, SG_DEBUG, "runway marking = " << rwy.marking[rwhalf] );

        // Displaced threshold if it exists
        if ( rwy.threshold[rwhalf] > 0.0 ) {
            TG_LOG( SG_GENERAL, SG_DEBUG, "Displaced threshold for RW side " << rwhalf << " is " << rwy.threshold[rwhalf] );

            double start_pct = end_pct;
            end_pct = start_pct + ( rwy.threshold[rwhalf] / length );
            Runway::gen_runway_section( runway_half,
                                        start_pct, end_pct,
                                        0.0, 1.0,
                                        0.0, 1.0, 0.0, 1.0,
                                        heading,
                                        "",
                                        rwy_polys,
                                        slivers,
                                        accum,
                                        empty );
        }

        // Generate runway
        Runway::gen_runway_section( runway_half,
                                    end_pct, 1.0,
                                    0.0, 1.0,
                                    0.0, 0.28, 0.0, 1.0,
                                    heading,
                                    "",
                                    rwy_polys,
                                    slivers,
                                    accum,
                                    empty );
    }
}
