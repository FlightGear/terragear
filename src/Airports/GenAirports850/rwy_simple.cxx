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

#include "poly_extra.hxx"
#include "rwy_common.hxx"
#include "texparams.hxx"

using std::string;


// generate a simple runway.  The routine modifies rwy_polys,
// texparams, and accum
void gen_simple_rwy( const TGRunway& rwy_info,
                     double alt_m,
                     const string& material,
		     superpoly_list *rwy_polys,
		     texparams_list *texparams,
		     TGPolygon *accum )
{
    int i;

    TGPolygon runway = gen_runway_w_mid( rwy_info, alt_m, 0.0, 0.0 );

    TGPolygon runway_half;

for ( int rwhalf=1; rwhalf<3; ++rwhalf ){

    if (rwhalf == 1) {

    //Create the first half of the runway (first entry in apt.dat)
    runway_half.erase();
    runway_half.add_node( 0, runway.get_pt(0, 3) );
    runway_half.add_node( 0, runway.get_pt(0, 4) );
    runway_half.add_node( 0, runway.get_pt(0, 5) );
    runway_half.add_node( 0, runway.get_pt(0, 2) );
    }

    else if (rwhalf == 2) {

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
    double length = rwy_info.length / 2.0 + 0.5;
    int marking = 0;
    double start1_pct = 0.0;
    double end1_pct = 0.0;
    double disp_thresh = 0.0;
    double heading = 0.0;
    double stopway = 0.0;
    string rwname;


    //
    // Displaced threshold if it exists
    //

    if (rwhalf == 1) {
            marking = rwy_info.marking_code1;
            disp_thresh = rwy_info.disp_thresh1;
            heading = rwy_info.heading + 180.0;
            rwname = rwy_info.rwy_no1;
            stopway = rwy_info.stopway1;
    }
    else if (rwhalf == 2) {
            marking = rwy_info.marking_code2;
            disp_thresh = rwy_info.disp_thresh2;
            heading = rwy_info.heading;
            rwname = rwy_info.rwy_no2;
            stopway = rwy_info.stopway2;
    }
    SG_LOG( SG_GENERAL, SG_INFO, "runway marking = " << marking );
    if ( disp_thresh > 0.0 ) {
        SG_LOG( SG_GENERAL, SG_INFO, "Displaced threshold for RW side " << rwhalf << " is "
                << disp_thresh );

        // reserve 90' for final arrows
        double thresh = disp_thresh - 90.0 * SG_FEET_TO_METER;

        // number of full center arrows
        int count = (int)(thresh / 200.0 * SG_FEET_TO_METER);

        // length of starting partial arrow
        double part_len = thresh - ( count * 200.0 * SG_FEET_TO_METER);
        double tex_pct = (200.0 * SG_FEET_TO_METER - part_len) / 200.0 * SG_FEET_TO_METER;

        // starting (possibly partial chunk)
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( part_len / length );
        gen_runway_section( rwy_info, runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, tex_pct, 1.0,
                            heading,
                            material, "dspl_thresh",
                            rwy_polys, texparams, accum );

        // main chunks
        for ( i = 0; i < count; ++i ) {
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( 200.0 * SG_FEET_TO_METER / length );
            gen_runway_section( rwy_info, runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                material, "dspl_thresh",
                                rwy_polys, texparams, accum );
        }

        // final arrows
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 90.0 * SG_FEET_TO_METER / length );
        gen_runway_section( rwy_info, runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, "dspl_arrows",
                            rwy_polys, texparams, accum );
    }

        gen_runway_section( rwy_info, runway_half,
                            0, 1,
                            0.0, 1.0,
                            0.0, 0.28, 0.0, 1.0,
                            heading,
                            material, "",
                            rwy_polys, texparams, accum );

}

}
