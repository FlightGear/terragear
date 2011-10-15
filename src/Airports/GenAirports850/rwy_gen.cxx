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
#include "runway.hxx"

#include <stdlib.h>

using std::string;

struct sections
{
    const char* tex;
    int size;
};

void Runway::gen_rw_marking( const TGPolygon& runway,
	   double &start1_pct, double &end1_pct,
	   double heading,
	   const string& material,
	   superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         ClipPolyType *accum, int marking) {

    std::vector<sections> rw_marking_list;

    if (marking == 5){
    // UK Precision runway sections from after the designation number
    // onwards to the middle (one half).
    // Set order of sections and their corresponding size
    static const struct sections uk_prec[] = {
	    { "tz_one_a", 380 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "aim_uk", 400 * SG_FEET_TO_METER },
	    { "tz_one_a", 400 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "tz_one_b", 200 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "tz_one_a", 400 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "tz_one_b", 200 * SG_FEET_TO_METER }
	    };
	    rw_marking_list.clear();
	    rw_marking_list.insert(  rw_marking_list.begin(), uk_prec, uk_prec + sizeof(uk_prec) / sizeof(uk_prec[0]) );
    }

    else if (marking == 4){
    // UK non-precision runway sections from after the designation number
    // onwards to the middle (one half).
    // Set order of sections and their corresponding size
    static const struct sections uk_nprec[] = {
	    { "centerline", 200 * SG_FEET_TO_METER },
	    { "aim_uk", 400 * SG_FEET_TO_METER },
	    };
	    rw_marking_list.clear();
	    rw_marking_list.insert(  rw_marking_list.begin(), uk_nprec, uk_nprec + sizeof(uk_nprec) / sizeof(uk_nprec[0]) );
    }

    else if (marking == 3){
    // Precision runway sections from after the designation number
    // onwards to the middle (one half).
    // Set order of sections and their corresponding size
    static const struct sections prec[] = {
	    { "tz_three", 380 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "aim", 400 * SG_FEET_TO_METER },
	    { "tz_two_a", 400 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "tz_two_b", 200 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "tz_one_a", 400 * SG_FEET_TO_METER },
	    { "rest", 200 * SG_FEET_TO_METER },
	    { "tz_one_b", 200 * SG_FEET_TO_METER }
	    };
	    rw_marking_list.clear();
	    rw_marking_list.insert(  rw_marking_list.begin(), prec, prec + sizeof(prec) / sizeof(prec[0]) );
    }

    else if (marking == 2){
    // Non-precision runway sections from after the designation number
    // onwards to the middle (one half).
    // Set order of sections and their corresponding size
    static const struct sections nprec[] = {
	    { "centerline", 200 * SG_FEET_TO_METER },
	    { "aim", 400 * SG_FEET_TO_METER }
	    };
	    rw_marking_list.clear();
	    rw_marking_list.insert(  rw_marking_list.begin(), nprec, nprec + sizeof(nprec) / sizeof(nprec[0]) );
    }

    //Now create the sections of the runway type
    double length = rwy.length / 2.0;

    for ( int i=0; i < rw_marking_list.size(); ++i) {
	    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway section texture = " << rw_marking_list[i].tex << " lenght: " << rw_marking_list[i].size);

	    if ( end1_pct < 1.0 ) {
		    start1_pct = end1_pct;
		    end1_pct = start1_pct + ( rw_marking_list[i].size / length );
		    gen_runway_section( runway,
			start1_pct, end1_pct,
			  0.0, 1.0,
			  0.0, 1.0, 0.0, 1.0,
			  heading,
			  material, rw_marking_list[i].tex,
			  rwy_polys, texparams, accum );
	    }
    }

}


// generate a runway.  The routine modifies
// rwy_polys, texparams, and accum.  For specific details and
// dimensions of precision runway markings, please refer to FAA
// document AC 150/5340-1H

void Runway::gen_rwy( double alt_m,
			const string& material,
			superpoly_list *rwy_polys,
			texparams_list *texparams,
			ClipPolyType *accum )
{
    SG_LOG( SG_GENERAL, SG_DEBUG, "Building runway = " << rwy.rwnum[0] << " / " << rwy.rwnum[1]);

    //
    // Generate the basic runway outlines
    //

    int i;

    TGPolygon runway = gen_runway_w_mid( alt_m,
                                         2 * SG_FEET_TO_METER,
                                         2 * SG_FEET_TO_METER );

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

    Point3D p;
    SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway half pts (run " << rwhalf << ")");
    for ( i = 0; i < runway_half.contour_size( 0 ); ++i ) {
	p = runway_half.get_pt(0, i);
	SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    }

    double length = rwy.length / 2.0;
    if ( length < 3075 * SG_FEET_TO_METER ) {
        SG_LOG( SG_GENERAL, SG_DEBUG,
	        "Runway " << rwy.rwnum[0] << " is not long enough ("
                << rwy.length << ") for precision markings!");
    }

    double start1_pct = 0.0;
    double end1_pct = 0.0;
    double heading = 0.0;
    string rwname;


    //
    // Displaced threshold if it exists
    //

    if (rwhalf == 0) {
	    heading = rwy.heading + 180.0;
            rwname = rwy.rwnum[0];
    }
    else if (rwhalf == 1) {
            heading = rwy.heading;
            rwname = rwy.rwnum[1];
    }
    SG_LOG( SG_GENERAL, SG_DEBUG, "runway marking = " << rwy.marking[rwhalf] );
    if ( rwy.threshold[rwhalf] > 0.0 ) {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Displaced threshold for RW side " << rwhalf << " is "
                << rwy.threshold[rwhalf] );

        // reserve 90' for final arrows
        double thresh = rwy.threshold[rwhalf] - 90.0 * SG_FEET_TO_METER;

        // number of full center arrows
        int count = (int)(thresh / 200.0 * SG_FEET_TO_METER);

        // length of starting partial arrow
        double part_len = thresh - ( count * 200.0 * SG_FEET_TO_METER);
        double tex_pct = (200.0 * SG_FEET_TO_METER - part_len) / 200.0 * SG_FEET_TO_METER;

        // starting (possibly partial chunk)
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( part_len / length );
        gen_runway_section( runway_half,
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
            gen_runway_section( runway_half,
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
        gen_runway_section( runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, "dspl_arrows",
                            rwy_polys, texparams, accum );
    }


 if (rwy.marking[rwhalf] == 0){

    // No threshold

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 10 / length );
    gen_runway_section( runway_half,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			heading,
			material, "no_threshold",
			rwy_polys, texparams, accum );
 } else {

    // Thresholds for all others

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 202.0 * SG_FEET_TO_METER / length );
    gen_runway_section( runway_half,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			heading,
			material, "threshold",
			rwy_polys, texparams, accum );
 }

    // Runway designation block
    gen_rw_designation( material, runway_half, heading,
                        rwname, start1_pct, end1_pct, rwy_polys, texparams, accum );



    if (rwy.marking[rwhalf] > 1){
    // Generate remaining markings depending on type of runway
    gen_rw_marking( runway_half,
		 start1_pct, end1_pct,
		 heading, material,
		 rwy_polys, texparams, accum, rwy.marking[rwhalf] );
    }

    //
    // The rest ...
    //

    // fit the 'rest' texture in as many times as will go evenly into
    // the remaining distance so we don't end up with a super short
    // section at the end.
    double ideal_rest_inc = ( 200.0 * SG_FEET_TO_METER / length );
    int divs = (int)((1.0 - end1_pct) / ideal_rest_inc) + 1;
    double rest1_inc = (1.0 - end1_pct) / divs;

    while ( end1_pct < 1.0 ) {
	start1_pct = end1_pct;
	end1_pct = start1_pct + rest1_inc;

	gen_runway_section( runway_half,
			    start1_pct, end1_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    heading,
			    material, "rest",
			    rwy_polys, texparams, accum );
    }

    gen_runway_overrun( runway_half, rwhalf,
                       material,
                       rwy_polys, texparams, accum );


}
}
