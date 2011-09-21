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

struct marking
{
    const char* tex;
    int size;
};

void gen_prec_marking( const TGRunway& rwy_info,
           const TGPolygon& runway,
	   double &start1_pct, double &end1_pct,
	   double heading,
	   const string& material,
	   superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         TGPolygon *accum) {

    // Precision runway sections from after the designation number
    // onwards to the middle (one half).
    // Set order of sections and their corresponding size
    static const struct marking prec_rw[] = {
						{ "tz_three", 380 },
						{ "rest", 200 },
						{ "aim", 400 },
						{ "tz_two_a", 400 },
						{ "rest", 200 },
						{ "tz_two_b", 200 },
						{ "rest", 200 },
						{ "tz_one_a", 400 },
						{ "rest", 200 },
						{ "tz_one_b", 200 }
					   };

    double length = rwy_info.length / 2.0 + 2.0;

    for ( int i=0; i < sizeof prec_rw / sizeof prec_rw[0]; ++i) {
    SG_LOG(SG_GENERAL, SG_INFO, "PREC_RW = " << prec_rw[i].tex << " lenght: " << prec_rw[i].size);
    start1_pct = end1_pct;
    end1_pct = start1_pct + ( prec_rw[i].size / length );
    gen_runway_section( rwy_info, runway,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			heading,
			material, prec_rw[i].tex,
			rwy_polys, texparams, accum );
    }

}

void gen_non_prec_marking( const TGRunway& rwy_info,
           const TGPolygon& runway,
	   double &start1_pct, double &end1_pct,
	   double heading,
	   const string& material,
	   superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         TGPolygon *accum) {

    // Non-precision runway sections from after the designation number
    // onwards to the middle (one half).
    // Set order of sections and their corresponding size
    static const struct marking non_prec_rw[] = {
						{ "centerline", 200 },
						{ "aim", 400 }
					   };

    double length = rwy_info.length / 2.0 + 2.0;

    for ( int i=0; i < sizeof non_prec_rw / sizeof non_prec_rw[0]; ++i) {
    SG_LOG(SG_GENERAL, SG_INFO, "NON_PREC_RW = " << non_prec_rw[i].tex << " lenght: " << non_prec_rw[i].size);
    start1_pct = end1_pct;
    end1_pct = start1_pct + ( non_prec_rw[i].size / length );
    gen_runway_section( rwy_info, runway,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			heading,
			material, non_prec_rw[i].tex,
			rwy_polys, texparams, accum );
    }

}
// generate a runway.  The routine modifies
// rwy_polys, texparams, and accum.  For specific details and
// dimensions of precision runway markings, please refer to FAA
// document AC 150/5340-1H

void gen_rwy( const TGRunway& rwy_info,
                        double alt_m,
			const string& material,
			superpoly_list *rwy_polys,
			texparams_list *texparams,
			TGPolygon *accum )
{
    SG_LOG( SG_GENERAL, SG_INFO, "Building runway = " << rwy_info.rwy_no1 << " / " << rwy_info.rwy_no2);

    //
    // Generate the basic runway outlines
    //

    int i;

    TGPolygon runway = gen_runway_w_mid( rwy_info, alt_m,
                                         2 * SG_FEET_TO_METER,
                                         2 * SG_FEET_TO_METER );

    TGPolygon runway_half;

for ( int rwhalf=1; rwhalf<3; ++rwhalf ){

    if (rwhalf == 1) {

    //Create first half of the runway (first entry in apt.dat)
    // runway half "b" (actually the first half)
    runway_half.erase();
    runway_half.add_node( 0, runway.get_pt(0, 3) );
    runway_half.add_node( 0, runway.get_pt(0, 4) );
    runway_half.add_node( 0, runway.get_pt(0, 5) );
    runway_half.add_node( 0, runway.get_pt(0, 2) );
    }

    else if (rwhalf == 2) {

     // runway half "a" (actually the second half in apt.dat)
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

    // we add 2' to the length for texture overlap.  This puts the
    // lines on the texture back to the edge of the runway where they
    // belong.
    double length = rwy_info.length / 2.0 + 2.0;
    if ( length < 3075 ) {
        SG_LOG( SG_GENERAL, SG_ALERT,
	        "Runway " << rwy_info.rwy_no1 << " is not long enough ("
                << rwy_info.length << ") for precision markings!");
    }

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
	    disp_thresh = rwy_info.disp_thresh1;
	    heading = rwy_info.heading + 180.0;
	    rwname = rwy_info.rwy_no1;
	    stopway = rwy_info.stopway1;
    }
    else if (rwhalf == 2) {
	    disp_thresh = rwy_info.disp_thresh2;
	    heading = rwy_info.heading;
	    rwname = rwy_info.rwy_no2;
	    stopway = rwy_info.stopway2;
    }
    
    if ( disp_thresh > 0.0 ) {
        SG_LOG( SG_GENERAL, SG_INFO, "Displaced threshold for RW side " << rwhalf << " is "
                << disp_thresh );

        // reserve 90' for final arrows
        double thresh = disp_thresh - 90.0;

        // number of full center arrows
        int count = (int)(thresh / 200.0);

        // length of starting partial arrow
        double part_len = thresh - ( count * 200.0 );
        double tex_pct = (200.0 - part_len) / 200.0;

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
            end1_pct = start1_pct + ( 200.0 / length );
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
        end1_pct = start1_pct + ( 90.0 / length );
        gen_runway_section( rwy_info, runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, "dspl_arrows",
                            rwy_polys, texparams, accum );
    }


 if ((rwhalf == 1 && !rwy_info.marking_code1 == 0) || (rwhalf == 2 && !rwy_info.marking_code2 == 0)){

    //
    // Threshold
    //

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 202.0 / length );
    gen_runway_section( rwy_info, runway_half,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			heading,
			material, "threshold",
			rwy_polys, texparams, accum );


    //
    // Runway designation letter
    //

    int len = rwname.length();
    string letter = "";
    for ( i = 0; i < len; ++i ) {
	string tmp = rwname.substr(i, 1);
	if ( tmp == "L" ) {
	    letter = "L";
	} else if ( tmp == "R" ) {
	    letter = "R";
	} else if ( tmp == "C" ) {
	    letter = "C";
	}
    }

	    
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation1 = " << rwname);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation letter1 = " << letter);

    if ( !letter.empty() ) {
	start1_pct = end1_pct;
	end1_pct = start1_pct + ( 90.0 / length );
	gen_runway_section( rwy_info, runway_half,
			    start1_pct, end1_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    heading,
			    material, letter,
			    rwy_polys, texparams, accum );
    }

    //
    // Runway designation number(s)
    //

    len = rwname.length();
    string snum = rwname;
    for ( i = 0; i < len; ++i ) {
	string tmp = rwname.substr(i, 1);
	if ( tmp == "L" || tmp == "R" || tmp == "C" || tmp == " " ) {
	    snum = rwname.substr(0, i);
	}
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Runway num = '" << snum );

    int num = atoi( snum.c_str() );
    while ( num <= 0 ) {
        num += 36;
    }

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 80.0 / length );
    gen_number_block( rwy_info, material, runway_half, heading,
		      num, start1_pct, end1_pct, rwy_polys, texparams, accum );
 }


    if ((rwhalf == 1 && rwy_info.marking_code1 == 3) || (rwhalf == 2 && rwy_info.marking_code2 == 3)){
    gen_prec_marking( rwy_info, runway_half,
		 start1_pct, end1_pct,
		 heading, material,
		 rwy_polys, texparams, accum );
    }

    if ((rwhalf == 1 && rwy_info.marking_code1 == 2) || (rwhalf == 2 && rwy_info.marking_code2 == 2)){
    gen_non_prec_marking( rwy_info, runway_half,
		 start1_pct, end1_pct,
		 heading, material,
		 rwy_polys, texparams, accum );
    }


    //
    // The rest ...
    //

    // fit the 'rest' texture in as many times as will go evenly into
    // the remaining distance so we don't end up with a super short
    // section at the end.
    double ideal_rest_inc = ( 200.0 / length );
    int divs = (int)((1.0 - end1_pct) / ideal_rest_inc) + 1;
    double rest1_inc = (1.0 - end1_pct) / divs;

    while ( end1_pct < 1.0 ) {
	start1_pct = end1_pct;
	end1_pct = start1_pct + rest1_inc;

	gen_runway_section( rwy_info, runway_half,
			    start1_pct, end1_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    heading,
			    material, "rest",
			    rwy_polys, texparams, accum );
    }

    gen_runway_stopway( rwy_info, runway_half, rwhalf,
                       material,
                       rwy_polys, texparams, accum );


}


}
