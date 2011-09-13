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
// $Id: rwy_gen.cxx,v 1.18 2004-11-19 22:25:49 curt Exp $
//

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>

#include "rwy_common.hxx"

#include <stdlib.h>

using std::string;


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

    // runway half "a" (actually the reverse half)
    TGPolygon runway_a;
    runway_a.erase();
    runway_a.add_node( 0, runway.get_pt(0, 0) );
    runway_a.add_node( 0, runway.get_pt(0, 1) );
    runway_a.add_node( 0, runway.get_pt(0, 2) );
    runway_a.add_node( 0, runway.get_pt(0, 5) );


    // runway half "b" (actually the forward half)
    TGPolygon runway_b;
    runway_b.erase();
    runway_b.add_node( 0, runway.get_pt(0, 3) );
    runway_b.add_node( 0, runway.get_pt(0, 4) );
    runway_b.add_node( 0, runway.get_pt(0, 5) );
    runway_b.add_node( 0, runway.get_pt(0, 2) );
	
    Point3D p;
    SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway pts (a half)");
    for ( i = 0; i < runway_a.contour_size( 0 ); ++i ) {
	p = runway_a.get_pt(0, i);
	SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    }
    SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway pts (b half)");
    for ( i = 0; i < runway_b.contour_size( 0 ); ++i ) {
	p = runway_b.get_pt(0, i);
	SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    }

    //
    // Setup some variables and values to help us chop up the runway
    // into its various sections
    //

    TGSuperPoly sp;
    TGTexParams tp;

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
    double start2_pct = 0.0;
    double end1_pct = 0.0;
    double end2_pct = 0.0;

    // 
    // Displaced threshold if it exists
    //

    if ( rwy_info.disp_thresh1 > 0.0 ) {
        SG_LOG( SG_GENERAL, SG_INFO, "Forward displaced threshold = " 
                << rwy_info.disp_thresh1 );

        // reserve 90' for final arrows
        double thresh = rwy_info.disp_thresh1 - 90.0;

        // number of full center arrows
        int count = (int)(thresh / 200.0);

        // length of starting partial arrow
        double part_len = thresh - ( count * 200.0 );
        double tex_pct = (200.0 - part_len) / 200.0;

        // starting (possibly partial chunk)
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( part_len / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, tex_pct, 1.0,
                            rwy_info.heading + 180.0,
                            material, "dspl_thresh",
                            rwy_polys, texparams, accum );

        // main chunks
        for ( i = 0; i < count; ++i ) {
            start2_pct = end2_pct;
            end2_pct = start2_pct + ( 200.0 / length );
            gen_runway_section( rwy_info, runway_b,
                                start2_pct, end2_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                rwy_info.heading + 180.0,
                                material, "dspl_thresh",
                                rwy_polys, texparams, accum );
        }

        // final arrows
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 90.0 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "dspl_arrows",
                            rwy_polys, texparams, accum );
    }

    if ( rwy_info.disp_thresh2 > 0.0 ) {
        SG_LOG( SG_GENERAL, SG_INFO, "Reverse displaced threshold = " 
                << rwy_info.disp_thresh2 );

        // reserve 90' for final arrows
        double thresh = rwy_info.disp_thresh2 - 90.0;

        // number of full center arrows
        int count = (int)(thresh / 200.0);

        // length of starting partial arrow
        double part_len = thresh - ( count * 200.0 );
        double tex_pct = (200.0 - part_len) / 200.0;

        // starting (possibly partial chunk)
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( part_len / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, tex_pct, 1.0,
                            rwy_info.heading,
                            material, "dspl_thresh",
                            rwy_polys, texparams, accum );

        // main chunks
        for ( i = 0; i < count; ++i ) {
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( 200.0 / length );
            gen_runway_section( rwy_info, runway_a,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                rwy_info.heading,
                                material, "dspl_thresh",
                                rwy_polys, texparams, accum );
        }

        // final arrows
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 90.0 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "dspl_arrows",
                            rwy_polys, texparams, accum );
    }

    //
    // Threshold
    //

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 202.0 / length );
    gen_runway_section( rwy_info, runway_a,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			rwy_info.heading,
			material, "threshold",
			rwy_polys, texparams, accum );

    start2_pct = end2_pct;
    end2_pct = start2_pct + ( 202.0 / length );
    gen_runway_section( rwy_info, runway_b,
			start2_pct, end2_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "threshold",
			rwy_polys, texparams, accum );

    //
    // Runway designation letter
    //

    int len = rwy_info.rwy_no1.length();
    string letter1 = "";
    string letter2 = "";
    for ( i = 0; i < len; ++i ) {
	string tmp = rwy_info.rwy_no1.substr(i, 1);
	if ( tmp == "L" ) {
	    letter1 = "L";
	} else if ( tmp == "R" ) {
	    letter1 = "R";
	} else if ( tmp == "C" ) {
	    letter1 = "C";
	}
    }

    len = rwy_info.rwy_no2.length();
    for ( i = 0; i < len; ++i ) {
	string tmp = rwy_info.rwy_no2.substr(i, 1);
	if ( tmp == "L" ) {
	    letter2 = "L";
	} else if ( tmp == "R" ) {
	    letter2 = "R";
	} else if ( tmp == "C" ) {
	    letter2 = "C";
	}
    }
	    
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation1 = " << rwy_info.rwy_no1);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation letter1 = " << letter1);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation2 = " << rwy_info.rwy_no2);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation letter2 = " << letter2);

    //TODO: add empty airport end generation
    if ( !letter1.empty() && !letter2.empty() ) {
	start1_pct = end1_pct;
	end1_pct = start1_pct + ( 90.0 / length );
	gen_runway_section( rwy_info, runway_a,
			    start1_pct, end1_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    rwy_info.heading,
			    material, letter2,
			    rwy_polys, texparams, accum );

	start2_pct = end2_pct;
	end2_pct = start2_pct + ( 90.0 / length );
	gen_runway_section( rwy_info, runway_b,
			    start2_pct, end2_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    rwy_info.heading + 180.0,
			    material, letter1,
			    rwy_polys, texparams, accum );
    }

    //
    // Runway designation number(s)
    //

    len = rwy_info.rwy_no1.length();
    string snum1 = rwy_info.rwy_no1;
    for ( i = 0; i < len; ++i ) {
	string tmp = rwy_info.rwy_no1.substr(i, 1);
	if ( tmp == "L" || tmp == "R" || tmp == "C" || tmp == " " ) {
	    snum1 = rwy_info.rwy_no1.substr(0, i);
	}
    }

    len = rwy_info.rwy_no2.length();
    string snum2 = rwy_info.rwy_no2;
    for ( i = 0; i < len; ++i ) {
	string tmp = rwy_info.rwy_no2.substr(i, 1);
	if ( tmp == "L" || tmp == "R" || tmp == "C" || tmp == " " ) {
	    snum2 = rwy_info.rwy_no2.substr(0, i);
	}
    }
    SG_LOG(SG_GENERAL, SG_INFO, "Runway num1 = '" << snum1 << "'");
    SG_LOG(SG_GENERAL, SG_INFO, "Runway num2 = '" << snum2 << "'");

    int num1 = atoi( snum1.c_str() );
    int num2 = atoi( snum2.c_str() );
    while ( num1 <= 0 ) {
        num1 += 36;
    }

    start2_pct = end2_pct;
    end2_pct = start2_pct + ( 80.0 / length );
    gen_number_block( rwy_info, material, runway_b, rwy_info.heading + 180.0,
		      num1, start2_pct, end2_pct, rwy_polys, texparams, accum );

    while ( num2 <= 0 ) {
	num2 += 36;
    }

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 80.0 / length );
    gen_number_block( rwy_info, material, runway_a, rwy_info.heading,
		      num2, start1_pct, end1_pct, rwy_polys, texparams, accum );

    //
    // Touch down zone x3
    //

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 380 / length );
    gen_runway_section( rwy_info, runway_a,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			rwy_info.heading,
			material, "tz_three",
			rwy_polys, texparams, accum );

    start2_pct = end2_pct;
    end2_pct = start2_pct + ( 380 / length );
    gen_runway_section( rwy_info, runway_b,
			start2_pct, end2_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "tz_three",
			rwy_polys, texparams, accum );

    // add a section of center stripe
    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_a,
                        start1_pct, end1_pct,
                        0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
                        rwy_info.heading,
                        material, "rest",
                        rwy_polys, texparams, accum );

    start2_pct = end2_pct;
    end2_pct = start2_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_b,
                        start2_pct, end2_pct,
                        0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
                        rwy_info.heading + 180.0,
                        material, "rest",
                        rwy_polys, texparams, accum );
   
    //
    // Aiming point
    //

    start1_pct = end1_pct;
    end1_pct = start1_pct + ( 400 / length );
    gen_runway_section( rwy_info, runway_a,
			start1_pct, end1_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			rwy_info.heading,
			material, "aim",
			rwy_polys, texparams, accum );

    start2_pct = end2_pct;
    end2_pct = start2_pct + ( 400 / length );
    gen_runway_section( rwy_info, runway_b,
			start2_pct, end2_pct,
			0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "aim",
			rwy_polys, texparams, accum );

    //
    // Touch down zone x2 (first)
    //

    if ( end1_pct < 1.0 ) {
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 400 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "tz_two_a",
                            rwy_polys, texparams, accum );
    }

    if ( end2_pct < 1.0 ) {
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 400 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "tz_two_a",
                            rwy_polys, texparams, accum );
    }

    // add a section of center stripe
    if ( end1_pct < 1.0 ) {
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "rest",
                            rwy_polys, texparams, accum );
    }

    if ( end2_pct < 1.0 ) {
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "rest",
                            rwy_polys, texparams, accum );
    }

    //
    // Touch down zone x2 (second)
    //

    if ( end1_pct < 1.0 ) {
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "tz_two_b",
                            rwy_polys, texparams, accum );
    }

    if ( end2_pct < 1.0 ) {
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "tz_two_b",
                            rwy_polys, texparams, accum );
    }

    // add a section of center stripe
    if ( end1_pct < 1.0 ) {
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "rest",
                            rwy_polys, texparams, accum );
    }

    if ( end2_pct < 1.0 ) {
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "rest",
                            rwy_polys, texparams, accum );
    }

    //
    // Touch down zone x1 (first)
    //

    if ( end1_pct < 1.0 ) {
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 400 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "tz_one_a",
                            rwy_polys, texparams, accum );
    }

    if ( end2_pct < 1.0 ) {
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 400 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "tz_one_a",
                            rwy_polys, texparams, accum );
    }

    // add a section of center stripe
    if ( end1_pct < 1.0 ) {
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "rest",
                            rwy_polys, texparams, accum );
    }

    if ( end2_pct < 1.0 ) {
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "rest",
                            rwy_polys, texparams, accum );
    }

    //
    // Touch down zone x1 (second)
    //

    if ( end1_pct < 1.0 ) {
        start1_pct = end1_pct;
        end1_pct = start1_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_a,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading,
                            material, "tz_one_b",
                            rwy_polys, texparams, accum );
    }

    if ( end2_pct < 1.0 ) {
        start2_pct = end2_pct;
        end2_pct = start2_pct + ( 200 / length );
        gen_runway_section( rwy_info, runway_b,
                            start2_pct, end2_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            rwy_info.heading + 180.0,
                            material, "tz_one_b",
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

	gen_runway_section( rwy_info, runway_a,
			    start1_pct, end1_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    rwy_info.heading,
			    material, "rest",
			    rwy_polys, texparams, accum );
    }

    ideal_rest_inc = ( 200.0 / length );
    divs = (int)((1.0 - end2_pct) / ideal_rest_inc) + 1;
    double rest2_inc = (1.0 - end2_pct) / divs;

    while ( end2_pct < 1.0 ) {
	start2_pct = end2_pct;
	end2_pct = start2_pct + rest2_inc;

	gen_runway_section( rwy_info, runway_b,
			    start2_pct, end2_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    rwy_info.heading + 180.0,
			    material, "rest",
			    rwy_polys, texparams, accum );
    }
    
    gen_runway_stopway( rwy_info, runway_a, runway_b,
    	    		material,
    	    		rwy_polys, texparams, accum );
}
