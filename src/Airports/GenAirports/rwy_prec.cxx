// rwy_prec.cxx -- Build a precision runway
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

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>

#include "rwy_common.hxx"
#include "rwy_nonprec.hxx"


// generate a precision approach runway.  The routine modifies
// rwy_polys, texparams, and accum.  For specific details and
// dimensions of precision runway markings, please refer to FAA
// document AC 150/5340-1H

void gen_precision_rwy( const FGRunway& rwy_info,
                        double alt_m,
			const string& material,
			superpoly_list *rwy_polys,
			texparams_list *texparams,
			FGPolygon *accum )
{

    //
    // Generate the basic runway outlines
    //

    int i, j;

    FGPolygon runway = gen_runway_w_mid( rwy_info, alt_m,
                                         2 * SG_FEET_TO_METER,
                                         2 * SG_FEET_TO_METER );

    // runway half "a"
    FGPolygon runway_a;
    runway_a.erase();
    runway_a.add_node( 0, runway.get_pt(0, 0) );
    runway_a.add_node( 0, runway.get_pt(0, 1) );
    runway_a.add_node( 0, runway.get_pt(0, 2) );
    runway_a.add_node( 0, runway.get_pt(0, 5) );


    // runway half "b"
    FGPolygon runway_b;
    runway_b.erase();
    runway_b.add_node( 0, runway.get_pt(0, 3) );
    runway_b.add_node( 0, runway.get_pt(0, 4) );
    runway_b.add_node( 0, runway.get_pt(0, 5) );
    runway_b.add_node( 0, runway.get_pt(0, 2) );
	
    Point3D p;
    SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway pts (a half)");
    for ( j = 0; j < runway_a.contour_size( 0 ); ++j ) {
	p = runway_a.get_pt(0, j);
	SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    }
    SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway pts (b half)");
    for ( j = 0; j < runway_b.contour_size( 0 ); ++j ) {
	p = runway_b.get_pt(0, j);
	SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    }

    //
    // Setup some variables and values to help us chop up the runway
    // into its various sections
    //

    FGSuperPoly sp;
    FGTexParams tp;

    // we add 2' to the length for texture overlap.  This puts the
    // lines on the texture back to the edge of the runway where they
    // belong.
    double length = rwy_info.length / 2.0 + 2.0;
    if ( length < 3075 ) {
        SG_LOG( SG_GENERAL, SG_ALERT,
	        "Runway " << rwy_info.rwy_no << " is not long enough ("
                << rwy_info.length << ") for precision markings!");
    }

    double start_pct = 0;
    double end_pct = 0;

    //
    // Threshold
    //

    end_pct = start_pct + ( 192.0 / length );
    gen_runway_section( rwy_info, runway_a,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading,
			material, "threshold",
			rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "threshold",
			rwy_polys, texparams, accum );

    //
    // Runway designation letter
    //

    int len = rwy_info.rwy_no.length();
    string letter = "";
    string rev_letter = "";
    for ( i = 0; i < len; ++i ) {
	string tmp = rwy_info.rwy_no.substr(i, 1);
	if ( tmp == "L" ) {
	    letter = "L";
	    rev_letter = "R";
	} else if ( tmp == "R" ) {
	    letter = "R";
	    rev_letter = "L";
	} else if ( tmp == "C" ) {
	    letter == "C";
	    rev_letter = "C";
	}
    }
	    
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation = " << rwy_info.rwy_no);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation letter = " << letter);

    if ( letter != "" ) {
	start_pct = end_pct;
	end_pct = start_pct + ( 90.0 / length );
	gen_runway_section( rwy_info, runway_a,
			    start_pct, end_pct,
			    0.0, 1.0,
			    rwy_info.heading,
			    material, rev_letter,
			    rwy_polys, texparams, accum );

	gen_runway_section( rwy_info, runway_b,
			    start_pct, end_pct,
			    0.0, 1.0,
			    rwy_info.heading + 180.0,
			    material, letter,
			    rwy_polys, texparams, accum );
    }

    //
    // Runway designation number(s)
    //

    start_pct = end_pct;
    end_pct = start_pct + ( 90.0 / length );

    len = rwy_info.rwy_no.length();
    string snum = rwy_info.rwy_no;
    for ( i = 0; i < len; ++i ) {
	string tmp = rwy_info.rwy_no.substr(i, 1);
	if ( tmp == "L" || tmp == "R" || tmp == "C" || tmp == " " ) {
	    snum = rwy_info.rwy_no.substr(0, i);
	}
    }
    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway num = '" << snum << "'");
    int num = atoi( snum.c_str() );
    while ( num <= 0 ) {
        num += 36;
    }

    gen_number_block( rwy_info, material, runway_b, rwy_info.heading + 180.0,
		      num, start_pct, end_pct, rwy_polys, texparams, accum );

    num += 18;
    while ( num > 36 ) {
	num -= 36;
    }

    gen_number_block( rwy_info, material, runway_a, rwy_info.heading,
		      num, start_pct, end_pct, rwy_polys, texparams, accum );

    //
    // Touch down zone x3
    //

    start_pct = end_pct;
    end_pct = start_pct + ( 380 / length );
    gen_runway_section( rwy_info, runway_a,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading,
			material, "tz_three",
			rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "tz_three",
			rwy_polys, texparams, accum );

    // add a section of center stripe
    start_pct = end_pct;
    end_pct = start_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_a,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading,
                        material, "rest",
                        rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading + 180.0,
                        material, "rest",
                        rwy_polys, texparams, accum );
   
    //
    // Aiming point
    //

    start_pct = end_pct;
    end_pct = start_pct + ( 400 / length );
    gen_runway_section( rwy_info, runway_a,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading,
			material, "aim",
			rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "aim",
			rwy_polys, texparams, accum );

    //
    // Touch down zone x2 (first)
    //

    if ( end_pct >= 1.0 ) {
	return;
    }

    start_pct = end_pct;
    end_pct = start_pct + ( 400 / length );
    gen_runway_section( rwy_info, runway_a,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading,
			material, "tz_two_a",
			rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "tz_two_a",
			rwy_polys, texparams, accum );

    // add a section of center stripe
    start_pct = end_pct;
    end_pct = start_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_a,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading,
                        material, "rest",
                        rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading + 180.0,
                        material, "rest",
                        rwy_polys, texparams, accum );
   
    //
    // Touch down zone x2 (second)
    //

    if ( end_pct >= 1.0 ) {
	return;
    }

    start_pct = end_pct;
    end_pct = start_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_a,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading,
			material, "tz_two_b",
			rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "tz_two_b",
			rwy_polys, texparams, accum );

    // add a section of center stripe
    start_pct = end_pct;
    end_pct = start_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_a,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading,
                        material, "rest",
                        rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading + 180.0,
                        material, "rest",
                        rwy_polys, texparams, accum );
   
    //
    // Touch down zone x1 (first)
    //

    if ( end_pct >= 1.0 ) {
	return;
    }

    start_pct = end_pct;
    end_pct = start_pct + ( 400 / length );
    gen_runway_section( rwy_info, runway_a,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading,
			material, "tz_one_a",
			rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "tz_one_a",
			rwy_polys, texparams, accum );

    // add a section of center stripe
    start_pct = end_pct;
    end_pct = start_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_a,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading,
                        material, "rest",
                        rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
                        start_pct, end_pct,
                        0.0, 1.0,
                        rwy_info.heading + 180.0,
                        material, "rest",
                        rwy_polys, texparams, accum );
   
    //
    // Touch down zone x1 (second)
    //

    if ( end_pct >= 1.0 ) {
	return;
    }

    start_pct = end_pct;
    end_pct = start_pct + ( 200 / length );
    gen_runway_section( rwy_info, runway_a,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading,
			material, "tz_one_b",
			rwy_polys, texparams, accum );

    gen_runway_section( rwy_info, runway_b,
			start_pct, end_pct,
			0.0, 1.0,
			rwy_info.heading + 180.0,
			material, "tz_one_b",
			rwy_polys, texparams, accum );

    //
    // The rest ...
    //

    // fit the 'rest' texture in as many times as will go evenly into
    // the remaining distance so we don't end up with a super short
    // section at the end.
    double ideal_rest_inc = ( 200.0 / length );
    int divs = (int)((1.0 - end_pct) / ideal_rest_inc) + 1;
    double rest_inc = (1.0 - end_pct) / divs;

    while ( end_pct < 1.0 ) {
	start_pct = end_pct;
	end_pct = start_pct + rest_inc;

	gen_runway_section( rwy_info, runway_a,
			    start_pct, end_pct,
			    0.0, 1.0,
			    rwy_info.heading,
			    material, "rest",
			    rwy_polys, texparams, accum );

	gen_runway_section( rwy_info, runway_b,
			    start_pct, end_pct,
			    0.0, 1.0,
			    rwy_info.heading + 180.0,
			    material, "rest",
			    rwy_polys, texparams, accum );
    }
}
