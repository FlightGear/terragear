// rwy_visual.cxx -- Build a visual approach runway
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


#include "rwy_common.hxx"
#include "rwy_visual.hxx"


// generate a visual approach runway.  The routine modifies rwy_polys,
// texparams, and accum.  For specific details and dimensions of
// precision runway markings, please refer to FAA document AC
// 150/5340-1H

void gen_visual_rwy( const FGRunway& rwy_info,
		     const string& material,
		     superpoly_list *rwy_polys,
		     texparams_list *texparams,
		     FGPolygon *accum )
{
    int i, j;

    //
    // Generate the basic runway outlines
    //

    FGPolygon runway = gen_runway_w_mid( rwy_info );

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
    cout << "raw runway pts (a half)" << endl;
    for ( j = 0; j < runway_a.contour_size( 0 ); ++j ) {
	p = runway_a.get_pt(0, j);
	cout << " point = " << p << endl;
    }
    cout << "raw runway pts (b half)" << endl;
    for ( j = 0; j < runway_b.contour_size( 0 ); ++j ) {
	p = runway_b.get_pt(0, j);
	cout << " point = " << p << endl;
    }

    //
    // Setup some variables and values to help us chop up the runway
    // into its various sections
    //

    FGSuperPoly sp;
    FGTexParams tp;

    double length = rwy_info.length / 2.0;
    if ( length < 1150 ) {
	cout << "This runway is not long enough for visual markings!"
	     << endl;
    }

    double start_pct = 0;
    double end_pct = 0;


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
	    
    cout << "Runway designation = " << rwy_info.rwy_no << endl;
    cout << "Runway designation letter = " << letter << endl;

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
    end_pct = start_pct + ( 100.0 / length );

    len = rwy_info.rwy_no.length();
    string snum = rwy_info.rwy_no;
    for ( i = 0; i < len; ++i ) {
	string tmp = rwy_info.rwy_no.substr(i, 1);
	if ( tmp == "L" || tmp == "R" || tmp == "C" || tmp == " " ) {
	    snum = rwy_info.rwy_no.substr(0, i);
	}
    }
    cout << "Runway num = '" << snum << "'" << endl;
    int num = atoi( snum.c_str() );

    gen_number_block( rwy_info, material, runway_b, rwy_info.heading + 180.0,
		      num, start_pct, end_pct, rwy_polys, texparams, accum );

    num += 18;
    if ( num > 36 ) {
	num -= 36;
    }

    gen_number_block( rwy_info, material, runway_a, rwy_info.heading,
		      num, start_pct, end_pct, rwy_polys, texparams, accum );

    if ( false ) {
	//
	// Intermediate area before aiming point ...
	//

	if ( end_pct >= 1.0 ) {
	    return;
	}

	start_pct = end_pct;
	end_pct = start_pct + ( 450.0 / length );
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

	if ( end_pct >= 1.0 ) {
	    return;
	}

	start_pct = end_pct;
	end_pct = start_pct + ( 450.0 / length );
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

	if ( end_pct >= 1.0 ) {
	    return;
	}

	start_pct = end_pct;
	end_pct = start_pct + ( 500 / length );
	if ( end_pct > 1.0 ) { end_pct = 1.0; }
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
    }

    //
    // The rest ...
    //

    while ( end_pct < 1.0 ) {
	start_pct = end_pct;
	end_pct = start_pct + ( 400.0 / length );

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


