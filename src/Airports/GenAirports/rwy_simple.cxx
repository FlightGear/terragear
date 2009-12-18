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
// $Id: rwy_simple.cxx,v 1.12 2004-11-19 22:25:49 curt Exp $
//


#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>

#include "poly_extra.hxx"
#include "rwy_common.hxx"
#include "texparams.hxx"
#include "rwy_nonprec.hxx"


// generate a simple runway.  The routine modifies rwy_polys,
// texparams, and accum
void gen_simple_rwy( const TGRunway& rwy_info,
                     double alt_m,
                     const string& material,
		     superpoly_list *rwy_polys,
		     texparams_list *texparams,
		     TGPolygon *accum )
{
    int j, k;

    TGPolygon runway = gen_runway_w_mid( rwy_info, alt_m, 0.0, 0.0 );

    // runway half "a"
    TGPolygon runway_a;
    runway_a.erase();
    runway_a.add_node( 0, runway.get_pt(0, 0) );
    runway_a.add_node( 0, runway.get_pt(0, 1) );
    runway_a.add_node( 0, runway.get_pt(0, 2) );
    runway_a.add_node( 0, runway.get_pt(0, 5) );

    // runway half "b"
    TGPolygon runway_b;
    runway_b.erase();
    runway_b.add_node( 0, runway.get_pt(0, 5) );
    runway_b.add_node( 0, runway.get_pt(0, 2) );
    runway_b.add_node( 0, runway.get_pt(0, 3) );
    runway_b.add_node( 0, runway.get_pt(0, 4) );
	
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
	
    TGSuperPoly sp;
    TGTexParams tp;

    TGPolygon clipped_a = tgPolygonDiff( runway_a, *accum );
    TGPolygon split_a = tgPolygonSplitLongEdges( clipped_a, 400.0 );
    sp.erase();
    sp.set_poly( split_a );
    sp.set_material( material );
    rwy_polys->push_back( sp );
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped_a = " << clipped_a.contours());
    *accum = tgPolygonUnion( runway_a, *accum );
    tp = TGTexParams( runway_a.get_pt(0,0),
		      rwy_info.width * SG_FEET_TO_METER,
		      rwy_info.length * SG_FEET_TO_METER / 2.0,
                      rwy_info.heading );
    texparams->push_back( tp );

    TGPolygon clipped_b = tgPolygonDiff( runway_b, *accum );
    TGPolygon split_b = tgPolygonSplitLongEdges( clipped_b, 400.0 );
    sp.erase();
    sp.set_poly( split_b );
    sp.set_material( material );
    rwy_polys->push_back( sp );
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped_b = " << clipped_b.contours());
    *accum = tgPolygonUnion( runway_b, *accum );
    tp = TGTexParams( runway_b.get_pt(0,2),
		      rwy_info.width * SG_FEET_TO_METER,
		      rwy_info.length * SG_FEET_TO_METER / 2.0,
                      rwy_info.heading + 180.0 );
    texparams->push_back( tp );

#if 0
    // after clip, but before removing T intersections
    char tmpa[256], tmpb[256];
    sprintf( tmpa, "a%d", i );
    sprintf( tmpb, "b%d", i );
    write_polygon( clipped_a, tmpa );
    write_polygon( clipped_b, tmpb );
#endif

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped runway pts (a)");
    for ( j = 0; j < clipped_a.contours(); ++j ) {
	for ( k = 0; k < clipped_a.contour_size( j ); ++k ) {
	    p = clipped_a.get_pt(j, k);
	    SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
	}
    }

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped runway pts (b)");
    for ( j = 0; j < clipped_b.contours(); ++j ) {
	for ( k = 0; k < clipped_b.contour_size( j ); ++k ) {
	    p = clipped_b.get_pt(j, k);
	    SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
	}
    }
    
    gen_runway_stopway( rwy_info, runway_a, runway_b,
    	    		material,
    	    		rwy_polys, texparams, accum );
}
