// taxiway.cxx -- Build a taxiway
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


#include <simgear/constants.h>

#include "poly_extra.hxx"
#include "rwy_common.hxx"
#include "taxiway.hxx"


// generate a taxiway.  The routine modifies rwy_polys, texparams, and
// accum
void gen_taxiway( const FGRunway& rwy_info, const string& material,
		  superpoly_list *rwy_polys,
		  texparams_list *texparams,
		  FGPolygon *accum )
{
    int j, k;

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
    runway_b.add_node( 0, runway.get_pt(0, 5) );
    runway_b.add_node( 0, runway.get_pt(0, 2) );
    runway_b.add_node( 0, runway.get_pt(0, 3) );
    runway_b.add_node( 0, runway.get_pt(0, 4) );
	
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
	
    FGSuperPoly sp;
    FGTexParams tp;

    double xfactor = 1.0;
    double yfactor = 1.0;
    if ( fabs(rwy_info.width) > SG_EPSILON ) {
        xfactor = 150.0 / rwy_info.width;
        cout << "xfactor = " << xfactor << endl;
    }
    if ( fabs(rwy_info.length) > SG_EPSILON ) {
        yfactor = 150.0 / rwy_info.length;
        cout << "yfactor = " << yfactor << endl;
    }

    cout << "len = " << rwy_info.length << endl;
    cout << "width = " << rwy_info.width << endl;

    FGPolygon clipped_a = polygon_diff( runway_a, *accum );
    FGPolygon split_a = split_long_edges( clipped_a, 400.0 );
    sp.erase();
    sp.set_poly( split_a );
    sp.set_material( material );
    sp.set_flag( 1 );           // mark as a taxiway
    rwy_polys->push_back( sp );
    cout << "clipped_a = " << clipped_a.contours() << endl;
    *accum = polygon_union( runway_a, *accum );
    tp = FGTexParams( Point3D( rwy_info.lon, rwy_info.lat, 0 ),
		      Point3D( (-250 / 2.0) * SG_FEET_TO_METER,
                               0.0,
			       0 ),
		      Point3D( (250 / 2.0) * SG_FEET_TO_METER,
			       (250 / 2.0) * SG_FEET_TO_METER,
			       0.0 ),
		      rwy_info.heading );
    texparams->push_back( tp );

    FGPolygon clipped_b = polygon_diff( runway_b, *accum );
    FGPolygon split_b = split_long_edges( clipped_b, 400.0 );
    sp.erase();
    sp.set_poly( split_b );
    sp.set_material( material );
    rwy_polys->push_back( sp );
    cout << "clipped_b = " << clipped_b.contours() << endl;
    *accum = polygon_union( runway_b, *accum );
    tp = FGTexParams( Point3D( rwy_info.lon, rwy_info.lat, 0 ),
		      Point3D( (-250 / 2.0) * SG_FEET_TO_METER,
			       0.0,
			       0 ),
		      Point3D( (250 / 2.0) * SG_FEET_TO_METER,
			       (250 / 2.0) * SG_FEET_TO_METER,
			       0.0 ),
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
    cout << "clipped runway pts (a)" << endl;
    for ( j = 0; j < clipped_a.contours(); ++j ) {
	for ( k = 0; k < clipped_a.contour_size( j ); ++k ) {
	    p = clipped_a.get_pt(j, k);
	    cout << " point = " << p << endl;
	}
    }

    // print runway points
    cout << "clipped runway pts (b)" << endl;
    for ( j = 0; j < clipped_b.contours(); ++j ) {
	for ( k = 0; k < clipped_b.contour_size( j ); ++k ) {
	    p = clipped_b.get_pt(j, k);
	    cout << " point = " << p << endl;
	}
    }

}
