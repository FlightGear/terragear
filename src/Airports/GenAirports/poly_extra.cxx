// poly_extra.cxx -- Extra polygon manipulation routines
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

#include <iostream>
SG_USING_STD(cout);
SG_USING_STD(cerr);
SG_USING_STD(endl);

#include <stdio.h>

#include <simgear/math/sg_geodesy.hxx>

#include <Geometry/poly_support.hxx>

#include "poly_extra.hxx"


// Divide segment if there are other existing points on it, return the
// new polygon
void add_intermediate_nodes( int contour, const Point3D& start, 
			     const Point3D& end, const FGTriNodes& tmp_nodes,
			     FGPolygon *result )
{
    point_list nodes = tmp_nodes.get_node_list();

    // cout << "  add_intermediate_nodes()" << endl;
    printf("   %.7f %.7f %.7f <=> %.7f %.7f %.7f\n",
	   start.x(), start.y(), start.z(), end.x(), end.y(), end.z() );

    
    Point3D new_pt;
    bool found_extra = find_intermediate_node( start, end, nodes, &new_pt );

    if ( found_extra ) {
	// recurse with two sub segments
	// cout << "dividing " << p0 << " " << nodes[extra_index]
	//      << " " << p1 << endl;
	add_intermediate_nodes( contour, start, new_pt, tmp_nodes, 
				result );

	result->add_node( contour, new_pt );

	add_intermediate_nodes( contour, new_pt, end, tmp_nodes,
				result );
    } else {
	// this segment does not need to be divided
    }
}


// Search each segment for additional vertex points that may have been
// created elsewhere that lie on the segment and split it there to
// avoid "T" intersections.

FGPolygon add_nodes_to_poly( const FGPolygon& poly, 
				    const FGTriNodes& tmp_nodes ) {
    int i, j;
    FGPolygon result; result.erase();
    Point3D p0, p1;

    // cout << "add_nodes_to_poly" << endl;

    for ( i = 0; i < poly.contours(); ++i ) {
	// cout << "contour = " << i << endl;
	for ( j = 0; j < poly.contour_size(i) - 1; ++j ) {
	    p0 = poly.get_pt( i, j );
	    p1 = poly.get_pt( i, j + 1 );

	    // add start of segment
	    result.add_node( i, p0 );

	    // add intermediate points
	    add_intermediate_nodes( i, p0, p1, tmp_nodes, &result );

	    // end of segment is beginning of next segment
	}
	p0 = poly.get_pt( i, poly.contour_size(i) - 1 );
	p1 = poly.get_pt( i, 0 );

	// add start of segment
	result.add_node( i, p0 );

	// add intermediate points
	add_intermediate_nodes( i, p0, p1, tmp_nodes, &result );

	// end of segment is beginning of next segment
	// 5/9/2000 CLO - this results in duplicating the last point
	// of a contour so I have removed this line.
	// result.add_node( i, p1 );

	// maintain original hole flag setting
	result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    return result;
}


// Traverse a polygon and split edges until they are less than max_len
// (specified in meters)
FGPolygon split_long_edges( const FGPolygon &poly, double max_len ) {
    FGPolygon result;
    Point3D p0, p1;
    int i, j, k;

    cout << "split_long_edges()" << endl;

    for ( i = 0; i < poly.contours(); ++i ) {
	// cout << "contour = " << i << endl;
	for ( j = 0; j < poly.contour_size(i) - 1; ++j ) {
	    p0 = poly.get_pt( i, j );
	    p1 = poly.get_pt( i, j + 1 );

	    double az1, az2, s;
	    geo_inverse_wgs_84( 0.0,
				p0.y(), p0.x(), p1.y(), p1.x(),
				&az1, &az2, &s );
	    cout << "distance = " << s << endl;

	    if ( s > max_len ) {
		int segments = (int)(s / max_len) + 1;
		cout << "segments = " << segments << endl;

		double dx = (p1.x() - p0.x()) / segments;
		double dy = (p1.y() - p0.y()) / segments;

		for ( k = 0; k < segments; ++k ) {
		    Point3D tmp( p0.x() + dx * k, p0.y() + dy * k, 0.0 );
		    cout << tmp << endl;
		    result.add_node( i, tmp );
		}
	    } else {
		cout << p0 << endl;
		result.add_node( i, p0 );
	    }
		
	    // end of segment is beginning of next segment
	}
	p0 = poly.get_pt( i, poly.contour_size(i) - 1 );
	p1 = poly.get_pt( i, 0 );

	double az1, az2, s;
	geo_inverse_wgs_84( 0.0,
			    p0.y(), p0.x(), p1.y(), p1.x(),
			    &az1, &az2, &s );
	cout << "distance = " << s << endl;

	if ( s > max_len ) {
	    int segments = (int)(s / max_len) + 1;
	    cout << "segments = " << segments << endl;
	    
	    double dx = (p1.x() - p0.x()) / segments;
	    double dy = (p1.y() - p0.y()) / segments;

	    for ( k = 0; k < segments; ++k ) {
		Point3D tmp( p0.x() + dx * k, p0.y() + dy * k, 0.0 );
		cout << tmp << endl;
		result.add_node( i, tmp );
	    }
	} else {
	    cout << p0 << endl;
	    result.add_node( i, p0 );
	}

	// maintain original hole flag setting
	result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    return result;
}


// Traverse a polygon and toss all the internal holes
FGPolygon strip_out_holes( const FGPolygon &poly ) {
    FGPolygon result; result.erase();

    cout << "strip_out_holes()" << endl;

    for ( int i = 0; i < poly.contours(); ++i ) {
	// cout << "contour = " << i << endl;
        point_list contour = poly.get_contour( i );
        if ( ! poly.get_hole_flag(i) ) {
            result.add_contour( contour, poly.get_hole_flag(i) );
        }
    }

    return result;
}


