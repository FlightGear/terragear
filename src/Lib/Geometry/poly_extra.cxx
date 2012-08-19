// poly_extra.cxx -- Extra polygon manipulation routines
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
// $Id: poly_extra.cxx,v 1.9 2004-11-19 22:25:49 curt Exp $
//

#include <stdio.h>

#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>

#include <Geometry/poly_support.hxx>

#include "poly_extra.hxx"


// Divide segment if there are other existing points on it, return the
// new polygon
void add_intermediate_nodes( int contour, const Point3D& start, 
                             const Point3D& end, point_list& tmp_nodes,
                             TGPolygon *result,
                             const double bbEpsilon,
                             const double errEpsilon
                           )
{
    Point3D new_pt;

    SG_LOG(SG_GENERAL, SG_BULK, "   " << start << " <==> " << end );

    bool found_extra = find_intermediate_node( start, end, tmp_nodes, &new_pt, bbEpsilon, errEpsilon );

    if ( found_extra ) {
        // recurse with two sub segments
        // SG_LOG(SG_GENERAL, SG_DEBUG, "dividing " << p0 << " " << nodes[extra_index]
        //      << " " << p1);
        add_intermediate_nodes( contour, start, new_pt, tmp_nodes, result, bbEpsilon, errEpsilon  );

        result->add_node( contour, new_pt );
        SG_LOG(SG_GENERAL, SG_BULK, "    adding = " << new_pt);

        add_intermediate_nodes( contour, new_pt, end, tmp_nodes, result, bbEpsilon, errEpsilon  );
    }
}


// Search each segment for additional vertex points that may have been
// created elsewhere that lie on the segment and split it there to
// avoid "T" intersections.

TGPolygon add_nodes_to_poly( const TGPolygon& poly, 
                             const TGTriNodes& nodes ) {
    int i, j;
    TGPolygon result; result.erase();
    point_list tmp_nodes = nodes.get_node_list();
    Point3D p0, p1;

    // SG_LOG(SG_GENERAL, SG_DEBUG, "add_nodes_to_poly");

    for ( i = 0; i < poly.contours(); ++i ) {
	// SG_LOG(SG_GENERAL, SG_DEBUG, "contour = " << i);
	for ( j = 0; j < poly.contour_size(i) - 1; ++j ) {
	    p0 = poly.get_pt( i, j );
	    p1 = poly.get_pt( i, j + 1 );

	    // add start of segment
	    result.add_node( i, p0 );

	    // add intermediate points
        add_intermediate_nodes( i, p0, p1, tmp_nodes, &result, SG_EPSILON*10, SG_EPSILON*4 );

	    // end of segment is beginning of next segment
	}
	p0 = poly.get_pt( i, poly.contour_size(i) - 1 );
	p1 = poly.get_pt( i, 0 );

	// add start of segment
	result.add_node( i, p0 );

	// add intermediate points
    add_intermediate_nodes( i, p0, p1, tmp_nodes, &result, SG_EPSILON*10, SG_EPSILON*4 );

	// end of segment is beginning of next segment
	// 5/9/2000 CLO - this results in duplicating the last point
	// of a contour so I have removed this line.
	// result.add_node( i, p1 );

	// maintain original hole flag setting
	result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    return result;
}

TGPolygon add_tgnodes_to_poly( const TGPolygon& poly, 
                               const TGNodes* nodes ) {
    TGPolygon result; result.erase();
    Point3D min, max;
    Point3D p0, p1;
    point_list poly_points;

    poly.get_bounding_box(min, max);
    SG_LOG(SG_GENERAL, SG_DEBUG, "add_tgnodes_to_poly : min " << min << " max " << max );

    poly_points = nodes->get_geod_inside( min, max );

    for ( int i = 0; i < poly.contours(); ++i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "contour = " << i);
        for ( int j = 0; j < poly.contour_size(i) - 1; ++j ) {
            p0 = poly.get_pt( i, j );
            p1 = poly.get_pt( i, j + 1 );

            // add start of segment
            result.add_node( i, p0 );

            // add intermediate points
            add_intermediate_nodes( i, p0, p1, poly_points, &result, SG_EPSILON*10, SG_EPSILON*4 );

            // end of segment is beginning of next segment
        }
        p0 = poly.get_pt( i, poly.contour_size(i) - 1 );
        p1 = poly.get_pt( i, 0 );

        // add start of segment
        result.add_node( i, p0 );

        // add intermediate points
        add_intermediate_nodes( i, p0, p1, poly_points, &result, SG_EPSILON*10, SG_EPSILON*4 );

        // maintain original hole flag setting
        result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    return result;
}