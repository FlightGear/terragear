// convex_hull.cxx -- calculate the convex hull of a set of points
//
// Written by Curtis Olson, started September 1998.
//
// Copyright (C) 1998  Curtis L. Olson  - curt@me.umn.edu
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


#include <math.h>
#include <stdio.h>

#include <map>

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/misc/exception.hxx>

SG_USING_STD(less);
SG_USING_STD(map);

#include <simgear/constants.h>

#include "convex_hull.hxx"
#include "point2d.hxx"


// stl map typedefs
typedef map < double, double, less<double> > map_container;
typedef map_container::iterator map_iterator;


// Calculate theta of angle (a, b, c)
double calc_angle(Point3D a, Point3D b, Point3D c) {
    Point3D u, v;
    double udist, vdist, uv_dot, tmp;

    // u . v = ||u|| * ||v|| * cos(theta)

    u.setx( b.x() - a.x() );
    u.sety( b.y() - a.y() );
    udist = sqrt( u.x() * u.x() + u.y() * u.y() );
    // printf("udist = %.6f\n", udist);

    v.setx( b.x() - c.x() );
    v.sety( b.y() - c.y() );
    vdist = sqrt( v.x() * v.x() + v.y() * v.y() );
    // printf("vdist = %.6f\n", vdist);

    uv_dot = u.x() * v.x() + u.y() * v.y();
    // printf("uv_dot = %.6f\n", uv_dot);

    tmp = uv_dot / (udist * vdist);
    // printf("tmp = %.6f\n", tmp);

    return acos(tmp);
}


// Test to see if angle(Pa, Pb, Pc) < 180 degrees
bool test_point(Point3D Pa, Point3D Pb, Point3D Pc) {
    double a1, a2;

    Point3D origin( 0.0 );

    Point3D a( cos(Pa.y()) * Pa.x(),
	       sin(Pa.y()) * Pa.x(), 0 );

    Point3D b( cos(Pb.y()) * Pb.x(),
	       sin(Pb.y()) * Pb.x(), 0 );

    Point3D c( cos(Pc.y()) * Pc.x(),
	       sin(Pc.y()) * Pc.x(), 0 );

    // printf("a is %.6f %.6f\n", a.x, a.y);
    // printf("b is %.6f %.6f\n", b.x, b.y);
    // printf("c is %.6f %.6f\n", c.x, c.y);

    a1 = calc_angle(a, b, origin);
    a2 = calc_angle(origin, b, c);

    // printf("a1 = %.2f  a2 = %.2f\n", a1 * SGD_RADIANS_TO_DEGREES, a2 * SGD_RADIANS_TO_DEGREES);

    return ( (a1 + a2) < SGD_PI );
}


// calculate the convex hull of a set of points, return as a list of
// point2d.  The algorithm description can be found at:
// http://riot.ieor.berkeley.edu/riot/Applications/ConvexHull/CHDetails.html
TGPolygon convex_hull( const point_list& input_list ) {
    int i;

    map_iterator map_current, map_next, map_next_next, map_last;

    // list of translated points
    point_list trans_list;

    // points sorted by radian degrees
    map_container radians_map;

    // will contain the convex hull
    TGPolygon con_hull;

    Point3D p, Pa, Pb, Pc, result;
    double sum_x, sum_y;
    int in_count, last_size;

    // STEP ONE:  Find an average midpoint of the input set of points
    in_count = input_list.size();
    sum_x = sum_y = 0.0;

    for ( i = 0; i < in_count; ++i ) {
	sum_x += input_list[i].x();
	sum_y += input_list[i].y();
    }

    Point3D average( sum_x / in_count, sum_y / in_count, 0 );

    // printf("Average center point is %.4f %.4f\n", average.x, average.y);

    // STEP TWO:  Translate input points so average is at origin
    trans_list.clear();

    for ( i = 0; i < in_count; ++i ) {
	p = Point3D( input_list[i].x() - average.x(),
		     input_list[i].y() - average.y(), 0 );
	// printf("%.6f %.6f\n", p.x, p.y);
	trans_list.push_back( p );
    }

    // STEP THREE:  convert to radians and sort by theta
    radians_map.clear();

    for ( i = 0; i < in_count; ++i ) {
	p = cart_to_polar_2d( trans_list[i] );
	if ( p.x() > radians_map[p.y()] ) {
	    radians_map[p.y()] = p.x();
	}
    }

    /*
    // printf("Sorted list\n");
    map_current = radians_map.begin();
    map_last = radians_map.end();
    for ( ; map_current != map_last ; ++map_current ) {
	p.setx( (*map_current).first );
	p.sety( (*map_current).second );

	printf("p is %.6f %.6f\n", p.x(), p.y());
    }
    */

    // STEP FOUR: traverse the sorted list and eliminate everything
    // not on the perimeter.
    // printf("Traversing list\n");

    // double check list size ... this should never fail because a
    // single runway will always generate four points.
    if ( radians_map.size() < 3 ) {
        throw sg_exception("convex hull not possible with < 3 points");
    }

    // ensure that we run the while loop at least once
    last_size = radians_map.size() + 1;

    while ( last_size > (int)radians_map.size() ) {
	// printf("Running an iteration of the graham scan algorithm\n"); 
	last_size = radians_map.size();

	map_current = radians_map.begin();
	while ( map_current != radians_map.end() ) {
	    // get first element
	    Pa.sety( (*map_current).first );
	    Pa.setx( (*map_current).second );

	    // get second element
	    map_next = map_current;
	    ++map_next;
	    if ( map_next == radians_map.end() ) {
		map_next = radians_map.begin();
	    }
	    Pb.sety( (*map_next).first );
	    Pb.setx( (*map_next).second );

	    // get third element
	    map_next_next = map_next;
	    ++map_next_next;
	    if ( map_next_next == radians_map.end() ) {
		map_next_next = radians_map.begin();
	    }
	    Pc.sety( (*map_next_next).first );
	    Pc.setx( (*map_next_next).second );

	    // printf("Pa is %.6f %.6f\n", Pa.y(), Pa.x());
	    // printf("Pb is %.6f %.6f\n", Pb.y(), Pb.x());
	    // printf("Pc is %.6f %.6f\n", Pc.y(), Pc.x());

	    if ( test_point(Pa, Pb, Pc) ) {
		// printf("Accepted a point\n");
		// accept point, advance Pa, Pb, and Pc.
		++map_current;
	    } else {
		// printf("REJECTED A POINT\n");
		// reject point, delete it and advance only Pb and Pc
		map_next = map_current;
		++map_next;
		if ( map_next == radians_map.end() ) {
		    map_next = radians_map.begin();
		}
		radians_map.erase( map_next );
	    }
	}
    }

    // translate back to correct lon/lat
    // printf("Final sorted convex hull\n");
    con_hull.erase();
    map_current = radians_map.begin();
    map_last = radians_map.end();
    for ( ; map_current != map_last ; ++map_current ) {
	p.sety( (*map_current).first );
	p.setx( (*map_current).second );

	result.setx( cos(p.y()) * p.x() + average.x() );
	result.sety( sin(p.y()) * p.x() + average.y() );

	// printf("%.6f %.6f\n", result.x, result.y);

	con_hull.add_node(0, result);
    }

    return con_hull;
}
