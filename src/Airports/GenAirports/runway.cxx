// area.c -- routines to assist with inserting "areas" into FG terrain
//
// Written by Curtis Olson, started March 1998.
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

#include <simgear/constants.h>
#include <simgear/math/fg_types.hxx>
#include <simgear/math/fg_geodesy.hxx>

#include "runway.hxx"
#include "point2d.hxx"


// calc new x, y for a rotation
double rot_x(double x, double y, double theta) {
    return ( x * cos(theta) + y * sin(theta) );
}


// calc new x, y for a rotation
double rot_y(double x, double y, double theta) {
    return ( -x * sin(theta) + y * cos(theta) );
}


FGPolygon batch_cart_to_polar_2d( const FGPolygon& in_list ) {
    FGPolygon out_list;
    Point3D p;

    out_list.erase();
    for ( int i = 0; i < (int)in_list.contour_size( 0 ); ++i ) {
	p = cart_to_polar_2d( in_list.get_pt( 0, i ) );
	out_list.add_node( 0, p );
    }

    return out_list;
}


// given a set of 2d coordinates relative to a center point, and the
// lon, lat of that center point (specified in degrees), as well as a
// potential orientation angle, generate the corresponding lon and lat
// of the original 2d verticies.
FGPolygon gen_area(Point3D origin, double angle, const FGPolygon& cart_list) {
    FGPolygon rad_list;
    FGPolygon result_list;
    Point3D p;

    // convert to polar coordinates
    rad_list = batch_cart_to_polar_2d(cart_list);

    // display points
    // cout << "converted to polar" << endl;
    // for ( int i = 0; i < rad_list.contour_size( 0 ); ++i ) {
    //     cout << "  " << rad_list.get_pt(0, i) << endl;
    // }

    // rotate by specified angle
    // cout << "Rotating points by " << angle << endl;
    for ( int i = 0; i < rad_list.contour_size( 0 ); ++i) {
	p = rad_list.get_pt( 0, i );
	double theta = p.y() + angle;
        while ( theta < FG_2PI ) {
            theta += FG_2PI;
	}
	p.sety( theta );
	rad_list.set_pt( 0, i, p );
	// cout << "  " << p << endl;
    }

    // find actual lon,lat of coordinates
    // cout << "convert to lon, lat relative to " << origin << endl;
    for ( int i = 0; i < (int)rad_list.contour_size( 0 ); ++i ) {
	// p = calc_lon_lat(origin_rad, rad_list.get_pt(0, i) );

	double lat2, lon2, az2;
	geo_direct_wgs_84 ( 0, origin.y(), origin.x(), 
			    rad_list.get_pt(0, i).y() * RAD_TO_DEG, 
			    rad_list.get_pt(0, i).x(), 
			    &lat2, &lon2, &az2 );

	// convert from radians to degress
	p.setx( lon2 );
	p.sety( lat2 );
	// cout << "  " << p << endl;
	result_list.add_node( 0, p );
    }

    return result_list;
}


// generate an area for a runway
FGPolygon gen_runway_area( const FGRunway& runway,
 			   double len_scale = 1.0,
			   double width_scale = 1.0 ) {

    FGPolygon result_list;
    FGPolygon tmp_list;

    double l, w;

    /*
    printf("runway: lon = %.2f lat = %.2f hdg = %.2f len = %.2f width = %.2f\n",
	   lon, lat, heading, length, width);
    */

    Point3D origin(runway.lon, runway.lat, 0);
    l = runway.length * len_scale * FEET_TO_METER / 2.0;
    w = runway.width * width_scale * FEET_TO_METER / 2.0;

    // generate untransformed runway area vertices
    tmp_list.add_node( 0, Point3D( l, w, 0 ) );
    tmp_list.add_node( 0, Point3D( l, -w, 0 ) );
    tmp_list.add_node( 0, Point3D( -l, -w, 0 ) );
    tmp_list.add_node( 0, Point3D( -l, w, 0 ) );

    // display points
    // cout << "Untransformed, unrotated runway" << endl;
    // for ( int i = 0; i < tmp_list.contour_size( 0 ); ++i ) {
    //    cout << "  " << tmp_list.get_pt(0, i) << endl;
    // }

    // rotate, transform, and convert points to lon, lat in degrees
    result_list = gen_area(origin, runway.heading * DEG_TO_RAD, tmp_list);

    // display points
    // cout << "Results in radians." << endl;
    // for ( int i = 0; i < result_list.contour_size( 0 ); ++i ) {
    //     cout << "  " << result_list.get_pt(0, i) << endl;
    // }

    return result_list;
}


// generate an area for a runway and include midpoints
FGPolygon gen_runway_w_mid( const FGRunway& runway, 
			    double len_scale = 1.0,
			    double width_scale = 1.0 ) {
    FGPolygon result_list;
    FGPolygon tmp_list;

    double l, w;

    /*
    printf("runway: lon = %.2f lat = %.2f hdg = %.2f len = %.2f width = %.2f\n",
	   lon, lat, heading, length, width);
    */

    Point3D origin(runway.lon, runway.lat, 0);
    l = runway.length * len_scale * FEET_TO_METER / 2.0;
    w = runway.width * width_scale * FEET_TO_METER / 2.0;

    // generate untransformed runway area vertices
    tmp_list.add_node( 0, Point3D( l, w, 0 ) );
    tmp_list.add_node( 0, Point3D( l, -w, 0 ) );
    tmp_list.add_node( 0, Point3D( 0, -w, 0 ) );
    tmp_list.add_node( 0, Point3D( -l, -w, 0 ) );
    tmp_list.add_node( 0, Point3D( -l, w, 0 ) );
    tmp_list.add_node( 0, Point3D( 0, w, 0 ) );

    // display points
    // cout << "Untransformed, unrotated runway" << endl;
    // for ( int i = 0; i < tmp_list.contour_size( 0 ); ++i ) {
    //    cout << "  " << tmp_list.get_pt(0, i) << endl;
    // }

    // rotate, transform, and convert points to lon, lat in degrees
    result_list = gen_area(origin, runway.heading * DEG_TO_RAD, tmp_list);

    // display points
    // cout << "Results in radians." << endl;
    // for ( int i = 0; i < result_list.contour_size( 0 ); ++i ) {
    //     cout << "  " << result_list.get_pt(0, i) << endl;
    // }

    return result_list;
}
