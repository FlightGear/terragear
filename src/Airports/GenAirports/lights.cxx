// lights.cxx -- Generate runway lighting
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


#include <simgear/math/sg_geodesy.hxx>

#include "lights.hxx"


// calculate the runway light direction vector.  I don't want to think
// about matrix transformations tonight, so instead I can take the
// center of one runway end - the center of the other end to get the
// direction of the runway.  Combine this with an appropriate portion
// of the local 'up' vector gives the light direction vector for the
// runway.
Point3D gen_runway_light_vector( const FGRunway& rwy_info ) {
    // Generate the 4 corners of the runway
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );
    point_list corner;
    for ( int i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }
    Point3D end1 = (corner[0] + corner[1]) / 2.0;
    Point3D end2 = (corner[2] + corner[3]) / 2.0;
    Point3D cart1 = sgGeodToCart( end1 );
    Point3D cart2 = sgGeodToCart( end2 );

    Point3D rwy_vec = cart2 - cart1;

    // FIXME
    // need to angle up (i.e. 3 degrees)

    double length = rwy_vec.distance3D( Point3D(0.0) );
    return rwy_vec / length;
}


// generate runway edge lighting
// 60 meters spacing or the next number down that divides evenly.
void gen_runway_edge_lights( const FGRunway& rwy_info, 
			     point_list *lights, point_list *normals ) {
    int i;

    double len = rwy_info.length * SG_FEET_TO_METER;
    int divs = (int)(len / 60.0) + 1;

    Point3D normal = gen_runway_light_vector( rwy_info );

    // using FGPolygon is a bit innefficient, but that's what the
    // routine returns.
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 2.0, 2.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc1 = (corner[3] - corner[0]) / divs;
    Point3D inc2 = (corner[2] - corner[1]) / divs;

    Point3D pt1 = corner[0];
    Point3D pt2 = corner[1];
    lights->push_back( pt1 );
    normals->push_back( normal );
    lights->push_back( pt2 );
    normals->push_back( normal );

    for ( i = 0; i < divs; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
	lights->push_back( pt1 );
	normals->push_back( normal );
	lights->push_back( pt2 );
	normals->push_back( normal );
    }
}


void gen_runway_lights( const FGRunway& rwy_info, 
			point_list *lights, point_list *normals ) {
    // Make edge lighting
    gen_runway_edge_lights( rwy_info, lights, normals );
}
