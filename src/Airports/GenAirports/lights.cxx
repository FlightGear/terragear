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


#include "lights.hxx"


// generate runway lighting

void gen_runway_lights( const FGRunway& rwy_info, point_list *lights ) {
    int i;

    // using FGPolygon is a bit innefficient, but that's what the
    // routine returns.
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc1 = (corner[3] - corner[0]) / 20.0;
    Point3D inc2 = (corner[2] - corner[1]) / 20.0;

    Point3D pt1 = corner[0];
    Point3D pt2 = corner[1];
    lights->push_back( pt1 );
    lights->push_back( pt2 );

    for ( i = 0; i < 20; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
	lights->push_back( pt1 );
	lights->push_back( pt2 );
    }
}


