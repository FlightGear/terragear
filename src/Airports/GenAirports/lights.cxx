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


// calculate the runway light direction vector.  We take the center of
// one runway end - the center of the other end to get the direction
// of the runway.  Combine this with an appropriate portion of the
// local 'up' vector based on the provide 'angle' gives the light
// direction vector for the runway.
static Point3D gen_runway_light_vector( const FGRunway& rwy_info,
                                        double angle, bool recip ) {
    double length;

    // Generate the 4 corners of the runway
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );
    point_list corner;
    for ( int i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D end1, end2;
    if ( !recip ) {
        end1 = (corner[0] + corner[1]) / 2.0;
        end2 = (corner[2] + corner[3]) / 2.0;
    } else {
        end2 = (corner[0] + corner[1]) / 2.0;
        end1 = (corner[2] + corner[3]) / 2.0;
   }
    Point3D cart1 = sgGeodToCart( end1 * SG_DEGREES_TO_RADIANS );
    Point3D cart2 = sgGeodToCart( end2 * SG_DEGREES_TO_RADIANS );
    cout << "cart1 = " << cart1 << " cart2 = " << cart2 << endl;

    Point3D up = cart1;
    length = up.distance3D( Point3D(0.0) );
    up = up / length;

    Point3D rwy_vec = cart2 - cart1;
    cout << "rwy_vec = " << rwy_vec << endl;

    // angle up specified amount
    length = rwy_vec.distance3D( Point3D(0.0) );
    double up_length = length * tan(angle * SG_DEGREES_TO_RADIANS);
    Point3D light_vec = rwy_vec + (up * up_length);

    length = light_vec.distance3D( Point3D(0.0) );
    light_vec = light_vec / length;

    return light_vec;
}


// calculate the runway length vector.  We take the center of one
// runway end - the center of the other end to get the direction of
// the runway.
static Point3D gen_runway_length_vector( const FGRunway& rwy_info, bool recip )
{
    double length;

    // Generate the 4 corners of the runway
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );
    point_list corner;
    for ( int i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D end1, end2;
    if ( recip ) {
        end2 = (corner[0] + corner[1]) / 2.0;
        end1 = (corner[2] + corner[3]) / 2.0;
    } else {
        end1 = (corner[0] + corner[1]) / 2.0;
        end2 = (corner[2] + corner[3]) / 2.0;
    }
    Point3D cart1 = sgGeodToCart( end1 * SG_DEGREES_TO_RADIANS );
    Point3D cart2 = sgGeodToCart( end2 * SG_DEGREES_TO_RADIANS );
    cout << "cart1 = " << cart1 << " cart2 = " << cart2 << endl;

    Point3D rwy_vec = cart2 - cart1;
    cout << "rwy_vec = " << rwy_vec << endl;

    length = rwy_vec.distance3D( Point3D(0.0) );
    rwy_vec = rwy_vec / length;

    return rwy_vec;
}


// calculate a vector orthogonal to the runway direction in the
// surface plane.  As you approach the runway, positive will be left.
static Point3D gen_runway_left_vector( const FGRunway& rwy_info, bool recip )
{
    double length;

    // Generate the 4 corners of the runway
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );
    point_list corner;
    for ( int i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D end1, end2;
    if ( recip ) {
        end1 = corner[2];
        end2 = corner[3];
    } else {
        end1 = corner[0];
        end2 = corner[1];
    }
    Point3D cart1 = sgGeodToCart( end1 * SG_DEGREES_TO_RADIANS );
    Point3D cart2 = sgGeodToCart( end2 * SG_DEGREES_TO_RADIANS );
    cout << "cart1 = " << cart1 << " cart2 = " << cart2 << endl;

    Point3D left_vec = cart2 - cart1;
    cout << "left_vec = " << left_vec << endl;

    length = left_vec.distance3D( Point3D(0.0) );
    left_vec = left_vec / length;

    return left_vec;
}


// generate runway edge lighting
// 60 meters spacing or the next number down that divides evenly.
static FGSuperPoly gen_runway_edge_lights( const FGRunway& rwy_info,
                                           bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;

    double len = rwy_info.length * SG_FEET_TO_METER;
    int divs = (int)(len / 60.0) + 1;

    Point3D normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    // using FGPolygon is a bit innefficient, but that's what the
    // routine returns.
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 2.0, 2.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc1, inc2;
    Point3D pt1, pt2;

    if ( recip ) {
        inc1 = (corner[0] - corner[3]) / divs;
        inc2 = (corner[1] - corner[2]) / divs;
        pt1 = corner[3];
        pt2 = corner[2];
    } else {
        inc1 = (corner[3] - corner[0]) / divs;
        inc2 = (corner[2] - corner[1]) / divs;
        pt1 = corner[0];
        pt2 = corner[1];
    }

    lights.push_back( pt1 );
    normals.push_back( normal );
    lights.push_back( pt2 );
    normals.push_back( normal );

    for ( i = 0; i < divs; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
	lights.push_back( pt1 );
	normals.push_back( normal );
	lights.push_back( pt2 );
	normals.push_back( normal );
    }

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lights, false );
    normals_poly.add_contour( normals, false );

    FGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_WHITE_LIGHTS" );

    return result;
}


// generate a simple 2 bar VASI 
static FGSuperPoly gen_vasi( const FGRunway& rwy_info, float alt_m,
                             bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;

    cout << "gen vasi " << rwy_info.rwy_no << endl;

    Point3D normal;

    // using FGPolygon is a bit innefficient, but that's what the
    // routine returns.
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    // determine the start point.
    Point3D ref;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref = corner[0];
        length_hdg = 360.0 - rwy_info.heading;
    } else {
        ref = corner[2];
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    // offset 600' upwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        600 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    // offset 50' left
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                        50 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // downwind bar
    normal = gen_runway_light_vector( rwy_info, 2.5, recip );

    // unit1
    Point3D pt1 = ref;
    lights.push_back( pt1 );
    normals.push_back( normal );
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        1 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );
    
    // unit2
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        16 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        1 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );

    // unit3
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        16 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        1 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );

    // upwind bar
    normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    // unit1
    pt1 = ref;
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), length_hdg, 
                        700 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        1 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );
    
    // unit2
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        16 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        1 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );

    // unit3
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        16 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        1 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normals.push_back( normal );

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lights, false );
    normals_poly.add_contour( normals, false );

    FGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_VASI_LIGHTS" );

    return result;
}


void gen_runway_lights( const FGRunway& rwy_info, float alt_m,
			superpoly_list &lights ) {

    cout << "gen runway lights " << rwy_info.rwy_no << " " << rwy_info.end1_flags << " " << rwy_info.end2_flags << endl;;

    // Approach lighting
    if ( rwy_info.end1_flags.substr(2,1) == "V" ) {
        FGSuperPoly s = gen_vasi( rwy_info, alt_m, false );
        lights.push_back( s );
    }
    if ( rwy_info.end2_flags.substr(2,1) == "V" ) {
        FGSuperPoly s = gen_vasi( rwy_info, alt_m, true );
        lights.push_back( s );
    }

    // Make edge lighting
    {
        FGSuperPoly s = gen_runway_edge_lights( rwy_info, false );
        lights.push_back( s );
    }
    {
        FGSuperPoly s = gen_runway_edge_lights( rwy_info, true );
        lights.push_back( s );
    }
}
