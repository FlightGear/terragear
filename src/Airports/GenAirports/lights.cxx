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

SG_USING_STD(cout);
SG_USING_STD(endl);


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
static superpoly_list gen_runway_edge_lights( const FGRunway& rwy_info,
                                              const string& kind, bool recip )
{
    point_list w_lights; w_lights.clear();
    point_list y_lights; y_lights.clear();
    point_list w_normals; w_normals.clear();
    point_list y_normals; y_normals.clear();
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
        inc1 = (corner[3] - corner[0]) / divs;
        inc2 = (corner[2] - corner[1]) / divs;
        pt1 = corner[0];
        pt2 = corner[1];
    } else {
        inc1 = (corner[0] - corner[3]) / divs;
        inc2 = (corner[1] - corner[2]) / divs;
        pt1 = corner[3];
        pt2 = corner[2];
    }

    double dist = rwy_info.length;
    double step = dist / divs;

    w_lights.push_back( pt1 );
    w_normals.push_back( normal );
    w_lights.push_back( pt2 );
    w_normals.push_back( normal );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
        if ( dist > 2000.0 ) {
            w_lights.push_back( pt1 );
            w_normals.push_back( normal );
            w_lights.push_back( pt2 );
            w_normals.push_back( normal );
        } else {
            y_lights.push_back( pt1 );
            y_normals.push_back( normal );
            y_lights.push_back( pt2 );
            y_normals.push_back( normal );
        }
        dist -= step;
    }

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    FGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    if ( kind == "H" ) {
        white.set_material( "RWY_WHITE_LIGHTS" );
    } else if ( kind == "M" ) {
        white.set_material( "RWY_WHITE_MEDIUM_LIGHTS" );
    } else if ( kind == "L" ) {
        white.set_material( "RWY_WHITE_LOW_LIGHTS" );
    }

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( y_lights, false );
    normals_poly.add_contour( y_normals, false );

    FGSuperPoly yellow;
    yellow.set_poly( lights_poly );
    yellow.set_normals( normals_poly );
    if ( kind == "H" ) {
        yellow.set_material( "RWY_YELLOW_LIGHTS" );
    } else if ( kind == "M" ) {
        yellow.set_material( "RWY_YELLOW_MEDIUM_LIGHTS" );
    } else if ( kind == "L" ) {
        yellow.set_material( "RWY_YELLOW_LOW_LIGHTS" );
    }

    superpoly_list result; result.clear();

    result.push_back( white );
    result.push_back( yellow );

    return result;
}


// generate threshold lights for a 3 degree approach 
static superpoly_list gen_runway_threshold_lights( const FGRunway& rwy_info,
                                                   const string& kind,
                                                   float alt_m, bool recip )
{
    point_list g_lights; g_lights.clear();
    point_list g_normals; g_normals.clear();
    point_list r_lights; r_lights.clear();
    point_list r_normals; r_normals.clear();
    int i;

    cout << "gen threshold " << rwy_info.rwy_no << endl;

    Point3D normal;

    // using FGPolygon is a bit innefficient, but that's what the
    // routine returns.
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    // determine the start point.
    Point3D ref1, ref2, ref3, ref4;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref1 = corner[0];
        ref2 = corner[1];
        ref3 = corner[3];
        ref4 = corner[2];
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref1 = corner[2];
        ref2 = corner[3];
        ref3 = corner[1];
        ref4 = corner[0];
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    // offset 5' downwind
    geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), length_hdg, 
                        -5 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref1 = Point3D( lon, lat, 0.0 );
    geo_direct_wgs_84 ( alt_m, ref2.lat(), ref2.lon(), length_hdg, 
                        -5 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref2 = Point3D( lon, lat, 0.0 );

    // offset 5' upwind
    geo_direct_wgs_84 ( alt_m, ref3.lat(), ref3.lon(), length_hdg, 
                        -5 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref3 = Point3D( lon, lat, 0.0 );
    geo_direct_wgs_84 ( alt_m, ref4.lat(), ref4.lon(), length_hdg, 
                        -5 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref4 = Point3D( lon, lat, 0.0 );

    // five lights each
    for ( int i = 0; i < 5; ++i ) {
        g_lights.push_back( ref1 );
        g_normals.push_back( normal );
    
        g_lights.push_back( ref2 );
        g_normals.push_back( normal );

        r_lights.push_back( ref3 );
        r_normals.push_back( normal );
    
        r_lights.push_back( ref4 );
        r_normals.push_back( normal );

        // offset 10' towards center
        geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), left_hdg, 
                            -10 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref1 = Point3D( lon, lat, 0.0 );
        geo_direct_wgs_84 ( alt_m, ref2.lat(), ref2.lon(), left_hdg, 
                            10 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref2 = Point3D( lon, lat, 0.0 );
        geo_direct_wgs_84 ( alt_m, ref3.lat(), ref3.lon(), left_hdg, 
                            -10 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref3 = Point3D( lon, lat, 0.0 );
        geo_direct_wgs_84 ( alt_m, ref4.lat(), ref4.lon(), left_hdg, 
                            10 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref4 = Point3D( lon, lat, 0.0 );
       
    }

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    FGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );
    if ( kind == "H" ) {
        green.set_material( "RWY_GREEN_LIGHTS" );
    } else if ( kind == "M" ) {
        green.set_material( "RWY_GREEN_MEDIUM_LIGHTS" );
    } else if ( kind == "L" ) {
        green.set_material( "RWY_GREEN_LOW_LIGHTS" );
    }

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    FGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    if ( kind == "H" ) {
        red.set_material( "RWY_RED_LIGHTS" );
    } else if ( kind == "M" ) {
        red.set_material( "RWY_RED_MEDIUM_LIGHTS" );
    } else if ( kind == "L" ) {
        red.set_material( "RWY_RED_LOW_LIGHTS" );
    }

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );

    return result;
}


// generate runway center line lighting, 50' spacing.
static superpoly_list gen_runway_center_line_lights( const FGRunway& rwy_info,
                                                     bool recip )
{
    point_list w_lights; w_lights.clear();
    point_list r_lights; r_lights.clear();
    point_list w_normals; w_normals.clear();
    point_list r_normals; r_normals.clear();
    int i;

    double len = rwy_info.length;
    // this should be 25' technically but I'm trying 50' to space things out
    int divs = (int)(len / (50.0*SG_FEET_TO_METER)) + 1;

    Point3D normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    // using FGPolygon is a bit innefficient, but that's what the
    // routine returns.
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 2.0, 2.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc;
    Point3D pt1, pt2;

    if ( recip ) {
        pt1 = (corner[0] + corner[1] ) / 2.0;
        pt2 = (corner[2] + corner[3] ) / 2.0;
    } else {
        pt1 = (corner[2] + corner[3] ) / 2.0;
        pt2 = (corner[0] + corner[1] ) / 2.0;
    }
    inc = (pt2 - pt1) / divs;

    double dist = len;
    double step = len / divs;
    pt1 += inc;                 // move 25' in
    dist -= step;
    bool use_white = true;

    while ( dist > 0.0 ) {
        if ( dist > 3000.0 ) {
            w_lights.push_back( pt1 );
            w_normals.push_back( normal );
        } else if ( dist > 1000.0 ) {
            if ( use_white ) {
                w_lights.push_back( pt1 );
                w_normals.push_back( normal );
            } else {
                r_lights.push_back( pt1 );
                r_normals.push_back( normal );
            }
            use_white = !use_white;
        } else {
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );
        }
  	pt1 += inc;
  	pt1 += inc;
        dist -= step;
        dist -= step;
    }

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    FGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_MEDIUM_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    FGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_MEDIUM_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( white );
    result.push_back( red );

    return result;
}


// generate touch down zone lights
static FGSuperPoly gen_touchdown_zone_lights( const FGRunway& rwy_info,
                                              float alt_m, bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;

    cout << "gen touchdown zone lights " << rwy_info.rwy_no << endl;

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
        ref = (corner[0] + corner[1]) / 2;
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref = (corner[2] + corner[3]) / 2;
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    for ( i = 0; i < 30; ++i ) {
        // offset 100' upwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            100 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        Point3D pt1 = ref;

        // left side bar
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            36 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        lights.push_back( pt1 );
        normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        lights.push_back( pt1 );
        normals.push_back( normal );

        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        lights.push_back( pt1 );
        normals.push_back( normal );

        pt1 = ref;

        // right side bar
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -36 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        lights.push_back( pt1 );
        normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        lights.push_back( pt1 );
        normals.push_back( normal );

        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        lights.push_back( pt1 );
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


// generate a simple 2 bar VASI for a 3 degree approach
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
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
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


// generate a simple PAPI for a 3 degree approach 
static FGSuperPoly gen_papi( const FGRunway& rwy_info, float alt_m,
                             bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;

    cout << "gen papi " << rwy_info.rwy_no << endl;

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
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref = corner[2];
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    // offset 950' upwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        950 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    // offset 50' left
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                        50 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // unit1
    Point3D pt1 = ref;
    lights.push_back( pt1 );
    normal = gen_runway_light_vector( rwy_info, 3.5, recip );
    normals.push_back( normal );
    
    // unit2
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        30 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normal = gen_runway_light_vector( rwy_info, 3.167, recip );
    normals.push_back( normal );

    // unit3
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        30 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normal = gen_runway_light_vector( rwy_info, 2.833, recip );
    normals.push_back( normal );

    // unit4
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        30 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lights.push_back( pt1 );
    normal = gen_runway_light_vector( rwy_info, 2.5, recip );
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


// generate REIL lights
static FGSuperPoly gen_reil( const FGRunway& rwy_info, float alt_m,
                             bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;

    cout << "gen reil " << rwy_info.rwy_no << endl;

    Point3D normal;

    // using FGPolygon is a bit innefficient, but that's what the
    // routine returns.
    FGPolygon poly_corners = gen_runway_area_w_expand( rwy_info, 0.0, 0.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    // determine the start point.
    Point3D ref1, ref2;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref1 = corner[0];
        ref2 = corner[1];
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref1 = corner[2];
        ref2 = corner[3];
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    // offset 40' downwind
    geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), length_hdg, 
                        -40 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref1 = Point3D( lon, lat, 0.0 );
    // offset 40' left
    geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), left_hdg, 
                        40 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref1 = Point3D( lon, lat, 0.0 );

    lights.push_back( ref1 );
    normal = gen_runway_light_vector( rwy_info, 10, recip );
    normals.push_back( normal );
    
    // offset 40' downwind
    geo_direct_wgs_84 ( alt_m, ref2.lat(), ref2.lon(), length_hdg, 
                        -40 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref2 = Point3D( lon, lat, 0.0 );
    // offset 40' left
    geo_direct_wgs_84 ( alt_m, ref2.lat(), ref2.lon(), left_hdg, 
                        -40 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref2 = Point3D( lon, lat, 0.0 );

    lights.push_back( ref2 );
    normal = gen_runway_light_vector( rwy_info, 10, recip );
    normals.push_back( normal );

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lights, false );
    normals_poly.add_contour( normals, false );

    FGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_REIL_LIGHTS" );

    return result;
}


// generate ALSF-I/II and SALS approach lighting schemes
static superpoly_list gen_alsf( const FGRunway& rwy_info,
                                float alt_m, const string &kind, bool recip )
{
    point_list g_lights; g_lights.clear();
    point_list w_lights; w_lights.clear();
    point_list r_lights; r_lights.clear();
    point_list s_lights; s_lights.clear();
    point_list g_normals; g_normals.clear();
    point_list w_normals; w_normals.clear();
    point_list r_normals; r_normals.clear();
    point_list s_normals; s_normals.clear();
    int i, j;

    cout << "gen ALSF/SALS lights " << rwy_info.rwy_no << endl;

    Point3D normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    // Generate the threshold lights

    double len = rwy_info.length * SG_FEET_TO_METER;
    int divs = (int)(len / 10.0) + 1;

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
        inc1 = (corner[0] - corner[1]) / divs;
        inc2 = (corner[3] - corner[2]) / divs;
        pt1 = corner[1];
        pt2 = corner[2];
    } else {
        inc1 = (corner[2] - corner[3]) / divs;
        inc2 = (corner[1] - corner[0]) / divs;
        pt1 = corner[3];
        pt2 = corner[0];
    }

    double dist = rwy_info.length;
    double step = dist / divs;

    g_lights.push_back( pt1 );
    g_normals.push_back( normal );
    r_lights.push_back( pt2 );
    r_normals.push_back( normal );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
        g_lights.push_back( pt1 );
        g_normals.push_back( normal );
        r_lights.push_back( pt2 );
        r_normals.push_back( normal );
        dist -= step;
    }

    // Generate long center bar of lights

    // determine the start point.
    Point3D ref_save;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref_save = (corner[0] + corner[1]) / 2;
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref_save = (corner[2] + corner[3]) / 2;
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    Point3D ref = ref_save;

    int count;
    if ( kind == "1" || kind == "2" ) {
        // ALSF-I or ALSF-II
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -100 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
        count = 30;
    } else {
        // SALS
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -300 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
        count = 13;
    }

    for ( i = 0; i < count; ++i ) {
        pt1 = ref;
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        pt1 = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -100 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
    }

    ref = ref_save;

    if ( kind == "1" || kind == "O" ) {
        // Terminating bar

        // offset 200' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -200 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        pt1 = ref;
 
        // left 3 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            15 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );

        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );

        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );

        pt1 = ref;

        // right 3 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -15 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );
    } else if ( kind == "2" ) {
        // Generate red side row lights

        for ( i = 0; i < 9; ++i ) {
            // offset 100' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -100 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );

            pt1 = ref;
 
            // left 3 side lights
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                36 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );

            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );

            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );

            pt1 = ref;

            // right 3 side lights
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                -36 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );
    
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                -5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );
    
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                -5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );
        }
    }

    if ( kind == "1" || kind == "O" ) {
        // Generate pre-threshold bar

        ref = ref_save;

        // offset 100' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -100 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
    
        pt1 = ref;

        // left 5 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            75 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );

        for ( j = 0; j < 4; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );
        }

        pt1 = ref;

        // rioght 5 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -75 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );

        for ( j = 0; j < 4; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt1 );
            r_normals.push_back( normal );
        }
    } else if ( kind == "2" ) {
        // Generate -500 extra horizontal row of lights

        ref = ref_save;

        // offset 500' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -500 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
    
        pt1 = ref;

        // left 4 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            11.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        for ( j = 0; j < 3; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            w_lights.push_back( pt1 );
            w_normals.push_back( normal );
        }

        pt1 = ref;

        // right 4 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -11.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        for ( j = 0; j < 3; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                                -5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt1 = Point3D( lon, lat, 0.0 );
            w_lights.push_back( pt1 );
            w_normals.push_back( normal );
        }
    }

    ref = ref_save;

    if ( kind == "O" ) {
        // generate SALS secondary threshold

        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -200 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
        count = 30;

        pt1 = ref;
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );

        pt1 = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt1 );
        r_normals.push_back( normal );
    }

    // Generate -1000' extra horizontal row of lights

    ref = ref_save;

    // offset 1000' downwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    
    pt1 = ref;
 
    // left 8 side lights
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt1 );
    w_normals.push_back( normal );

    for ( j = 0; j < 7; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    pt1 = ref;
 
    // right 8 side lights
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        -15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt1 );
    w_normals.push_back( normal );

    for ( j = 0; j < 7; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    ref = ref_save;

    if ( kind == "1" || kind == "2" ) {
        // generate rabbit lights

        // start 1000' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        for ( i = 0; i < 21; ++i ) {
            s_lights.push_back( ref );
            s_normals.push_back( normal );

            // offset 100' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -100 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    }

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    FGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    FGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    FGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );
    result.push_back( white );

    if ( s_lights.size() ) {
        lights_poly.erase();
        normals_poly.erase();
        lights_poly.add_contour( s_lights, false );
        normals_poly.add_contour( s_normals, false );

        FGSuperPoly sequenced;
        sequenced.set_poly( lights_poly );
        sequenced.set_normals( normals_poly );
        sequenced.set_material( "RWY_SEQUENCED_LIGHTS" );

        result.push_back( sequenced );
    }
 
    return result;
}


// generate SSALS, SSALF, and SSALR approach lighting scheme (kind =
// S, F, or R)
static superpoly_list gen_ssalx( const FGRunway& rwy_info,
                                 float alt_m, const string& kind, bool recip )
{
    point_list g_lights; g_lights.clear();
    point_list w_lights; w_lights.clear();
    point_list r_lights; r_lights.clear();
    point_list s_lights; s_lights.clear();
    point_list g_normals; g_normals.clear();
    point_list w_normals; w_normals.clear();
    point_list r_normals; r_normals.clear();
    point_list s_normals; s_normals.clear();
    int i, j;

    cout << "gen SSALx lights " << rwy_info.rwy_no << endl;

    Point3D normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    // Generate the threshold lights

    double len = rwy_info.length * SG_FEET_TO_METER;
    int divs = (int)(len / 10.0) + 1;

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
        inc1 = (corner[0] - corner[1]) / divs;
        inc2 = (corner[3] - corner[2]) / divs;
        pt1 = corner[1];
        pt2 = corner[2];
    } else {
        inc1 = (corner[2] - corner[3]) / divs;
        inc2 = (corner[1] - corner[0]) / divs;
        pt1 = corner[3];
        pt2 = corner[0];
    }

    double dist = rwy_info.length;
    double step = dist / divs;

    g_lights.push_back( pt1 );
    g_normals.push_back( normal );
    r_lights.push_back( pt2 );
    r_normals.push_back( normal );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
        g_lights.push_back( pt1 );
        g_normals.push_back( normal );
        r_lights.push_back( pt2 );
        r_normals.push_back( normal );
        dist -= step;
    }

    // Generate long center bar of lights (every 200')

    // determine the start point.
    Point3D ref_save;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref_save = (corner[0] + corner[1]) / 2;
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref_save = (corner[2] + corner[3]) / 2;
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    Point3D ref = ref_save;

    for ( i = 0; i < 7; ++i ) {
        // offset 200' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -200 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        pt1 = ref;
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        pt1 = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    // Generate -1000' extra horizontal row of lights

    ref = ref_save;

    // offset 1000' downwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    
    pt1 = ref;
 
    // left 5 side lights
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt1 );
    w_normals.push_back( normal );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    pt1 = ref;
 
    // right 5 side lights
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        -15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt1 );
    w_normals.push_back( normal );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    if ( kind == "R" ) {
        // generate 8 rabbit lights

        ref = ref_save;

        // start 1600' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -1600 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        for ( i = 0; i < 8; ++i ) {
            s_lights.push_back( ref );
            s_normals.push_back( normal );

            // offset 200' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -200 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    } else if ( kind == "F" ) {
        // generate 3 sequenced lights aligned with last 3 light bars
        ref = ref_save;

        // start 1000' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        for ( i = 0; i < 3; ++i ) {
            s_lights.push_back( ref );
            s_normals.push_back( normal );
             
            // offset 200' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -200 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    }

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    FGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    FGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    FGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );
    result.push_back( white );

    if ( s_lights.size() > 0 ) {
        lights_poly.erase();
        normals_poly.erase();
        lights_poly.add_contour( s_lights, false );
        normals_poly.add_contour( s_normals, false );

        FGSuperPoly sequenced;
        sequenced.set_poly( lights_poly );
        sequenced.set_normals( normals_poly );
        sequenced.set_material( "RWY_SEQUENCED_LIGHTS" );

        result.push_back( sequenced );
    }
 
    return result;
}


// generate MALS, MALSF, and MALSR approach lighting scheme (kind =
// ' ', F, or R)
static superpoly_list gen_malsx( const FGRunway& rwy_info,
                                 float alt_m, const string& kind, bool recip )
{
    point_list g_lights; g_lights.clear();
    point_list w_lights; w_lights.clear();
    point_list r_lights; r_lights.clear();
    point_list s_lights; s_lights.clear();
    point_list g_normals; g_normals.clear();
    point_list w_normals; w_normals.clear();
    point_list r_normals; r_normals.clear();
    point_list s_normals; s_normals.clear();
    int i, j;

    cout << "gen SSALx lights " << rwy_info.rwy_no << endl;

    Point3D normal = gen_runway_light_vector( rwy_info, 3.0, recip );

    // Generate the threshold lights

    double len = rwy_info.length * SG_FEET_TO_METER;
    int divs = (int)(len / 10.0) + 1;

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
        inc1 = (corner[0] - corner[1]) / divs;
        inc2 = (corner[3] - corner[2]) / divs;
        pt1 = corner[1];
        pt2 = corner[2];
    } else {
        inc1 = (corner[2] - corner[3]) / divs;
        inc2 = (corner[1] - corner[0]) / divs;
        pt1 = corner[3];
        pt2 = corner[0];
    }

    double dist = rwy_info.length;
    double step = dist / divs;

    g_lights.push_back( pt1 );
    g_normals.push_back( normal );
    r_lights.push_back( pt2 );
    r_normals.push_back( normal );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
        g_lights.push_back( pt1 );
        g_normals.push_back( normal );
        r_lights.push_back( pt2 );
        r_normals.push_back( normal );
        dist -= step;
    }

    // Generate long center bar of lights (every 200')

    // determine the start point.
    Point3D ref_save;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref_save = (corner[0] + corner[1]) / 2;
        length_hdg = rwy_info.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref_save = (corner[2] + corner[3]) / 2;
        length_hdg = rwy_info.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    Point3D ref = ref_save;

    for ( i = 0; i < 7; ++i ) {
        // offset 200' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -200 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        pt1 = ref;
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );

        pt1 = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    // Generate -1000' extra horizontal row of lights

    ref = ref_save;

    // offset 1000' downwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    
    pt1 = ref;
 
    // left 5 side lights
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        23 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt1 );
    w_normals.push_back( normal );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    pt1 = ref;
 
    // right 5 side lights
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        -23 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt1 );
    w_normals.push_back( normal );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                            -2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt1 = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt1 );
        w_normals.push_back( normal );
    }

    if ( kind == "R" ) {
        // generate 5 rabbit lights

        ref = ref_save;

        // start 1600' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -1600 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        for ( i = 0; i < 8; ++i ) {
            s_lights.push_back( ref );
            s_normals.push_back( normal );

            // offset 200' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -200 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    } else if ( kind == "F" ) {
        // generate 3 sequenced lights aligned with last 3 light bars
        ref = ref_save;

        // start 1000' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        for ( i = 0; i < 3; ++i ) {
            s_lights.push_back( ref );
            s_normals.push_back( normal );
             
            // offset 200' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -200 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    }

    FGPolygon lights_poly; lights_poly.erase();
    FGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    FGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    FGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    FGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );
    result.push_back( white );

    if ( s_lights.size() > 0 ) {
        lights_poly.erase();
        normals_poly.erase();
        lights_poly.add_contour( s_lights, false );
        normals_poly.add_contour( s_normals, false );

        FGSuperPoly sequenced;
        sequenced.set_poly( lights_poly );
        sequenced.set_normals( normals_poly );
        sequenced.set_material( "RWY_SEQUENCED_LIGHTS" );

        result.push_back( sequenced );
    }
 
    return result;
}


// top level runway light generator
void gen_runway_lights( const FGRunway& rwy_info, float alt_m,
			superpoly_list &lights ) {

    cout << "gen runway lights " << rwy_info.rwy_no << " "
         << rwy_info.end1_flags << " " << rwy_info.end2_flags << endl;;

    unsigned int i;

    // Make edge lighting
    string edge_type = rwy_info.surface_flags.substr(3,1);
    if ( edge_type != (string)"N" ) {
        // forward direction
        superpoly_list s;
        s = gen_runway_edge_lights( rwy_info, edge_type, false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }

        // reverse direction
        s = gen_runway_edge_lights( rwy_info, edge_type, true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // Centerline lighting
    if ( rwy_info.surface_flags.substr(0,1) == "Y" ) {
        // forward direction
        superpoly_list s;
        s = gen_runway_center_line_lights( rwy_info, false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }

        // reverse direction
        s = gen_runway_center_line_lights( rwy_info, true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // Touchdown zone lighting
    if ( rwy_info.end1_flags.substr(0,1) == "Y" ) {
        FGSuperPoly s = gen_touchdown_zone_lights( rwy_info, alt_m, false );
        lights.push_back( s );
    }
    if ( rwy_info.end2_flags.substr(0,1) == "Y" ) {
        FGSuperPoly s = gen_touchdown_zone_lights( rwy_info, alt_m, true );
        lights.push_back( s );
    }

    // REIL lighting
    if ( rwy_info.end1_flags.substr(1,1) == "Y" ) {
        FGSuperPoly s = gen_reil( rwy_info, alt_m, false );
        lights.push_back( s );
    }
    if ( rwy_info.end2_flags.substr(1,1) == "Y" ) {
        FGSuperPoly s = gen_reil( rwy_info, alt_m, true );
        lights.push_back( s );
    }

    // PAPI lighting
    if ( rwy_info.end1_flags.substr(2,1) == "P" ) {
        FGSuperPoly s = gen_papi( rwy_info, alt_m, false );
        lights.push_back( s );
    }
    if ( rwy_info.end2_flags.substr(2,1) == "P" ) {
        FGSuperPoly s = gen_papi( rwy_info, alt_m, true );
        lights.push_back( s );
    }

    // VASI lighting
    if ( rwy_info.end1_flags.substr(2,1) == "V" ) {
        FGSuperPoly s = gen_vasi( rwy_info, alt_m, false );
        lights.push_back( s );
    }
    if ( rwy_info.end2_flags.substr(2,1) == "V" ) {
        FGSuperPoly s = gen_vasi( rwy_info, alt_m, true );
        lights.push_back( s );
    }

    // Approach lighting

    ////////////////////////////////////////////////////////////
    // NOT IMPLIMENTED:
    //
    // code "A" == ALS Approach light system (assumed white lights)
    // 
    // Please send me documentation for this configuration
    ////////////////////////////////////////////////////////////

    // ALSF-I
    if ( rwy_info.end1_flags.substr(3,1) == "B" ) {
        superpoly_list s = gen_alsf( rwy_info, alt_m, "1", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "B" ) {
        superpoly_list s = gen_alsf( rwy_info, alt_m, "1", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // ALSF-II
    if ( rwy_info.end1_flags.substr(3,1) == "C" ) {
        superpoly_list s = gen_alsf( rwy_info, alt_m, "2", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "C" ) {
        superpoly_list s = gen_alsf( rwy_info, alt_m, "2", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    ////////////////////////////////////////////////////////////
    // NOT IMPLIMENTED:
    //
    // code: "D" CAL Calvert (British)
    //
    // code: "E" CAL-II Calvert (British) - Cat II and II
    // 
    // Please send me documentation for this configuration
    ////////////////////////////////////////////////////////////

    // LDIN

    // MALS
    if ( rwy_info.end1_flags.substr(3,1) == "G" ) {
        superpoly_list s = gen_malsx( rwy_info, alt_m, "x", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "G" ) {
        superpoly_list s = gen_malsx( rwy_info, alt_m, "x", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // MALSF
    if ( rwy_info.end1_flags.substr(3,1) == "H" ) {
        superpoly_list s = gen_malsx( rwy_info, alt_m, "F", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "H" ) {
        superpoly_list s = gen_malsx( rwy_info, alt_m, "F", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // MALSR
    if ( rwy_info.end1_flags.substr(3,1) == "J" ) {
        superpoly_list s = gen_malsx( rwy_info, alt_m, "R", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "J" ) {
        superpoly_list s = gen_malsx( rwy_info, alt_m, "R", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // SALS (Essentially ALSF-1 without the lead in rabbit lights, and
    // a shorter center bar)

    if ( rwy_info.end1_flags.substr(3,1) == "O" ) {
        superpoly_list s = gen_alsf( rwy_info, alt_m, "O", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "O" ) {
        superpoly_list s = gen_alsf( rwy_info, alt_m, "O", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // SSALS
    if ( rwy_info.end1_flags.substr(3,1) == "P" ) {
        superpoly_list s = gen_ssalx( rwy_info, alt_m, "P", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "P" ) {
        superpoly_list s = gen_ssalx( rwy_info, alt_m, "P", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // SSALF
    if ( rwy_info.end1_flags.substr(3,1) == "Q" ) {
        superpoly_list s = gen_ssalx( rwy_info, alt_m, "F", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "Q" ) {
        superpoly_list s = gen_ssalx( rwy_info, alt_m, "F", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // SSALR
    if ( rwy_info.end1_flags.substr(3,1) == "R" ) {
        superpoly_list s = gen_ssalx( rwy_info, alt_m, "R", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(3,1) == "R" ) {
        superpoly_list s = gen_ssalx( rwy_info, alt_m, "R", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }

    // Many aproach lighting systems define the threshold lighting
    // needed, but for those that don't (i.e. REIL) make threshold
    // lighting
    if ( rwy_info.end1_flags.substr(1,1) == "Y" ) {
        // forward direction
        superpoly_list s = gen_runway_threshold_lights( rwy_info, edge_type,
                                                        alt_m, false );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
    if ( rwy_info.end2_flags.substr(1,1) == "Y" ) {
        // reverse direction
        superpoly_list s = gen_runway_threshold_lights( rwy_info, edge_type,
                                                        alt_m, true );
        for ( i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
}
