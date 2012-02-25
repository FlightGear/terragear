// lights.cxx -- Generate runway lighting
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <cstdlib>

#include <simgear/math/sg_geodesy.hxx>

#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>

#include "runway.hxx"

using std::cout;
using std::endl;
using std::string;


// calculate the runway light direction vector.  We take the center of
// one runway end - the center of the other end to get the direction
// of the runway.  Combine this with an appropriate portion of the
// local 'up' vector based on the provide 'angle' gives the light
// direction vector for the runway.
Point3D Runway::gen_runway_light_vector( double angle, bool recip ) {
    double length;

    // Generate the 4 corners of the runway
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 0.0, 0.0, 0.0, 0.0 );
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
    SG_LOG(SG_GENERAL, SG_DEBUG, "cart1 = " << cart1 << " cart2 = " << cart2);

    Point3D up = cart1;
    length = up.distance3D( Point3D(0.0) );
    up = up / length;

    Point3D rwy_vec = cart2 - cart1;
    SG_LOG(SG_GENERAL, SG_DEBUG, "rwy_vec = " << rwy_vec);

    // angle up specified amount
    length = rwy_vec.distance3D( Point3D(0.0) );
    double up_length = length * tan(angle * SG_DEGREES_TO_RADIANS);
    Point3D light_vec = rwy_vec + (up * up_length);

    length = light_vec.distance3D( Point3D(0.0) );
    light_vec = light_vec / length;

    return light_vec;
}

// generate runway edge lighting
// 60 meters spacing or the next number down that divides evenly.
superpoly_list Runway::gen_runway_edge_lights( bool recip )
{
    point_list w_lights; w_lights.clear();
    point_list y_lights; y_lights.clear();
    point_list w_normals; w_normals.clear();
    point_list y_normals; y_normals.clear();
    int i;

    int divs = (int)(rwy.length / 60.0) + 1;

    Point3D normal = gen_runway_light_vector( 3.0, recip );

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    2.0 );

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

    double dist = rwy.length;
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

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    TGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    //Different intensities
    if (rwy.edge_lights == 3)
        white.set_material( "RWY_WHITE_LIGHTS" );
    else if (rwy.edge_lights == 2)
        white.set_material( "RWY_WHITE_MEDIUM_LIGHTS" );
    else if (rwy.edge_lights == 1)
        white.set_material( "RWY_WHITE_LOW_LIGHTS" );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( y_lights, false );
    normals_poly.add_contour( y_normals, false );

    TGSuperPoly yellow;
    yellow.set_poly( lights_poly );
    yellow.set_normals( normals_poly );
    //Different intensities
    if (rwy.edge_lights == 3)
        yellow.set_material( "RWY_YELLOW_LIGHTS" );
    else if (rwy.edge_lights == 2)
        yellow.set_material( "RWY_YELLOW_MEDIUM_LIGHTS" );
    else if (rwy.edge_lights == 1)
        yellow.set_material( "RWY_YELLOW_LOW_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( white );
    result.push_back( yellow );

    return result;
}


// generate taxiway edge lighting
// 100 meters spacing or the next number down that divides evenly.
superpoly_list Runway::gen_taxiway_edge_lights( const int kind, bool recip )
{
    point_list b_lights; b_lights.clear();
    point_list b_normals; b_normals.clear();
    int i;

    double len = rwy.length;
    int divs;
    if ( len > 100.0 ) {
        // for lengths of 300' or more, max spacing is 200'
        divs = (int)(len / 70.0) + 1;
    } else {
        // for lengths <= 300', max spacing = 100'
        divs = (int)(len / 35.0) + 1;
    }

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0, 0.0, 0.0, 2.0 );

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

    double dist = rwy.length;
    double step = dist / divs;

    Point3D up = sgGeodToCart( corner[0] * SG_DEGREES_TO_RADIANS );
    double length = up.distance3D( Point3D(0.0) );
    up = up / length;

    b_lights.push_back( pt1 );
    b_normals.push_back( up );
    b_lights.push_back( pt2 );
    b_normals.push_back( up );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt1 += inc1;
	pt2 += inc2;
        b_lights.push_back( pt1 );
        b_normals.push_back( up );
        b_lights.push_back( pt2 );
        b_normals.push_back( up );
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( b_lights, false );
    normals_poly.add_contour( b_normals, false );

    TGSuperPoly blue;
    blue.set_poly( lights_poly );
    blue.set_normals( normals_poly );
    blue.set_material( "RWY_BLUE_TAXIWAY_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( blue );

    return result;
}


// generate threshold lights for a 3 degree approach 
superpoly_list Runway::gen_runway_threshold_lights( const int kind,
                                                   float alt_m, bool recip )
{
    point_list g_lights; g_lights.clear();
    point_list g_normals; g_normals.clear();
    point_list r_lights; r_lights.clear();
    point_list r_normals; r_normals.clear();
    int i;

    cout << "gen threshold " << rwy.rwnum[0] << endl;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 0.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    0.0 );

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
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref1 = corner[2];
        ref2 = corner[3];
        ref3 = corner[1];
        ref4 = corner[0];
        length_hdg = rwy.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    Point3D normal1 = gen_runway_light_vector( 3.0, recip );
    Point3D normal2 = gen_runway_light_vector( 3.0, !recip );

    // offset 5' downwind
    geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), length_hdg, 
                        -5 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref1 = Point3D( lon, lat, 0.0 );
    geo_direct_wgs_84 ( alt_m, ref2.lat(), ref2.lon(), length_hdg, 
                        -5 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref2 = Point3D( lon, lat, 0.0 );

    // five lights for each side
    for ( i = 0; i < 5; ++i ) {
        g_lights.push_back( ref1 );
        g_normals.push_back( normal1 );
    
        g_lights.push_back( ref2 );
        g_normals.push_back( normal1 );

        r_lights.push_back( ref1 );
        r_normals.push_back( normal2 );
    
        r_lights.push_back( ref2 );
        r_normals.push_back( normal2 );

        // offset 10' towards center
        geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), left_hdg, 
                            -10 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref1 = Point3D( lon, lat, 0.0 );
        geo_direct_wgs_84 ( alt_m, ref2.lat(), ref2.lon(), left_hdg, 
                            10 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref2 = Point3D( lon, lat, 0.0 );
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    TGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );
    /* other intensities for when the data file supports it
     *
     * green.set_material( "RWY_GREEN_MEDIUM_LIGHTS" );
     * green.set_material( "RWY_GREEN_LOW_LIGHTS" );
     */

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    TGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );
    /* other intensities for when the data file supports it
     *
     * red.set_material( "RWY_RED_MEDIUM_LIGHTS" );
     * red.set_material( "RWY_RED_LOW_LIGHTS" );
     */

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );

    return result;
}


// generate runway center line lighting, 50' spacing.
superpoly_list Runway::gen_runway_center_line_lights( bool recip )
{
    point_list w_lights; w_lights.clear();
    point_list r_lights; r_lights.clear();
    point_list w_normals; w_normals.clear();
    point_list r_normals; r_normals.clear();
    int i;

    double len = rwy.length;
    // this should be 25' technically but I'm trying 50' to space things out
    int divs = (int)(len / (50.0*SG_FEET_TO_METER)) + 1;

    Point3D normal = gen_runway_light_vector( 3.0, recip );

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    2.0 );

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

    superpoly_list result; result.clear();

    if ( w_lights.size() > 0 ) {
        TGPolygon lights_poly; lights_poly.erase();
        TGPolygon normals_poly; normals_poly.erase();
        lights_poly.add_contour( w_lights, false );
        normals_poly.add_contour( w_normals, false );

        TGSuperPoly white;
        white.set_poly( lights_poly );
        white.set_normals( normals_poly );
        white.set_material( "RWY_WHITE_MEDIUM_LIGHTS" );

        result.push_back( white );
    }

    if ( r_lights.size() > 0 ) {
        TGPolygon lights_poly; lights_poly.erase();
        TGPolygon normals_poly; normals_poly.erase();
        lights_poly.add_contour( r_lights, false );
        normals_poly.add_contour( r_normals, false );

        TGSuperPoly red;
        red.set_poly( lights_poly );
        red.set_normals( normals_poly );
        red.set_material( "RWY_RED_MEDIUM_LIGHTS" );

        result.push_back( red );
    }

    return result;
}

/*
// generate taxiway center line lighting, 50' spacing.
static superpoly_list gen_taxiway_center_line_lights( bool recip )
{
    point_list g_lights; g_lights.clear();
    point_list g_normals; g_normals.clear();
    int i;

    double len = rwy.length;
    // this should be ??' technically but I'm trying 50' to space things out
    int divs = (int)(len / 70) + 1;

    Point3D normal = gen_runway_light_vector( 3.0, recip );

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0, 0.0, 0.0, 2.0 );

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

    while ( dist > 0.0 ) {
        g_lights.push_back( pt1 );
        g_normals.push_back( normal );

  	pt1 += inc;
  	pt1 += inc;
        dist -= step;
        dist -= step;
    }

    superpoly_list result; result.clear();

    if ( g_lights.size() > 0 ) {
        TGPolygon lights_poly; lights_poly.erase();
        TGPolygon normals_poly; normals_poly.erase();
        lights_poly.add_contour( g_lights, false );
        normals_poly.add_contour( g_normals, false );

        TGSuperPoly green;
        green.set_poly( lights_poly );
        green.set_normals( normals_poly );
        green.set_material( "RWY_GREEN_TAXIWAY_LIGHTS" );

        result.push_back( green );
    }

    return result;
}
*/

// generate touch down zone lights
TGSuperPoly Runway::gen_touchdown_zone_lights( float alt_m, bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;

    cout << "gen touchdown zone lights " << rwy.rwnum[0] << endl;

    Point3D normal;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 0.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    0.0 );

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
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref = (corner[2] + corner[3]) / 2;
        length_hdg = rwy.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    normal = gen_runway_light_vector( 3.0, recip );

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

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lights, false );
    normals_poly.add_contour( normals, false );

    TGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_WHITE_LIGHTS" );

    return result;
}

/*
// generate a simple 2 bar VASI
static TGSuperPoly gen_vasi( const TGLightobj& rwy_light, float alt_m,
                             bool recip, TGPolygon *apt_base )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;
    string flag;
    double gs_angle = 3.0;

    cout << "gen vasi " << rwy.rwnum[0] << endl;

    Point3D normal;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( rwy_info, 0.0, 0.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    0.0 );

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
        flag = rwy.rwnum[1];
        gs_angle = rwy_info.gs_angle1;
    } else {
        ref = corner[2];
        length_hdg = rwy_info.heading;
        flag = rwy.rwnum[0];
        gs_angle = rwy_info.gs_angle2;
    }

    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    if ( gs_angle < 0.5 ) {
        gs_angle = 3.0;
    }

    // offset 600' upwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        600 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    // offset 50' left
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                        50 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // downwind bar
    normal = gen_runway_light_vector( rwy_info, gs_angle - 0.5, recip );

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

    // grass base
    Point3D base_pt = (ref + pt1) / 2.0;
    TGPolygon obj_base = gen_wgs84_area( base_pt, 15.0, 0.0, 0.0, 15.0,
                                         length_hdg, alt_m, false );
    *apt_base = tgPolygonUnion( obj_base, *apt_base );

    // upwind bar
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        700 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    normal = gen_runway_light_vector( rwy_info, gs_angle, recip );

    // unit1
    pt1 = ref;
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

    // grass base
    base_pt = (ref + pt1) / 2.0;
    obj_base = gen_wgs84_area( base_pt, 15.0, 0.0, 0.0, 15.0,
                               length_hdg, alt_m, false );
    *apt_base = tgPolygonUnion( obj_base, *apt_base );

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lights, false );
    normals_poly.add_contour( normals, false );

    TGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_VASI_LIGHTS" );

    result.set_flag( flag );

    return result;
    
}
*/

// Generate the airports light objects (PAPI/VASI)
/*void gen_airport_lightobj( const TGLightobj& rwy_light, float alt_m, superpoly_list &lights )
{
    point_list lightobj; lightobj.clear();
    point_list normals; normals.clear();

    cout << "gen papi " << rwy_light.rwy_name << endl;

    Point3D ref;
    double lon, lat, r;
    double gs_angle = rwy_light.glideslope;
    double left_hdg = rwy_light.heading - 90.0;

    ref.setlat( rwy_light.lat );
    ref.setlon( rwy_light.lon );

    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    if ( gs_angle < 0.5 ) {
        gs_angle = 3.0;
    }

    // Calculate the normal once for all object parts.
    // SG takes care of the angle.

    // calculate a second point in the object heading direction
    geo_direct_wgs_84 ( rwy_light.lat, rwy_light.lon, rwy_light.heading,
			100, &lat, &lon, &r);

    Point3D end1, end2;

    end1.setlat( lat );
    end1.setlon( lon );

    end2.setlat( rwy_light.lat);
    end2.setlon( rwy_light.lon);

    Point3D cart1 = sgGeodToCart( end1 * SG_DEGREES_TO_RADIANS );
    Point3D cart2 = sgGeodToCart( end2 * SG_DEGREES_TO_RADIANS );

    Point3D up = cart1;
    double length = up.distance3D( Point3D(0.0) );
    up = up / length;

    Point3D obj_vec = cart2 - cart1;

    // angle up specified amount
    length = obj_vec.distance3D( Point3D(0.0) );
    double up_length = length * tan( rwy_light.glideslope * SG_DEGREES_TO_RADIANS);
    Point3D light_vec = obj_vec + (up * up_length);

    length = light_vec.distance3D( Point3D(0.0) );
    Point3D normal = light_vec / length;

    SG_LOG(SG_GENERAL, SG_DEBUG, "obj_normal = " << normal);

    // We know our normal, now create the lights

    // unit1
    Point3D pt1 = ref;
    lightobj.push_back( pt1 );
    normals.push_back( normal );
    
    // unit2
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        30 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lightobj.push_back( pt1 );
    normals.push_back( normal );

    // unit3
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        30 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lightobj.push_back( pt1 );
    normals.push_back( normal );

    // unit4
    geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg, 
                        30 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt1 = Point3D( lon, lat, 0.0 );
    lightobj.push_back( pt1 );
    normals.push_back( normal );

    // grass base
    Point3D base_pt = (ref + pt1) / 2.0;
    TGPolygon obj_base = gen_wgs84_area( base_pt, 15.0, 0.0, 0.0, 30.0,
                                         length_hdg, alt_m, false );
    *apt_base = tgPolygonUnion( obj_base, *apt_base );

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lightobj, false );
    normals_poly.add_contour( normals, false );

    TGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_VASI_LIGHTS" );

    lights.push_back( result);
}
*/

// generate REIL lights
TGSuperPoly Runway::gen_reil( float alt_m, bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;
    string flag;

    cout << "gen reil " << rwy.rwnum[0] << endl;

    Point3D normal;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 0.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    0.0 );

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
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
        flag = rwy.rwnum[1];
    } else {
        ref1 = corner[2];
        ref2 = corner[3];
        length_hdg = rwy.heading;
        flag = rwy.rwnum[0];
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
    normal = gen_runway_light_vector( 10, recip );
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
    normal = gen_runway_light_vector( 10, recip );
    normals.push_back( normal );

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lights, false );
    normals_poly.add_contour( normals, false );

    TGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_REIL_LIGHTS" );

    result.set_flag( flag );

    return result;
}


// generate Calvert-I/II approach lighting schemes
superpoly_list Runway::gen_calvert( float alt_m, const string &kind, bool recip )
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
    string flag;
    if ( kind == "1" ) {
        cout << "gen Calvert lights " << rwy.rwnum[0] << endl;
    } else if ( kind == "2" ) {
	cout << "gen Calvert/II lights " << rwy.rwnum[0] << endl;
    } else {
	cout << "gen unknown Calvert lights " << rwy.rwnum[0] << endl;
    }

    Point3D normal1 = gen_runway_light_vector( 3.0, recip );
    Point3D normal2 = gen_runway_light_vector( 3.0, !recip );

    // Generate the threshold lights

    double len = rwy.length;
    int divs = (int)(len / 10.0) + 1;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    2.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc;
    Point3D pt;

    if ( recip ) {
        inc = (corner[0] - corner[1]) / divs;
        pt = corner[1];
        flag = rwy.rwnum[1];
    } else {
        inc = (corner[2] - corner[3]) / divs;
        pt = corner[3];
        flag = rwy.rwnum[0];
    }

    double dist = rwy.length;
    double step = dist / divs;

    g_lights.push_back( pt );
    g_normals.push_back( normal1 );
    r_lights.push_back( pt );
    r_normals.push_back( normal2 );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt += inc;
        g_lights.push_back( pt );
        g_normals.push_back( normal1 );
        r_lights.push_back( pt );
        r_normals.push_back( normal2 );
        dist -= step;
    }

    // Generate long center bar of lights

    // determine the start point.
    Point3D ref_save;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref_save = (corner[0] + corner[1]) / 2;
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref_save = (corner[2] + corner[3]) / 2;
        length_hdg = rwy.heading;
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    Point3D ref = ref_save;
    //
    // Centre row of lights 1xlights out to 300m
    // 2 x lights from 300m to 600m
    // 3 x lights from 600m to 900m
    // light spacing is 30m
    // 
    // calvert2 has reds instead of whites out to 300m
#define CALVERT_HORIZ_SPACING	30
#define CALVERT_VERT_SPACING	10
#define CALVERT2_VERT_SPACING	2

    int count;
    //if ( kind == "1" || kind == "2" ) {
    //    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
    //                        -100 * SG_FEET_TO_METER, &lat, &lon, &r );
    //    ref = Point3D( lon, lat, 0.0 );
    //    count = 10;
    //}
    count=30;

    Point3D saved;
    Point3D crossbar[5];
    Point3D pair;
    // first set of single lights
    for ( i = 0; i < count; ++i ) {
        pt = ref;

        // centre lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), length_hdg, 
                            -1 * CALVERT_HORIZ_SPACING, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );

	if (kind == "1" ) {
            if ( i >= 10 && i < 20 ) {
                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                    CALVERT_VERT_SPACING/2, &lat, &lon, &r );
                pair = Point3D( lon, lat, 0.0 );
                w_lights.push_back( pair );
                w_normals.push_back( normal1 );

                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                    -1 * CALVERT_VERT_SPACING/2, &lat, &lon,
                                    &r );
                pair = Point3D( lon, lat, 0.0 );
                w_lights.push_back( pair );
                w_normals.push_back( normal1 );
            } else if (i >= 20)	{
                w_lights.push_back( pt );
                w_normals.push_back( normal1 );

                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                    CALVERT_VERT_SPACING, &lat, &lon, &r );
                pair = Point3D( lon, lat, 0.0 );
                w_lights.push_back( pair );
                w_normals.push_back( normal1 );

                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                    -1 * CALVERT_VERT_SPACING, &lat, &lon, &r );
                pair = Point3D( lon, lat, 0.0 );
                w_lights.push_back( pair );
                w_normals.push_back( normal1 );
            } else {
                w_lights.push_back( pt );
                w_normals.push_back( normal1 );
            }
	} else {
	    if ( i < 10 ) {
		// cal2 has red centre lights
        	r_lights.push_back( pt );
        	r_normals.push_back( normal1 );
	    } else {
		// cal2 has red centre lights
        	w_lights.push_back( pt );
        	w_normals.push_back( normal1 );
	    }
	}

	switch ( i ) {
	case 4:
            crossbar[0] = pt;
            break;
	case 9:
            crossbar[1] = pt;
            break;
	case 14:
            crossbar[2] = pt;
            break;
	case 19:
            crossbar[3] = pt;
            break;
	case 24:
            crossbar[4] = pt;
            break;
	}
			
	// add 2 more rows if CAL/II (white)
	//
	
	if ( kind == "2" ) {
            saved = pt;
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                CALVERT2_VERT_SPACING, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            w_lights.push_back( pt );
            w_normals.push_back( normal1 );

            // five rows < 300m
            if ( i < 10 ) {
                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                    CALVERT2_VERT_SPACING, &lat, &lon, &r );
                pt = Point3D( lon, lat, 0.0 );
                w_lights.push_back( pt );
                w_normals.push_back( normal1 );

                // outer strip of lights
                for (j=0;j<9;j++) {
                    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg,
                                        CALVERT2_VERT_SPACING, &lat, &lon, &r );
                    pt = Point3D( lon, lat, 0.0 );
                    if ( i == 0 || j > 3 ) {
                        w_lights.push_back( pt );
                        w_normals.push_back( normal1 );
                    }
                }
            }

            pt = saved;
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                -1 * CALVERT2_VERT_SPACING, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            w_lights.push_back( pt );
            w_normals.push_back( normal1 );

            // five rows < 300m
            if ( i < 10 ) {
                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                    -1 * CALVERT2_VERT_SPACING, &lat, &lon,
                                    &r );
                pt = Point3D( lon, lat, 0.0 );
                w_lights.push_back( pt );
                w_normals.push_back( normal1 );
                // outer strip of lights
                for ( j = 0; j < 9; j++ ) {
                    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg,
                                        -1 * CALVERT2_VERT_SPACING, &lat, &lon,
                                        &r );
                    pt = Point3D( lon, lat, 0.0 );
                    if ( i == 0 || j > 3 ) {
                        w_lights.push_back( pt );
                        w_normals.push_back( normal1 );
                    }
                }
            }

            pt = saved;	

	}
	ref = pt;
    
    }

    ref = ref_save;

    int spacing;
    int num_lights = 0;

    // draw nice crossbars
    for ( i = 0; i < 5; i++ ) {
	if (kind == "1") {
            spacing = CALVERT_VERT_SPACING;
	} else {
            spacing = CALVERT2_VERT_SPACING;
	}
	switch ( i ) {
	case 0:
            num_lights = 4;
            break;
	case 1:
            num_lights = 5;
            break;
	case 2:
            num_lights = 6;
            break;
	case 3:
            num_lights = 7;
            break;
	case 4:
            num_lights = 8;
            break;
	}

	pt = crossbar[i];
	for ( j = 0 ; j < num_lights; j++ ) {	
            // left side lights

            // space out from centre lights
            if ( j == 0 ) {
                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg,
                                    CALVERT_VERT_SPACING * j, &lat, &lon, &r );
                pt = Point3D( lon, lat, 0.0 );
            }

            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                spacing, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );

            if ( kind == "1" || i >= 2 ) {
                w_lights.push_back( pt );
                w_normals.push_back( normal1 );
            } else {
                r_lights.push_back( pt );
                r_normals.push_back( normal1 );
            }
	}

	pt = crossbar[i];
	for ( j = 0; j < num_lights; j++ ) {
            // right side lights
            // space out from centre lights
            if ( j == 0 ) {
                geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg,
                                    -1 * CALVERT_VERT_SPACING * j, &lat, &lon,
                                    &r );
                pt = Point3D( lon, lat, 0.0 );
            }

            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                -1 * spacing, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );

            if ( kind == "1" || i >= 2 ) {
                w_lights.push_back( pt );
                w_normals.push_back( normal1 );
            } else {
                r_lights.push_back( pt );
                r_normals.push_back( normal1 );
            }
        }
    }
    
    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    TGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );
    green.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    TGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );
    red.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    TGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_LIGHTS" );
    white.set_flag( flag );

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );
    result.push_back( white );

    if ( s_lights.size() ) {
        lights_poly.erase();
        normals_poly.erase();
        lights_poly.add_contour( s_lights, false );
        normals_poly.add_contour( s_normals, false );

        TGSuperPoly sequenced;
        sequenced.set_poly( lights_poly );
        sequenced.set_normals( normals_poly );
        sequenced.set_material( "RWY_SEQUENCED_LIGHTS" );
        sequenced.set_flag( flag );

        result.push_back( sequenced );
    }
 
    return result;
}

// generate ALSF-I/II and SALS/SALSF approach lighting schemes
superpoly_list Runway::gen_alsf( float alt_m, const string &kind, bool recip )
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
    string flag;

    cout << "gen ALSF/SALS lights " << rwy.rwnum[0] << endl;

    Point3D normal1 = gen_runway_light_vector( 3.0, recip );
    Point3D normal2 = gen_runway_light_vector( 3.0, !recip );

    // Generate the threshold lights

    double len = rwy.length;
    int divs = (int)(len / 10.0) + 1;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    2.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc;
    Point3D pt;

    if ( recip ) {
        inc = (corner[0] - corner[1]) / divs;
        pt = corner[1];
        flag = rwy.rwnum[1];
    } else {
        inc = (corner[2] - corner[3]) / divs;
        pt = corner[3];
        flag = rwy.rwnum[0];
    }

    double dist = rwy.length;
    double step = dist / divs;

    g_lights.push_back( pt );
    g_normals.push_back( normal1 );
    r_lights.push_back( pt );
    r_normals.push_back( normal2 );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt += inc;
        g_lights.push_back( pt );
        g_normals.push_back( normal1 );
        r_lights.push_back( pt );
        r_normals.push_back( normal2 );
        dist -= step;
    }

    // Generate long center bar of lights

    // determine the start point.
    Point3D ref_save;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref_save = (corner[0] + corner[1]) / 2;
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref_save = (corner[2] + corner[3]) / 2;
        length_hdg = rwy.heading;
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
        // SALS/SALSF
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -300 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
        count = 13;
    }

    for ( i = 0; i < count; ++i ) {
        pt = ref;
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        pt = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -100 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
    }

    ref = ref_save;

    if ( kind == "1" || kind == "O" || kind == "P" ) {
        // Terminating bar

        // offset 200' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -200 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        pt = ref;
 
        // left 3 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            15 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );

        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );

        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );

        pt = ref;

        // right 3 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -15 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );
    } else if ( kind == "2" ) {
        // Generate red side row lights

        for ( i = 0; i < 9; ++i ) {
            // offset 100' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -100 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );

            pt = ref;
 
            // left 3 side lights
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                36 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );

            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );

            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );

            pt = ref;

            // right 3 side lights
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                -36 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );
    
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                -5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );
    
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                -5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );
        }
    }

    if ( kind == "1" || kind == "O" || kind == "P" ) {
        // Generate pre-threshold bar

        ref = ref_save;

        // offset 100' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -100 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
    
        pt = ref;

        // left 5 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            75 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );

        for ( j = 0; j < 4; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );
        }

        pt = ref;

        // rioght 5 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -75 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );

        for ( j = 0; j < 4; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            r_lights.push_back( pt );
            r_normals.push_back( normal1 );
        }
    } else if ( kind == "2" ) {
        // Generate -500 extra horizontal row of lights

        ref = ref_save;

        // offset 500' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -500 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
    
        pt = ref;

        // left 4 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            11.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        for ( j = 0; j < 3; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            w_lights.push_back( pt );
            w_normals.push_back( normal1 );
        }

        pt = ref;

        // right 4 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -11.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        for ( j = 0; j < 3; ++j ) {
            geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                                -5 * SG_FEET_TO_METER, &lat, &lon, &r );
            pt = Point3D( lon, lat, 0.0 );
            w_lights.push_back( pt );
            w_normals.push_back( normal1 );
        }
    }

    ref = ref_save;

    if ( kind == "O" || kind == "P" ) {
        // generate SALS secondary threshold

        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -200 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
        count = 30;

        pt = ref;
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );

        pt = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        r_lights.push_back( pt );
        r_normals.push_back( normal1 );
    }

    // Generate -1000' extra horizontal row of lights

    ref = ref_save;

    // offset 1000' downwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    
    pt = ref;
 
    // left 8 side lights
    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                        15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt );
    w_normals.push_back( normal1 );

    for ( j = 0; j < 7; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    }

    pt = ref;
 
    // right 8 side lights
    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                        -15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt );
    w_normals.push_back( normal1 );

    for ( j = 0; j < 7; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
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
            s_normals.push_back( normal1 );

            // offset 100' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -100 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    } else if ( kind == "P" ) {
        // generate 3 sequenced lights aligned with last 3 light bars

        // start 1300' downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -1300 * SG_FEET_TO_METER, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );

        for ( i = 0; i < 3; ++i ) {
            s_lights.push_back( ref );
            s_normals.push_back( normal1 );
             
            // offset 100' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -100 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    TGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );
    green.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    TGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );
    red.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    TGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_LIGHTS" );
    white.set_flag( flag );

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );
    result.push_back( white );

    if ( s_lights.size() ) {
        lights_poly.erase();
        normals_poly.erase();
        lights_poly.add_contour( s_lights, false );
        normals_poly.add_contour( s_normals, false );

        TGSuperPoly sequenced;
        sequenced.set_poly( lights_poly );
        sequenced.set_normals( normals_poly );
        sequenced.set_material( "RWY_SEQUENCED_LIGHTS" );
        sequenced.set_flag( flag );

        result.push_back( sequenced );
    }
 
    return result;
}


// generate ODALS lights
TGSuperPoly Runway::gen_odals( float alt_m, bool recip )
{
    point_list lights; lights.clear();
    point_list normals; normals.clear();
    int i;
    string flag;

    cout << "gen odals " << rwy.rwnum[0] << endl;

    // ODALS lighting is omni-directional, but we generate a normal as
    // a placeholder to keep everything happy.
    Point3D normal( 0.0, 0.0, 0.0 );

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 0.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    0.0 );

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
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
        flag = rwy.rwnum[1];
    } else {
        ref1 = corner[2];
        ref2 = corner[3];
        length_hdg = rwy.heading;
        flag = rwy.rwnum[0];
    }
    left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }
    cout << "length hdg = " << length_hdg
         << " left heading = " << left_hdg << endl;

    Point3D ref = ( ref1 + ref2 ) / 2.0;

    // offset 40' downwind
    geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), length_hdg, 
                        -40 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref1 = Point3D( lon, lat, 0.0 );
    // offset 40' left
    geo_direct_wgs_84 ( alt_m, ref1.lat(), ref1.lon(), left_hdg, 
                        40 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref1 = Point3D( lon, lat, 0.0 );

    lights.push_back( ref1 );
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
    normals.push_back( normal );

    for ( i = 0; i < 5; ++i ) {
        // offset 100m downwind
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                            -100, &lat, &lon, &r );
        ref = Point3D( lon, lat, 0.0 );
        lights.push_back( ref );
        normals.push_back( normal );
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lights, false );
    normals_poly.add_contour( normals, false );

    TGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_ODALS_LIGHTS" );

    result.set_flag( flag );

    return result;
}


// generate SSALS, SSALF, and SSALR approach lighting scheme (kind =
// S, F, or R)
superpoly_list Runway::gen_ssalx( float alt_m, const string& kind, bool recip )
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
    string flag;

    cout << "gen SSALx lights " << rwy.rwnum[0] << endl;

    Point3D normal1 = gen_runway_light_vector( 3.0, recip );
    Point3D normal2 = gen_runway_light_vector( 3.0, !recip );

    // Generate the threshold lights

    double len = rwy.length;
    int divs = (int)(len / 10.0) + 1;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    2.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc;
    Point3D pt;

    if ( recip ) {
        inc = (corner[0] - corner[1]) / divs;
        pt = corner[1];
        flag = rwy.rwnum[1];
    } else {
        inc = (corner[2] - corner[3]) / divs;
        pt = corner[3];
        flag = rwy.rwnum[0];
    }

    double dist = rwy.length;
    double step = dist / divs;

    g_lights.push_back( pt );
    g_normals.push_back( normal1 );
    r_lights.push_back( pt );
    r_normals.push_back( normal2 );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt += inc;
        g_lights.push_back( pt );
        g_normals.push_back( normal1 );
        r_lights.push_back( pt );
        r_normals.push_back( normal2 );
        dist -= step;
    }

    // Generate long center bar of lights (every 200')

    // determine the start point.
    Point3D ref_save;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref_save = (corner[0] + corner[1]) / 2;
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref_save = (corner[2] + corner[3]) / 2;
        length_hdg = rwy.heading;
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

        pt = ref;
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        pt = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -3.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    }

    // Generate -1000' extra horizontal row of lights

    ref = ref_save;

    // offset 1000' downwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    
    pt = ref;
 
    // left 5 side lights
    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                        15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt );
    w_normals.push_back( normal1 );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    }

    pt = ref;
 
    // right 5 side lights
    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                        -15 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt );
    w_normals.push_back( normal1 );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
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
            s_normals.push_back( normal1 );

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
            s_normals.push_back( normal1 );
             
            // offset 200' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -200 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    TGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );
    green.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    TGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );
    red.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    TGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_LIGHTS" );
    white.set_flag( flag );

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );
    result.push_back( white );

    if ( s_lights.size() > 0 ) {
        lights_poly.erase();
        normals_poly.erase();
        lights_poly.add_contour( s_lights, false );
        normals_poly.add_contour( s_normals, false );

        TGSuperPoly sequenced;
        sequenced.set_poly( lights_poly );
        sequenced.set_normals( normals_poly );
        sequenced.set_material( "RWY_SEQUENCED_LIGHTS" );
        sequenced.set_flag( flag );

        result.push_back( sequenced );
    }
 
    return result;
}


// generate MALS, MALSF, and MALSR approach lighting scheme (kind =
// ' ', F, or R)
superpoly_list Runway::gen_malsx( float alt_m, const string& kind, bool recip )
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
    string flag;

    cout << "gen SSALx lights " << rwy.rwnum[0] << endl;

    Point3D normal1 = gen_runway_light_vector( 3.0, recip );
    Point3D normal2 = gen_runway_light_vector( 3.0, !recip );

    // Generate the threshold lights

    double len = rwy.length;
    int divs = (int)(len / 10.0) + 1;

    // using TGPolygon is a bit innefficient, but that's what the
    // routine returns.
    TGPolygon poly_corners
        = gen_runway_area_w_extend( 0.0, 2.0,
                                    rwy.threshold[0],
                                    rwy.threshold[1],
                                    2.0 );

    point_list corner;
    for ( i = 0; i < poly_corners.contour_size( 0 ); ++i ) {
	corner.push_back( poly_corners.get_pt( 0, i ) );
    }

    Point3D inc;
    Point3D pt;

    if ( recip ) {
        inc = (corner[0] - corner[1]) / divs;
        pt = corner[1];
        flag = rwy.rwnum[1];
    } else {
        inc = (corner[2] - corner[3]) / divs;
        pt = corner[3];
        flag = rwy.rwnum[0];
    }

    double dist = rwy.length;
    double step = dist / divs;

    g_lights.push_back( pt );
    g_normals.push_back( normal1 );
    r_lights.push_back( pt );
    r_normals.push_back( normal2 );
    dist -= step;

    for ( i = 0; i < divs; ++i ) {
	pt += inc;
        g_lights.push_back( pt );
        g_normals.push_back( normal1 );
        r_lights.push_back( pt );
        r_normals.push_back( normal2 );
        dist -= step;
    }

    // Generate long center bar of lights (every 200')

    // determine the start point.
    Point3D ref_save;
    double length_hdg, left_hdg;
    double lon, lat, r;
    if ( recip ) {
        ref_save = (corner[0] + corner[1]) / 2;
        length_hdg = rwy.heading + 180.0;
        if ( length_hdg > 360.0 ) { length_hdg -= 360.0; }
    } else {
        ref_save = (corner[2] + corner[3]) / 2;
        length_hdg = rwy.heading;
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

        pt = ref;
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        // left 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );

        pt = ref;

        // right 2 side lights
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    }

    // Generate -1000' extra horizontal row of lights

    ref = ref_save;

    // offset 1000' downwind
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        -1000 * SG_FEET_TO_METER, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );
    
    pt = ref;
 
    // left 5 side lights
    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                        23 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt );
    w_normals.push_back( normal1 );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
    }

    pt = ref;
 
    // right 5 side lights
    geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                        -23 * SG_FEET_TO_METER, &lat, &lon, &r );
    pt = Point3D( lon, lat, 0.0 );
    w_lights.push_back( pt );
    w_normals.push_back( normal1 );

    for ( j = 0; j < 4; ++j ) {
        geo_direct_wgs_84 ( alt_m, pt.lat(), pt.lon(), left_hdg, 
                            -2.5 * SG_FEET_TO_METER, &lat, &lon, &r );
        pt = Point3D( lon, lat, 0.0 );
        w_lights.push_back( pt );
        w_normals.push_back( normal1 );
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
            s_normals.push_back( normal1 );

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
            s_normals.push_back( normal1 );
             
            // offset 200' downwind
            geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                                -200 * SG_FEET_TO_METER, &lat, &lon, &r );
            ref = Point3D( lon, lat, 0.0 );
        }
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    TGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );
    green.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( r_lights, false );
    normals_poly.add_contour( r_normals, false );

    TGSuperPoly red;
    red.set_poly( lights_poly );
    red.set_normals( normals_poly );
    red.set_material( "RWY_RED_LIGHTS" );
    red.set_flag( flag );

    lights_poly.erase();
    normals_poly.erase();
    lights_poly.add_contour( w_lights, false );
    normals_poly.add_contour( w_normals, false );

    TGSuperPoly white;
    white.set_poly( lights_poly );
    white.set_normals( normals_poly );
    white.set_material( "RWY_WHITE_LIGHTS" );
    white.set_flag( flag );

    superpoly_list result; result.clear();

    result.push_back( green );
    result.push_back( red );
    result.push_back( white );

    if ( s_lights.size() > 0 ) {
        lights_poly.erase();
        normals_poly.erase();
        lights_poly.add_contour( s_lights, false );
        normals_poly.add_contour( s_normals, false );

        TGSuperPoly sequenced;
        sequenced.set_poly( lights_poly );
        sequenced.set_normals( normals_poly );
        sequenced.set_material( "RWY_SEQUENCED_LIGHTS" );
        sequenced.set_flag( flag );

        result.push_back( sequenced );
    }
 
    return result;
}


// top level runway light generator
void Runway::gen_runway_lights( float alt_m, superpoly_list *lights ) {

    unsigned int i;

    // Make edge lighting
    if ( rwy.edge_lights > 0 /* Has edge lighting */ ) {
        // forward direction
        superpoly_list s = gen_runway_edge_lights( false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }

        // reverse direction
        s = gen_runway_edge_lights( true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    // Centerline lighting
    if ( rwy.centerline_lights == 1 /* Has centerline lighting */ ) {
        // forward direction
        superpoly_list s = gen_runway_center_line_lights( false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }

	s = gen_runway_center_line_lights( true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    // Touchdown zone lighting
    if ( rwy.tz_lights[0] == 1 /* Has touchdown zone lighting */ ) {
        TGSuperPoly s = gen_touchdown_zone_lights( alt_m, false );
        lights->push_back( s );
    }
    if ( rwy.tz_lights[1] == 1 /* Has touchdown zone lighting */ ) {
        TGSuperPoly s = gen_touchdown_zone_lights( alt_m, true );
        lights->push_back( s );
    }

    // REIL lighting
    if ( rwy.reil[0] > 0 /* Has REIL lighting */ ) {
        TGSuperPoly s = gen_reil( alt_m, false );
        lights->push_back( s );
    }
    if ( rwy.reil[1] > 0 /* Has REIL lighting */ ) {
        TGSuperPoly s = gen_reil( alt_m, true );
        lights->push_back( s );
    }


    // Approach lighting

    if ( rwy.approach_lights[0] == 1 /* ALSF-I */ ) {
        superpoly_list s = gen_alsf( alt_m, "1", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 1 /* ALSF-I */ ) {
        superpoly_list s = gen_alsf( alt_m, "1", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 2 /* ALSF-II */ ) {
        superpoly_list s = gen_alsf( alt_m, "2", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if (rwy.approach_lights[1]  == 2 /* ALSF-II */ ) {
        superpoly_list s = gen_alsf( alt_m, "2", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 3  /* Calvert I */ ) {
        superpoly_list s = gen_calvert( alt_m, "1", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 3  /* Calvert I */ ) {
        superpoly_list s = gen_calvert( alt_m, "1", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 4  /* Calvert II */ ) {
        superpoly_list s = gen_calvert( alt_m, "2", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 4  /* Calvert II */ ) {
        superpoly_list s = gen_calvert( alt_m, "2", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 5 /* SSALR */ ) {
        superpoly_list s = gen_ssalx( alt_m, "R", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 5 /* SSALR */ ) {
        superpoly_list s = gen_ssalx( alt_m, "R", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 6 /* SSALF */ ) {
        superpoly_list s = gen_ssalx( alt_m, "F", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 6 /* SSALF */ ) {
        superpoly_list s = gen_ssalx( alt_m, "F", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    // SALS (Essentially ALSF-1 without the lead in rabbit lights, and
    // a shorter center bar)
    if ( rwy.approach_lights[0] == 7 /* SALS */ ) {
        superpoly_list s = gen_alsf( alt_m, "O", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 7 /* SALS */ ) {
        superpoly_list s = gen_alsf( alt_m, "O", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 8 /* MALSR */ ) {
        superpoly_list s = gen_malsx( alt_m, "R", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 8 /* MALSR */ ) {
        superpoly_list s = gen_malsx( alt_m, "R", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 9 /* MALSF */ ) {
        superpoly_list s = gen_malsx( alt_m, "F", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 9 /* MALSF */ ) {
        superpoly_list s = gen_malsx( alt_m, "F", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 10 /* MALSX */ ) {
        superpoly_list s = gen_malsx( alt_m, "x", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == 10 /* MALSX */ ) {
        superpoly_list s = gen_malsx( alt_m, "x", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == 11 /* ODALS Omni-directional approach light system */ ) {
        TGSuperPoly s = gen_odals( alt_m, false );
        lights->push_back( s );
    }
    if ( rwy.approach_lights[1] == 11 /* ODALS Omni-directional approach light system */ ) {
        TGSuperPoly s = gen_odals( alt_m, true );
        lights->push_back( s );
    }

    ////////////////////////////////////////////////////////////
    // NOT IMPLIMENTED:
    //
    // code: 12 - RAIL Runway alignment indicator lights (icw other systems)
    //
    ////////////////////////////////////////////////////////////


    if ( rwy.approach_lights[0] == -1 /* SALSF not supported by database */ ) {
        superpoly_list s = gen_alsf( alt_m, "P", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == -1 /* SALSF not supported by database */ ) {
        superpoly_list s = gen_alsf( alt_m, "P", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }

    if ( rwy.approach_lights[0] == -1 /* SSALS not supported by database */ ) {
        superpoly_list s = gen_ssalx( alt_m, "S", false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.approach_lights[1] == -1 /* SSALS not supported by database */ ) {
        superpoly_list s = gen_ssalx( alt_m, "S", true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }


    SG_LOG(SG_GENERAL, SG_DEBUG, "Edge light = " << rwy.edge_lights << " Centre light = " << rwy.centerline_lights );
    SG_LOG(SG_GENERAL, SG_DEBUG, "ALC1 flag = " << rwy.approach_lights[0] << " ALC2 flag2 = " << rwy.approach_lights[1] );

    // Many aproach lighting systems define the threshold lighting
    // needed, but for those that don't (i.e. REIL, ODALS, or Edge
    // lights defined but no approach lights)
    // make threshold lighting

    if ( rwy.reil[0] > 0 /* Has REIL lighting */
         || rwy.approach_lights[0] == 11 /* ODALS Omni-directional approach light system */
         || ( rwy.edge_lights > 0  && rwy.approach_lights[0] == 0 ) /* Has edge lighting, but no
                                             approach lighting */ )
    {
        // forward direction
        SG_LOG(SG_GENERAL, SG_DEBUG, "threshold lights for forward direction" );
        superpoly_list s = gen_runway_threshold_lights( 0,
                                                        alt_m, false );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
    if ( rwy.reil[1] > 0 /* Has REIL lighting */
         || rwy.approach_lights[1] == 11 /* ODALS Omni-directional approach light system */
         || ( rwy.edge_lights > 0  && rwy.approach_lights[1] == 0 ) /* Has edge lighting, but no
                                             approach lighting */ )
    {
        // reverse direction
        SG_LOG(SG_GENERAL, SG_DEBUG, "threshold lights for reverse direction" );
        superpoly_list s = gen_runway_threshold_lights( 0,
                                                        alt_m, true );
        for ( i = 0; i < s.size(); ++i ) {
            lights->push_back( s[i] );
        }
    }
}
