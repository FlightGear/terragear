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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <cstdlib>

#include <simgear/math/SGMath.hxx>
#include <simgear/debug/logstream.hxx>

#include "runway.hxx"

using std::string;

// calculate the runway light direction vector.  We take both runway
// ends to get the direction of the runway.
SGVec3f Runway::gen_runway_light_vector( float angle, bool recip ) {
    SGVec3f cart1, cart2;
    if ( !recip ) {
        cart1 = normalize(SGVec3f::fromGeod(GetStart()));
        cart2 = normalize(SGVec3f::fromGeod(GetEnd()));
    } else {
        cart2 = normalize(SGVec3f::fromGeod(GetStart()));
        cart1 = normalize(SGVec3f::fromGeod(GetEnd()));
    }

    SGVec3f runway_vec = normalize(cart1 - cart2);
    SGVec3f horizontal(normalize(cross(cart1, runway_vec)));
    SGQuatf rotation = SGQuatf::fromAngleAxisDeg(angle, horizontal);
    SGVec3f light_vec = rotation.transform(runway_vec);

    return light_vec;
}

// generate runway edge lighting
// 60 meters spacing or the next number down that divides evenly.
tglightcontour_list Runway::gen_runway_edge_lights( bool recip )
{
    tgLightContour r_lights;
    tgLightContour w_lights;
    tgLightContour y_lights;
    tglightcontour_list result;

    int i;
    double length_hdg;
    double dist = rwy.length - rwy.threshold[0] - rwy.threshold[1];
    int divs = (int)(dist / 60.0) + 1;
    double step = dist / divs;
    SGGeod pt1, pt2;

    SGVec3f normal = gen_runway_light_vector( 3.0, recip );

    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180.0);
        pt1 = SGGeodesy::direct(GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)]);
        pt2 = GetStart();
    } else {
        length_hdg = rwy.heading;
        pt1 = SGGeodesy::direct(GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)]);
        pt2 = GetEnd();
    }
    double left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0 );

    int tstep;
    double offset = 2 + rwy.width * 0.5;

    //front threshold
    if (rwy.threshold[get_thresh0(recip)] > step )
    {
        SGGeod pt0 = pt1;
        tstep = (int)(rwy.threshold[get_thresh0(recip)] / step);
        for ( i = 0; i < tstep; ++i ) {
            pt0 = SGGeodesy::direct(pt0, length_hdg, -step);
            r_lights.AddLight( SGGeodesy::direct(pt0, left_hdg,  offset), normal );
            r_lights.AddLight( SGGeodesy::direct(pt0, left_hdg, -offset), normal );
        }
    }

    for ( i = 0; i < divs; ++i ) {
        pt1 = SGGeodesy::direct(pt1, SGGeodesy::courseDeg(pt1, pt2), step);
        dist -= step;
        if ( dist > 610.0 || dist > rwy.length / 2 ) {
            w_lights.AddLight( SGGeodesy::direct(pt1, left_hdg,  offset), normal );
            w_lights.AddLight( SGGeodesy::direct(pt1, left_hdg, -offset), normal );
        } else if (dist > 5.0) {
            y_lights.AddLight( SGGeodesy::direct(pt1, left_hdg,  offset), normal );
            y_lights.AddLight( SGGeodesy::direct(pt1, left_hdg, -offset), normal );
        }
    }

    //back threshold
    if (rwy.threshold[get_thresh1(recip)] > step )
    {
        tstep = (int)(rwy.threshold[get_thresh1(recip)] / step);
        for ( i = 0; i < tstep; ++i ) {
            y_lights.AddLight( SGGeodesy::direct(pt1, left_hdg,  offset), normal );
            y_lights.AddLight( SGGeodesy::direct(pt1, left_hdg, -offset), normal );
            pt1 = SGGeodesy::direct(pt1, length_hdg, step);
        }
    }

    //Different intensities
    if (rwy.edge_lights == 3) {
        w_lights.SetType( "RWY_WHITE_LIGHTS" );
        y_lights.SetType( "RWY_YELLOW_LIGHTS" );
        r_lights.SetType( "RWY_RED_LIGHTS" );
    }
    else if (rwy.edge_lights == 2) {
        w_lights.SetType( "RWY_WHITE_MEDIUM_LIGHTS" );
        y_lights.SetType( "RWY_YELLOW_MEDIUM_LIGHTS" );
        r_lights.SetType( "RWY_RED_MEDIUM_LIGHTS" );
    }
    else if (rwy.edge_lights == 1) {
        w_lights.SetType( "RWY_WHITE_LOW_LIGHTS" );
        y_lights.SetType( "RWY_YELLOW_LOW_LIGHTS" );
        r_lights.SetType( "RWY_RED_LOW_LIGHTS" );
    }

    result.push_back( w_lights );
    result.push_back( y_lights );
    result.push_back( r_lights );

    return result;
}

// generate threshold lights for displaced/normal runways and with/without light bars
tglightcontour_list Runway::gen_runway_threshold_lights( const int kind, bool recip )
{
    tgLightContour r_lights;
    tgLightContour g_lights;
    tglightcontour_list result;
    int i;

    // determine the start point.
    SGGeod ref1, ref2;
    double length_hdg;

    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref1 = SGGeodesy::direct(GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)] - 1);
        ref2 = GetEnd();
    } else {
        length_hdg = rwy.heading;
        ref1 = SGGeodesy::direct(GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)] - 1);
        ref2 = GetStart();
    }

    SGVec3f normal1 = gen_runway_light_vector( 3.0, recip );
    SGVec3f normal2 = gen_runway_light_vector( 3.0, !recip );

    double left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);
    int divs = (int)(rwy.width + 4) / 3.0;
    double step = (rwy.width + 4) / divs;
    SGGeod pt1, pt2;
    double offset = 2 + rwy.width * 0.5;

    if ( GetsThreshold(recip) ) {
        SGGeod thresh1 = pt1 = SGGeodesy::direct(ref1, left_hdg, offset);
        SGGeod thresh2 = SGGeodesy::direct(ref1, left_hdg, -offset);
        // four lights for each side
        for ( i = 0; i < 4; ++i ) {
            g_lights.AddLight( thresh1, normal1 );
            g_lights.AddLight( thresh2, normal1 );

            // offset 3m towards outside
            thresh1 = SGGeodesy::direct(thresh1, left_hdg, 3);
            thresh2 = SGGeodesy::direct(thresh2, left_hdg, -3);
        }
    } else {
        pt1 = SGGeodesy::direct(ref2, left_hdg, offset);
    }

    if ( kind ) {
        // Add a green and red threshold lights bar
        SGGeod redbar = SGGeodesy::direct(ref2, left_hdg, offset);
        for ( i = 0; i < divs + 1; ++i ) {
            g_lights.AddLight( pt1, normal1 );
            r_lights.AddLight( redbar, normal2 );

            pt1 = SGGeodesy::direct(pt1, left_hdg, -step);
            redbar = SGGeodesy::direct(redbar, left_hdg, -step);
        }
    }

    // Now create the lights at the front of the runway
    pt1 = SGGeodesy::direct(ref2, left_hdg, offset);
    pt2 = SGGeodesy::direct(ref2, left_hdg, -offset);

    // Create groups of four lights in front of the displaced threshold
    for ( i = 0; i < 4; ++i ) {

        if (GetsThreshold(recip) ) {
            r_lights.AddLight( pt1, normal1);
            r_lights.AddLight( pt2, normal1);
        } else if (!kind) {
            g_lights.AddLight( pt1, normal1);
            g_lights.AddLight( pt2, normal1);
        }

        if (!kind) {
            r_lights.AddLight( pt1, normal2 );
            r_lights.AddLight( pt2, normal2 );
        }

        // offset 3m towards center
        pt1 = SGGeodesy::direct(pt1, left_hdg, -3);
        pt2 = SGGeodesy::direct(pt2, left_hdg,  3);
    }

    g_lights.SetType( "RWY_GREEN_LIGHTS" );
    r_lights.SetType( "RWY_RED_LIGHTS" );

    result.push_back( g_lights );
    result.push_back( r_lights );

    return result;
}


// generate runway center line lighting, 15m spacing.
tglightcontour_list Runway::gen_runway_center_line_lights( bool recip )
{
    tgLightContour w_lights;
    tgLightContour r_lights;
    tglightcontour_list result;

    SGVec3f normal = gen_runway_light_vector( 3.0, recip );

    SGGeod pt1, pt2;
    double length_hdg;

    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        pt1 = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)]);
        pt2 = GetStart();
    } else {
        length_hdg = rwy.heading;
        pt1 = SGGeodesy::direct(GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)]);
        pt2 = GetEnd();
    }

    double dist = SGGeodesy::distanceM(pt1, pt2);
    int divs = (int)(dist / 15.0) + 1;
    double step = dist / divs;
    bool use_white = true;

    while ( dist > 0.0 ) {
        if ( dist > 900.0 ) {
            w_lights.AddLight( pt1, normal );
        } else if ( dist > 300.0 ) {
            if ( use_white ) {
                w_lights.AddLight( pt1, normal );
            } else {
                r_lights.AddLight( pt1, normal );
            }
            use_white = !use_white;
        } else {
            r_lights.AddLight( pt1, normal );
        }
        length_hdg = SGGeodesy::courseDeg(pt1, pt2);
        pt1 = SGGeodesy::direct(pt1, length_hdg, step);
        dist -= step;
    }

    if ( w_lights.ContourSize() ) {
        w_lights.SetType( "RWY_WHITE_MEDIUM_LIGHTS" );
        result.push_back( w_lights );
    }

    if ( r_lights.ContourSize() ) {
        r_lights.SetType( "RWY_RED_MEDIUM_LIGHTS" );
        result.push_back( r_lights );
    }

    return result;
}


// generate touch down zone lights
tgLightContour Runway::gen_touchdown_zone_lights( bool recip )
{
    tgLightContour lights;

    SGVec3f normal = gen_runway_light_vector( 3.0, recip );

    // determine the start point.
    SGGeod ref;
    double length_hdg, left_hdg;
    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)]);
    } else {
        length_hdg = rwy.heading;
        ref = SGGeodesy::direct(GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)]);
    }
    left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);

    // calculate amount of touchdown light rows.
    // They should cover a distance of 900m or
    // half the runway length, whichever comes first. Spacing is 30m.
    int rows = (int)(rwy.length * 0.5) / 30;
    if (rows > 30) rows = 30;

    for ( int i = 0; i < rows; ++i ) {
        // offset 30m upwind
        ref = SGGeodesy::direct( ref, length_hdg, 30 );
        SGGeod pt1 = ref;

        // left side bar
        pt1 = SGGeodesy::direct( pt1, left_hdg, 11 );
        lights.AddLight( pt1, normal );

        pt1 = SGGeodesy::direct( pt1, left_hdg, 1.5 );
        lights.AddLight( pt1, normal );

        pt1 = SGGeodesy::direct( pt1, left_hdg, 1.5 );
        lights.AddLight( pt1, normal );

        pt1 = ref;

        // right side bar
        pt1 = SGGeodesy::direct( pt1, left_hdg, -11 );
        lights.AddLight( pt1, normal );

        pt1 = SGGeodesy::direct( pt1, left_hdg, -1.5 );
        lights.AddLight( pt1, normal );

        pt1 = SGGeodesy::direct( pt1, left_hdg, -1.5 );
        lights.AddLight( pt1, normal );
    }

    lights.SetType( "RWY_WHITE_LIGHTS" );

    return lights;;
}


// generate REIL lights
tgLightContour Runway::gen_reil( const int kind, bool recip )
{
    tgLightContour lights;
    string flag = rwy.rwnum[get_thresh0(recip)];
    SGVec3f normal;

    if (kind == 1) {
        // omnidirectional lights
        normal = normalize(SGVec3f::fromGeod(GetStart()));
    } else {
        // unidirectional lights
        normal = gen_runway_light_vector( 10, recip );
    }

    // determine the start point.
    SGGeod ref;
    double length_hdg, left_hdg;
    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)] - 1);
    } else {
        length_hdg = rwy.heading;
        ref = SGGeodesy::direct(GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)] - 1);
    }

    left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);
    double offset = rwy.width * 0.5 + 12;

    // left light
    lights.AddLight( SGGeodesy::direct( ref, left_hdg,  offset ), normal );

    // right light
    lights.AddLight( SGGeodesy::direct( ref, left_hdg, -offset ), normal );

    lights.SetType( "RWY_REIL_LIGHTS" );
    lights.SetFlag( flag );

    return lights;
}


// generate Calvert-I/II approach lighting schemes
tglightcontour_list Runway::gen_calvert( const string &kind, bool recip )
{
    tgLightContour w_lights;
    tgLightContour r_lights;
    string flag;

    SGVec3f normal = gen_runway_light_vector( 3.0, recip );

    // Generate long center bar of lights
    // determine the start point.
    SGGeod ref_save, pt;
    double length_hdg, left_hdg;
    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref_save = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    } else {
        length_hdg = rwy.heading;
        ref_save = SGGeodesy::direct( GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    }
    left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);

    SGGeod ref = ref_save;
    //
    // Centre row of lights:
    // 1 x lights out to 300m
    // 2 x lights from 300m to 600m
    // 3 x lights from 600m to 900m
    // light spacing is 30m
    //

    const double vert_space = 30;
    const double horiz_space = 10;
    const int count=30;

    SGGeod crossbar[5];
    SGGeod pair;

    // first set of single lights
    pt = ref;
    for ( int i = 0; i < count; ++i ) {
        // centre lights
        pt = SGGeodesy::direct(pt, length_hdg, -vert_space);

        if ( i >= 10 && i < 20 ) {
            w_lights.AddLight( SGGeodesy::direct(pt, left_hdg, horiz_space/2), normal );
            w_lights.AddLight( SGGeodesy::direct(pt, left_hdg, -horiz_space/2), normal );
        } else if (i >= 20) {
            w_lights.AddLight( pt, normal);
            w_lights.AddLight( SGGeodesy::direct(pt, left_hdg, horiz_space), normal );
            w_lights.AddLight( SGGeodesy::direct(pt, left_hdg, -horiz_space), normal );
        } else if (i < 10 && kind == "1" ) {
            w_lights.AddLight( pt, normal);
        } else {
            // cal2 has red centre lights
            r_lights.AddLight( pt, normal);
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

    }

    if ( kind == "2" ) {
        // add some red and white bars in the 300m area
        // in front of the threshold
        ref = ref_save;
        for ( int i = 0; i < 9; ++i ) {
            // offset upwind
            ref = SGGeodesy::direct( ref, length_hdg, -vert_space );
            pt = ref;

            // left side bar
            pt = SGGeodesy::direct( pt, left_hdg, 1.5 );
            w_lights.AddLight( pt, normal);

            pt = SGGeodesy::direct( pt, left_hdg, 1.5 );
            w_lights.AddLight( pt, normal);

            pt = ref;
            pt = SGGeodesy::direct( pt, left_hdg, 11 );
            r_lights.AddLight( pt, normal);

            pt = SGGeodesy::direct( pt, left_hdg, 1.5 );
            r_lights.AddLight( pt, normal);

            pt = SGGeodesy::direct( pt, left_hdg, 1.5 );
            r_lights.AddLight( pt, normal);

            pt = ref;

            // right side bar
            pt = SGGeodesy::direct( pt, left_hdg, -1.5 );
            w_lights.AddLight( pt, normal);

            pt = SGGeodesy::direct( pt, left_hdg, -1.5 );
            w_lights.AddLight( pt, normal);

            pt = ref;
            pt = SGGeodesy::direct( pt, left_hdg, -11 );
            r_lights.AddLight( pt, normal);

            pt = SGGeodesy::direct( pt, left_hdg, -1.5 );
            r_lights.AddLight( pt, normal);

            pt = SGGeodesy::direct( pt, left_hdg, -1.5 );
            r_lights.AddLight( pt, normal);
        }
    }

    ref = ref_save;
    int num_lights = 0;

    // draw nice crossbars
    for ( int i = 0; i < 5; ++i ) {
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
        for ( int j = 0 ; j < num_lights; ++j ) {
            // left side lights
            pt = SGGeodesy::direct(pt, left_hdg, horiz_space);
            w_lights.AddLight(pt, normal);
        }

        pt = crossbar[i];
        for ( int j = 0; j < num_lights; ++j ) {
            // right side lights
            pt = SGGeodesy::direct(pt, left_hdg, -horiz_space);
            w_lights.AddLight(pt, normal);
        }
    }

    tglightcontour_list result;
    r_lights.SetType( "RWY_RED_LIGHTS" );
    w_lights.SetType( "RWY_WHITE_LIGHTS" );

    result.push_back( r_lights );
    result.push_back( w_lights );

    return result;
}

// generate ALSF-I/II and SALS/SALSF approach lighting schemes
tglightcontour_list Runway::gen_alsf( const string &kind, bool recip )
{
    tgLightContour g_lights;
    tgLightContour w_lights;
    tgLightContour r_lights;
    tgLightContour s_lights;
    int i;
    string flag;

    SGVec3f normal = gen_runway_light_vector( 3.0, recip );

    // Generate long center bar of lights
    // determine the start point.
    SGGeod ref_save, pt;
    double length_hdg, left_hdg;
    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref_save = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)]);
    } else {
        length_hdg = rwy.heading;
        ref_save = SGGeodesy::direct( GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)]);
    }
    left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);

    SGGeod ref = ref_save;

    int count;
    if ( kind == "1" || kind == "2" ) {
        // ALSF-I or ALSF-II
        ref = SGGeodesy::direct(ref, length_hdg, -30);
        count = 30;
    } else {
        // SALS/SALSF
        ref = SGGeodesy::direct(ref, length_hdg, -90);
        count = 13;
    }

    for ( i = 0; i < count; ++i ) {
        pt = ref;
        w_lights.AddLight(pt, normal);

        // left 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, 1);
        w_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, 1);
        w_lights.AddLight(pt, normal);

        pt = ref;

        // right 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, -1);
        w_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, -1);
        w_lights.AddLight(pt, normal);

        ref = SGGeodesy::direct(ref, length_hdg, -30);
    }

    ref = ref_save;

    if ( kind == "1" || kind == "O" || kind == "P" ) {
        // Terminating bar

        // offset 60m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -60);

        pt = ref;

        // left 3 side lights
        pt = SGGeodesy::direct(pt, left_hdg, 4.5);
        r_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, 1.5);
        r_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, 1.5);
        r_lights.AddLight(pt, normal);

        pt = ref;

        // right 3 side lights
        pt = SGGeodesy::direct(pt, left_hdg, -4.5);
        r_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, -1.5);
        r_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, -1.5);
        r_lights.AddLight(pt, normal);
    } else if ( kind == "2" ) {
        // Generate red side row lights

        for ( i = 0; i < 9; ++i ) {
            // offset 30m downwind
            ref = SGGeodesy::direct(ref, length_hdg, -30);

            pt = ref;

            // left 3 side lights
            pt = SGGeodesy::direct(pt, left_hdg, 11);
            r_lights.AddLight(pt, normal);

            pt = SGGeodesy::direct(pt, left_hdg, 1.5);
            r_lights.AddLight(pt, normal);

            pt = SGGeodesy::direct(pt, left_hdg, 1.5);
            r_lights.AddLight(pt, normal);

            pt = ref;

            // right 3 side lights
            pt = SGGeodesy::direct(pt, left_hdg, -11);
            r_lights.AddLight(pt, normal);

            pt = SGGeodesy::direct(pt, left_hdg, -1.5);
            r_lights.AddLight(pt, normal);

            pt = SGGeodesy::direct(pt, left_hdg, -1.5);
            r_lights.AddLight(pt, normal);
        }
    }

    if ( kind == "1" || kind == "O" || kind == "P" ) {
        // Generate pre-threshold bar
        ref = ref_save;

        // offset 30m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -30);
        pt = ref;

        // left 5 side lights
        pt = SGGeodesy::direct(pt, left_hdg, 22.5);
        for ( i = 0; i < 5; ++i ) {
            r_lights.AddLight(pt, normal);
            pt = SGGeodesy::direct(pt, left_hdg, 1.0);
        }
        pt = ref;

        // right 5 side lights
        pt = SGGeodesy::direct(pt, left_hdg, -22.5);
        for ( i = 0; i < 5; ++i ) {
            r_lights.AddLight(pt, normal);
            pt = SGGeodesy::direct(pt, left_hdg, -1.0);
        }
    } else if ( kind == "2" ) {
        // Generate -150m extra horizontal row of lights
        ref = ref_save;

        // offset 150m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -150);
        pt = ref;

        // left 4 side lights
        pt = SGGeodesy::direct(pt, left_hdg, 4.25);

        for ( i = 0; i < 4; ++i ) {
            w_lights.AddLight(pt, normal);
            pt = SGGeodesy::direct(pt, left_hdg, 1.5);
        }
        pt = ref;

        // right 4 side lights
        pt = SGGeodesy::direct(pt, left_hdg, -4.25);

        for ( i = 0; i < 4; ++i ) {
            w_lights.AddLight(pt, normal);
            pt = SGGeodesy::direct(pt, left_hdg, -1.5);
        }
    }

    ref = ref_save;

    if ( kind == "O" || kind == "P" ) {
        // generate SALS secondary threshold
        ref = SGGeodesy::direct(ref, length_hdg, -60);

        pt = ref;
        r_lights.AddLight(pt, normal);

        // left 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, 1.0);
        r_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, 1.0);
        r_lights.AddLight(pt, normal);

        pt = ref;

        // right 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, -1.0);
        r_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, -1.0);
        r_lights.AddLight(pt, normal);
    }

    // Generate -300m horizontal crossbar
    ref = ref_save;

    // offset 300m downwind
    ref = SGGeodesy::direct(ref, length_hdg, -300);
    pt = ref;

    // left 8 side lights
    pt = SGGeodesy::direct(pt, left_hdg, 4.5);
    for ( i = 0; i < 8; ++i ) {
        w_lights.AddLight(pt, normal);
        pt = SGGeodesy::direct(pt, left_hdg, 1.5);
    }

    pt = ref;

    // right 8 side lights
    pt = SGGeodesy::direct(pt, left_hdg, -4.5);
    for ( i = 0; i < 8; ++i ) {
        w_lights.AddLight(pt, normal);
        pt = SGGeodesy::direct(pt, left_hdg, -1.5);
    }

    ref = ref_save;

    if ( kind == "1" || kind == "2" ) {
        // generate rabbit lights
        // start 300m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -300);

        for ( i = 0; i < 21; ++i ) {
            s_lights.AddLight(ref, normal);

            // offset 30m downwind
            ref = SGGeodesy::direct(ref, length_hdg, -30);
        }
    } else if ( kind == "P" ) {
        // generate 3 sequenced lights aligned with last 3 light bars
        // start 390m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -390);

        for ( i = 0; i < 3; ++i ) {
            s_lights.AddLight(ref, normal);

            // offset 30m downwind
            ref = SGGeodesy::direct(ref, length_hdg, -30);
        }
    }

    tglightcontour_list result;

    g_lights.SetType( "RWY_GREEN_LIGHTS" );
    w_lights.SetType( "RWY_WHITE_LIGHTS" );
    r_lights.SetType( "RWY_RED_LIGHTS" );

    result.push_back( g_lights );
    result.push_back( w_lights );
    result.push_back( r_lights );

    if ( s_lights.ContourSize() ) {
        s_lights.SetType( "RWY_SEQUENCED_LIGHTS");
        result.push_back( s_lights );
    }

    return result;
}


// generate ODALS or RAIL lights. Main differende between the two:
// ODALS is omnidirectional, RAIL is unidirectional
tgLightContour Runway::gen_odals( const int kind, bool recip )
{
    tgLightContour lights;

    int i;
    string material;
    SGVec3f normal;

    if (kind == 0) {
        // ODALS lighting is omni-directional, but we generate a normal as
        // a placeholder to keep everything happy.
        normal = normalize(SGVec3f::fromGeod(GetStart()));
        material = "RWY_ODALS_LIGHTS";
    } else {
        // RAIL lights are directional
        normal = gen_runway_light_vector( 3.0, recip );
        material = "RWY_SEQUENCED_LIGHTS";
    }

    // determine the start point.
    SGGeod ref;
    double length_hdg, left_hdg;

    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    } else {
        length_hdg = rwy.heading;
        ref = SGGeodesy::direct( GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    }
    left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);
    double offset = rwy.width / 2 + 14;

    if (kind == 0) {
        // offset 14m left of runway
        lights.AddLight( SGGeodesy::direct(ref, left_hdg, offset), normal);

        // offset 14m right of runway
        lights.AddLight( SGGeodesy::direct(ref, left_hdg, -offset), normal);
    }

    for ( i = 0; i < 5; ++i ) {
        // offset 90m downwind
        ref = SGGeodesy::direct( ref, length_hdg, -90 );
        lights.AddLight(ref, normal);
    }

    lights.SetType( material );

    return lights;
}


// generate SSALS, SSALF, and SSALR approach lighting scheme (kind =
// S, F, or R)
tglightcontour_list Runway::gen_ssalx( const string& kind, bool recip )
{
    tgLightContour g_lights;
    tgLightContour w_lights;
    tgLightContour r_lights;
    tgLightContour s_lights;
    int i;
    string flag;

    SGVec3f normal = gen_runway_light_vector( 3.0, recip );
    SGGeod ref_save, pt;

    // Generate long center bar of lights (every 200')
    // determine the start point.
    double length_hdg, left_hdg;
    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref_save = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    } else {
        length_hdg = rwy.heading;
        ref_save = SGGeodesy::direct( GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    }
    left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);

    SGGeod ref = ref_save;

    for ( i = 0; i < 7; ++i ) {
        // offset 60m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -60);

        pt = ref;
        w_lights.AddLight(pt, normal);

        // left 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, 1.0);
        w_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, 1.0);
        w_lights.AddLight(pt, normal);

        pt = ref;

        // right 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, -1.0);
        w_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, -1.0);
        w_lights.AddLight(pt, normal);
    }

    // Generate -300m extra horizontal row of lights
    ref = SGGeodesy::direct(ref_save, length_hdg, -300);
    pt = ref;

    // left 5 side lights
    pt = SGGeodesy::direct(pt, left_hdg, 4.5);
    for ( i = 0; i < 5; ++i ) {
        w_lights.AddLight(pt, normal);
        pt = SGGeodesy::direct(pt, left_hdg, 1.5);
    }

    pt = ref;
    // right 5 side lights
    pt = SGGeodesy::direct(pt, left_hdg, -4.5);
    for ( i = 0; i < 5; ++i ) {
        w_lights.AddLight(pt, normal);
        pt = SGGeodesy::direct(pt, left_hdg, -1.5);
    }

    if ( kind == "R" ) {
        // generate 8 rabbit lights
        ref = ref_save;

        // start 480m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -480);

        for ( i = 0; i < 8; ++i ) {
            s_lights.AddLight(ref, normal);

            // offset 60m downwind
            ref = SGGeodesy::direct(ref, length_hdg, -60);
        }
    } else if ( kind == "F" ) {
        // generate 3 sequenced lights aligned with last 3 light bars
        ref = ref_save;

        // start 300m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -300);

        for ( i = 0; i < 3; ++i ) {
            s_lights.AddLight(ref, normal);

            // offset 60m downwind
            ref = SGGeodesy::direct(ref, length_hdg, -60);
        }
    }

    tglightcontour_list result;
    g_lights.SetType( "RWY_GREEN_LIGHTS" );
    w_lights.SetType( "RWY_WHITE_LIGHTS" );
    r_lights.SetType( "RWY_RED_LIGHTS" );

    result.push_back( g_lights );
    result.push_back( w_lights );
    result.push_back( r_lights );

    if ( s_lights.ContourSize() ) {
        s_lights.SetType( "RWY_SEQUENCED_LIGHTS" );
        result.push_back( s_lights );
    }

    return result;
}


// generate MALS, MALSF, and MALSR approach lighting scheme (kind =
// ' ', F, or R)
tglightcontour_list Runway::gen_malsx( const string& kind, bool recip )
{
    tgLightContour g_lights;
    tgLightContour w_lights;
    tgLightContour r_lights;
    tgLightContour s_lights;
    int i;
    string flag;

    SGVec3f normal = gen_runway_light_vector( 3.0, recip );

    // Generate long center bar of lights (every 60m)
    // determine the start point.
    SGGeod ref_save, pt;
    double length_hdg, left_hdg;
    if ( recip ) {
        length_hdg = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        ref_save = SGGeodesy::direct( GetEnd(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    } else {
        length_hdg = rwy.heading;
        ref_save = SGGeodesy::direct( GetStart(), length_hdg, rwy.threshold[get_thresh0(recip)] );
    }
    left_hdg = SGMiscd::normalizePeriodic(0, 360, length_hdg - 90.0);
    SGGeod ref = ref_save;

    for ( i = 0; i < 7; ++i ) {
        // offset 60m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -60);

        pt = ref;
        w_lights.AddLight(pt, normal);

        // left 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, 1.0);
        w_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, 1.0);
        w_lights.AddLight(pt, normal);

        pt = ref;

        // right 2 side lights
        pt = SGGeodesy::direct(pt, left_hdg, -1.0);
        w_lights.AddLight(pt, normal);

        pt = SGGeodesy::direct(pt, left_hdg, -1.0);
        w_lights.AddLight(pt, normal);
    }

    // Generate -300m extra horizontal row of lights
    ref = SGGeodesy::direct(ref_save, length_hdg, -300);
    pt = ref;

    // left 5 side lights
    pt = SGGeodesy::direct(pt, left_hdg, 6.5);
    for ( i = 0; i < 5; ++i ) {
        w_lights.AddLight(pt, normal);
        pt = SGGeodesy::direct(pt, left_hdg, 0.75);
    }

    pt = ref;
    // right 5 side lights
    pt = SGGeodesy::direct(pt, left_hdg, -6.5);
    for ( i = 0; i < 5; ++i ) {
        w_lights.AddLight(pt, normal);
        pt = SGGeodesy::direct(pt, left_hdg, -0.75);
    }

    if ( kind == "R" ) {
        // generate 5 rabbit lights
        ref = ref_save;

        // start 480m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -480);
        for ( i = 0; i < 5; ++i ) {
            s_lights.AddLight(ref, normal);

            // offset 60m downwind
            ref = SGGeodesy::direct(ref, length_hdg, -60);
        }
    } else if ( kind == "F" ) {
        // generate 3 sequenced lights aligned with last 3 light bars
        ref = ref_save;

        // start 300m downwind
        ref = SGGeodesy::direct(ref, length_hdg, -300);
        for ( i = 0; i < 3; ++i ) {
            s_lights.AddLight(ref, normal);

            // offset 60m downwind
            ref = SGGeodesy::direct(ref, length_hdg, -60);
        }
    }

    tglightcontour_list result;
    g_lights.SetType( "RWY_GREEN_LIGHTS" );
    w_lights.SetType( "RWY_WHITE_LIGHTS" );
    r_lights.SetType( "RWY_RED_LIGHTS" );

    result.push_back( g_lights );
    result.push_back( w_lights );
    result.push_back( r_lights );

    if ( s_lights.ContourSize() ) {
        s_lights.SetType( "RWY_SEQUENCED_LIGHTS" );
        result.push_back( s_lights );
    }

    return result;
}


// top level runway light generator
void Runway::gen_runway_lights( tglightcontour_list& lights ) {

    unsigned int i, side;
    bool recip;

    for (side = 0; side < 2; ++side) {
        if (side == 0) {
            recip = false;
        } else {
            recip = true;
        }

        // Make edge lighting
        if ( rwy.edge_lights ) {
            tglightcontour_list s = gen_runway_edge_lights( recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        // Centerline lighting
        if ( rwy.centerline_lights ) {
            tglightcontour_list s = gen_runway_center_line_lights( recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        // Touchdown zone lighting
        if ( rwy.tz_lights[side] ) {
            tgLightContour s = gen_touchdown_zone_lights( recip );
            lights.push_back( s );
        }

        // REIL lighting
        if ( rwy.reil[side] ) {
            tgLightContour s = gen_reil( rwy.reil[side], recip );
            lights.push_back( s );
        }

        // Approach lighting
        if ( rwy.approach_lights[side] == 1 /* ALSF-I */ ) {
            tglightcontour_list s = gen_alsf( "1", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 2 /* ALSF-II */ ) {
            tglightcontour_list s = gen_alsf( "2", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 3  /* Calvert I */ ) {
            tglightcontour_list s = gen_calvert( "1", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 4  /* Calvert II */ ) {
            tglightcontour_list s = gen_calvert( "2", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 5 /* SSALR */ ) {
            tglightcontour_list s = gen_ssalx( "R", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 6 /* SSALF */ ) {
            tglightcontour_list s = gen_ssalx( "F", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        // SALS (Essentially ALSF-1 without the lead in rabbit lights, and
        // a shorter center bar)
        else if ( rwy.approach_lights[side] == 7 /* SALS */ ) {
            tglightcontour_list s = gen_alsf( "O", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 8 /* MALSR */ ) {
            tglightcontour_list s = gen_malsx( "R", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 9 /* MALSF */ ) {
            tglightcontour_list s = gen_malsx( "F", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 10 /* MALSX */ ) {
            tglightcontour_list s = gen_malsx( "x", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == 11 /* ODALS Omni-directional approach light system */ ) {
            tgLightContour s = gen_odals( 0, recip );
            lights.push_back( s );
        }

        // RAIL: Sequenced strobes with no other approach lights
        else if ( rwy.approach_lights[side] == 12 /* RAIL Runway alignment indicator lights */ ) {
            tgLightContour s = gen_odals( 1, recip );
            lights.push_back( s );
        }

#if 0
        else if ( rwy.approach_lights[side] == -1 /* SALSF not supported by database */ ) {
            superpoly_list s = gen_alsf( "P", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights->push_back( s[i] );
            }
        }

        else if ( rwy.approach_lights[side] == -1 /* SSALS not supported by database */ ) {
            superpoly_list s = gen_ssalx( "S", recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights->push_back( s[i] );
            }
        }
#endif

        // Approach light systems that have a threshold light bar
        // use a central routine for its creation
        if ( rwy.approach_lights[side] > 0 && rwy.approach_lights[side] < 11)
        {
            tglightcontour_list s = gen_runway_threshold_lights( 1, recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }

        // If the runway has edge lights but no approach lights,
        // or approach lights without a lights bar,
        // create a simple threshold lighting
        if ( rwy.edge_lights && (rwy.approach_lights[side] == 0 || rwy.approach_lights[side] > 10))
        {
            tglightcontour_list s = gen_runway_threshold_lights( 0, recip );
            for ( i = 0; i < s.size(); ++i ) {
                lights.push_back( s[i] );
            }
        }
    }
}
