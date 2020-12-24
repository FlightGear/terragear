// rwy_gen.cxx -- Build a runway
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

#include <stdlib.h>

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include <boost/lexical_cast.hpp>

#include <terragear/tg_shapefile.hxx>

#include "global.hxx"
#include "beznode.hxx"
#include "runway.hxx"

using std::string;
struct sections
{
    const char* tex;
    double size;
};

// UK Precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections uk_prec[] = {
    { "tz_one_a", 380 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "aim_uk", 400 * SG_FEET_TO_METER },
    { "tz_one_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_b", 200 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_b", 200 * SG_FEET_TO_METER }
};

// UK non-precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections uk_nprec[] = {
    { "centerline", 200 * SG_FEET_TO_METER },
    { "aim_uk", 400 * SG_FEET_TO_METER },
};

// Precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections prec[] = {
    { "tz_three", 380 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "aim", 400 * SG_FEET_TO_METER },
    { "tz_two_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_two_b", 200 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_b", 200 * SG_FEET_TO_METER }
};

// Non-precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections nprec[] = {
    { "centerline", 200 * SG_FEET_TO_METER },
    { "aim", 400 * SG_FEET_TO_METER }
};


tgPolygon Runway::gen_shoulder_section( SGGeod& p0, SGGeod& p1, SGGeod& t0, SGGeod& t1, int side, double heading, double width, std::string surface )
{
    SGGeod    s0, s1, s2, s3;
    tgPolygon poly;

    double wid_hdg = 0.0f;
    double az2     = 0.0f;
    double dist    = 0.0f;

    // calc heading and distance from p0 to p1
    SGGeodesy::inverse( p0, p1, wid_hdg, az2, dist );

    // s0 is width away from t1 in wid_hdg direction
    s0 = SGGeodesy::direct( t1, wid_hdg, width );

    // s1 is width away from t0 in wid_hdg direction
    s1 = SGGeodesy::direct( t0, wid_hdg, width );

    // s2 is nudge away from t0 in -wid_hdg direction
    s2 = SGGeodesy::direct( t0, wid_hdg, -0.01 );

    // s3 is nudge away from t1 in -wid_hdg direction
    s3 = SGGeodesy::direct( t1, wid_hdg, -0.01 );

    // Generate a poly
    poly.AddNode( 0, s0 );
    poly.AddNode( 0, s1 );
    poly.AddNode( 0, s2 );
    poly.AddNode( 0, s3 );
    poly = tgPolygon::Snap( poly, gSnap );

    poly.SetMaterial( surface );
    if (side == 0) {
        poly.SetTexParams( poly.GetNode(0,2), width, dist, heading );
        poly.SetTexLimits( 0,0,1,1 );
    } else {
        poly.SetTexParams( poly.GetNode(0,1), width, dist, heading );
        poly.SetTexLimits( 1,0,0,1 );
    }
    poly.SetTexMethod( TG_TEX_BY_TPS_CLIPU, 0.0, 0.0, 1.0, 0.0 );

    return poly;
}

// generate a section of texture with shoulders
void Runway::gen_runway_section( const tgPolygon& runway,
                                 double startl_pct, double endl_pct,
                                 double startw_pct, double endw_pct,
                                 double minu, double maxu, double minv, double maxv,
                                 double heading,
                                 const string& material,
                                 tgpolygon_list& rwy_polys,
                                 tgpolygon_list& shoulder_polys,
                                 tgcontour_list& slivers,
                                 tgAccumulator& accum,
                                 std::string& shapefile_name )
{
    double width = rwy.width;
    double length = rwy.length;
    double lshoulder_width = 0.0f;
    double rshoulder_width = 0.0f;
    std::string shoulder_surface = "";

#if 0
    static int runway_idx  = 0;
    static int section_idx = 0;
    static int clipped_idx = 0;
    char layer[64];
#endif
    SGVec2d a0 = SGVec2d( runway.GetNode(0, 1).getLongitudeDeg(), runway.GetNode(0, 1).getLatitudeDeg() );
    SGVec2d a1 = SGVec2d( runway.GetNode(0, 2).getLongitudeDeg(), runway.GetNode(0, 2).getLatitudeDeg() );
    SGVec2d a2 = SGVec2d( runway.GetNode(0, 0).getLongitudeDeg(), runway.GetNode(0, 0).getLatitudeDeg() );
    SGVec2d a3 = SGVec2d( runway.GetNode(0, 3).getLongitudeDeg(), runway.GetNode(0, 3).getLatitudeDeg() );

    if ( startl_pct > 0.0 ) {
        startl_pct -= nudge * SG_EPSILON;
    }
    if ( endl_pct < 1.0 ) {
        endl_pct += nudge * SG_EPSILON;
    }
    if ( endl_pct > 1.0 ) {
        endl_pct = 1.0;
    }

    // calculate if we are going to be creating shoulder polys
    if ( (rwy.shoulder > 0) && (rwy.surface < 3) ){
        if (rwy.shoulder == 1){
            shoulder_surface = "pa_shoulder";
        } else if (rwy.shoulder == 2){
            shoulder_surface = "pc_shoulder";
        }

        if ( startw_pct == 0.0f ) {
            lshoulder_width = width/4.0;
        }
        if ( endw_pct == 1.0f ) {
            rshoulder_width = width/4.0;
        }
    } else {  
        // We add a fake shoulder if the runway has an asphalt or concrete surface
        if ( (rwy.surface == 1) || (rwy.surface == 2) ) {
            if (rwy.surface == 1) {
                shoulder_surface = "pa_shoulder_f";
            } else if (rwy.surface == 2){
                shoulder_surface = "pc_shoulder_f";
            }

            if ( startw_pct == 0.0f ) {
                lshoulder_width = 1.0;
            }
            if ( endw_pct == 1.0f ) {
                rshoulder_width = 1.0;
            }
        }
    }

    // partial "w" percentages could introduce "T" intersections which
    // we compensate for later, but could still cause problems now
    // with our polygon clipping code.  This attempts to compensate
    // for that by nudging the areas a bit bigger so we don't end up
    // with polygon slivers.
    if ( startw_pct > 0.0 || endw_pct < 1.0 ) {
        if ( startw_pct > 0.0 ) {
            startw_pct -= nudge * SG_EPSILON;
        }
        if ( endw_pct < 1.0 ) {
            endw_pct += nudge * SG_EPSILON;
        }
    }

    TG_LOG(SG_GENERAL, SG_DEBUG, "start len % = " << startl_pct << " end len % = " << endl_pct);

    double dlx, dly;

    dlx = a1.x() - a0.x();
    dly = a1.y() - a0.y();

    SGVec2d t0 = SGVec2d( a0.x() + dlx * startl_pct,
                          a0.y() + dly * startl_pct );
    SGVec2d t1 = SGVec2d( a0.x() + dlx * endl_pct,
                          a0.y() + dly * endl_pct );

    dlx = a3.x() - a2.x();
    dly = a3.y() - a2.y();

    SGVec2d t2 = SGVec2d( a2.x() + dlx * startl_pct,
                          a2.y() + dly * startl_pct );

    SGVec2d t3 = SGVec2d( a2.x() + dlx * endl_pct,
                          a2.y() + dly * endl_pct );

    TG_LOG(SG_GENERAL, SG_DEBUG, "start wid % = " << startw_pct << " end wid % = " << endw_pct);

    double dwx, dwy;

    dwx = t0.x() - t2.x();
    dwy = t0.y() - t2.y();

    SGVec2d p0 = SGVec2d( t2.x() + dwx * startw_pct,
                          t2.y() + dwy * startw_pct );

    SGVec2d p1 = SGVec2d( t2.x() + dwx * endw_pct,
                          t2.y() + dwy * endw_pct );

    dwx = t1.x() - t3.x();
    dwy = t1.y() - t3.y();

    SGVec2d p2 = SGVec2d( t3.x() + dwx * startw_pct,
                          t3.y() + dwy * startw_pct );

    SGVec2d p3 = SGVec2d( t3.x() + dwx * endw_pct,
                          t3.y() + dwy * endw_pct );


    // convert vectors back to GEOD
    SGGeod pg0 = SGGeod::fromDeg( p0.x(), p0.y() );
    SGGeod pg1 = SGGeod::fromDeg( p1.x(), p1.y() );
    SGGeod pg2 = SGGeod::fromDeg( p2.x(), p2.y() );
    SGGeod pg3 = SGGeod::fromDeg( p3.x(), p3.y() );
    SGGeod tg0 = SGGeod::fromDeg( t0.x(), t0.y() );
    SGGeod tg1 = SGGeod::fromDeg( t1.x(), t1.y() );
    SGGeod tg2 = SGGeod::fromDeg( t2.x(), t2.y() );
    SGGeod tg3 = SGGeod::fromDeg( t3.x(), t3.y() );

    // check for left shoulder
    if ( lshoulder_width > 0.0f ) {
        tgPolygon sp;

        sp = gen_shoulder_section( pg0, pg1, tg0, tg1, 0, heading, lshoulder_width, shoulder_surface );
        shoulder_polys.push_back( sp );
    }

    // check for right shoulder
    if ( rshoulder_width > 0.0f ) {
        tgPolygon sp;

        sp = gen_shoulder_section( pg1, pg0, tg2, tg3, 1, heading, rshoulder_width, shoulder_surface );
        shoulder_polys.push_back( sp );
    }

    tgPolygon section;

    section.AddNode( 0, pg2 );
    section.AddNode( 0, pg0 );
    section.AddNode( 0, pg1 );
    section.AddNode( 0, pg3 );
    section = tgPolygon::Snap( section, gSnap );

    // print runway points
    TG_LOG(SG_GENERAL, SG_DEBUG, "pre clipped runway pts " << material_prefix << material);
    TG_LOG(SG_GENERAL, SG_DEBUG, section );

    if(  shapefile_name.size() ) {
        tgShapefile::FromPolygon( section, "./airport_dbg", std::string("preclip"), shapefile_name );
    }

    // Clip the new polygon against what ever has already been created.
    tgPolygon clipped = accum.Diff( section );

    if(  shapefile_name.size() ) {
        tgShapefile::FromPolygon( clipped, "./airport_dbg", std::string("postclip"), shapefile_name );
    }

    //tgPolygon::RemoveSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );

    accum.Add( section );

    // Store away what we need to know for texture coordinate
    // calculation.  (CLO 10/20/02: why can't we calculate texture
    // coordinates here?  Oh, becuase later we need to massage the
    // polygons to avoid "T" intersections and clean up other
    // potential artifacts and we may add or remove points and need to
    // do new texture coordinate calcs later.

    double len = length / 2.0;
    double sect_len = len * ( endl_pct - startl_pct );
    double sect_wid = width * ( endw_pct - startw_pct );

    split.SetMaterial( material_prefix + material );
    split.SetTexParams( pg0, sect_wid, sect_len, heading );
    split.SetTexLimits( minu, minv, maxu, maxv );
    split.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, 0.0, 0.0, 1.0, 1.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    rwy_polys.push_back( split );
}

// generate a section of texture without shoulders
void Runway::gen_runway_section( const tgPolygon& runway,
                                 double startl_pct, double endl_pct,
                                 double startw_pct, double endw_pct,
                                 double minu, double maxu, double minv, double maxv,
                                 double heading,
                                 const string& material,
                                 tgpolygon_list& rwy_polys,
                                 tgcontour_list& slivers,
                                 tgAccumulator& accum,
                                 std::string& shapefile_name )
{
    double width = rwy.width;
    double length = rwy.length;

    SGVec2d a0 = SGVec2d( runway.GetNode(0, 1).getLongitudeDeg(), runway.GetNode(0, 1).getLatitudeDeg() );
    SGVec2d a1 = SGVec2d( runway.GetNode(0, 2).getLongitudeDeg(), runway.GetNode(0, 2).getLatitudeDeg() );
    SGVec2d a2 = SGVec2d( runway.GetNode(0, 0).getLongitudeDeg(), runway.GetNode(0, 0).getLatitudeDeg() );
    SGVec2d a3 = SGVec2d( runway.GetNode(0, 3).getLongitudeDeg(), runway.GetNode(0, 3).getLatitudeDeg() );

    if ( startl_pct > 0.0 ) {
        startl_pct -= nudge * SG_EPSILON;
    }
    if ( endl_pct < 1.0 ) {
        endl_pct += nudge * SG_EPSILON;
    }
    if ( endl_pct > 1.0 ) {
        endl_pct = 1.0;
    }

    // partial "w" percentages could introduce "T" intersections which
    // we compensate for later, but could still cause problems now
    // with our polygon clipping code.  This attempts to compensate
    // for that by nudging the areas a bit bigger so we don't end up
    // with polygon slivers.
    if ( startw_pct > 0.0 || endw_pct < 1.0 ) {
        if ( startw_pct > 0.0 ) {
            startw_pct -= nudge * SG_EPSILON;
        }
        if ( endw_pct < 1.0 ) {
            endw_pct += nudge * SG_EPSILON;
        }
    }

    TG_LOG(SG_GENERAL, SG_DEBUG, "start len % = " << startl_pct << " end len % = " << endl_pct);

    double dlx, dly;

    dlx = a1.x() - a0.x();
    dly = a1.y() - a0.y();

    SGVec2d t0 = SGVec2d( a0.x() + dlx * startl_pct,
                          a0.y() + dly * startl_pct );
    SGVec2d t1 = SGVec2d( a0.x() + dlx * endl_pct,
                          a0.y() + dly * endl_pct );

    dlx = a3.x() - a2.x();
    dly = a3.y() - a2.y();

    SGVec2d t2 = SGVec2d( a2.x() + dlx * startl_pct,
                          a2.y() + dly * startl_pct );

    SGVec2d t3 = SGVec2d( a2.x() + dlx * endl_pct,
                          a2.y() + dly * endl_pct );

    TG_LOG(SG_GENERAL, SG_DEBUG, "start wid % = " << startw_pct << " end wid % = " << endw_pct);

    double dwx, dwy;

    dwx = t0.x() - t2.x();
    dwy = t0.y() - t2.y();

    SGVec2d p0 = SGVec2d( t2.x() + dwx * startw_pct,
                          t2.y() + dwy * startw_pct );

    SGVec2d p1 = SGVec2d( t2.x() + dwx * endw_pct,
                          t2.y() + dwy * endw_pct );

    dwx = t1.x() - t3.x();
    dwy = t1.y() - t3.y();

    SGVec2d p2 = SGVec2d( t3.x() + dwx * startw_pct,
                          t3.y() + dwy * startw_pct );

    SGVec2d p3 = SGVec2d( t3.x() + dwx * endw_pct,
                          t3.y() + dwy * endw_pct );


    // convert vectors back to GEOD
    SGGeod pg0 = SGGeod::fromDeg( p0.x(), p0.y() );
    SGGeod pg1 = SGGeod::fromDeg( p1.x(), p1.y() );
    SGGeod pg2 = SGGeod::fromDeg( p2.x(), p2.y() );
    SGGeod pg3 = SGGeod::fromDeg( p3.x(), p3.y() );

    tgPolygon section;

    section.AddNode( 0, pg2 );
    section.AddNode( 0, pg0 );
    section.AddNode( 0, pg1 );
    section.AddNode( 0, pg3 );
    section = tgPolygon::Snap( section, gSnap );

    // print runway points
    TG_LOG(SG_GENERAL, SG_DEBUG, "pre clipped runway pts " << material_prefix << material);
    TG_LOG(SG_GENERAL, SG_DEBUG, section );

    // Clip the new polygon against what ever has already been created.
    tgPolygon clipped = accum.Diff( section );

    //tgPolygon::RemoveSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );

    accum.Add( section );

    // Store away what we need to know for texture coordinate
    // calculation.  (CLO 10/20/02: why can't we calculate texture
    // coordinates here?  Oh, becuase later we need to massage the
    // polygons to avoid "T" intersections and clean up other
    // potential artifacts and we may add or remove points and need to
    // do new texture coordinate calcs later.

    double len = length / 2.0;
    double sect_len = len * ( endl_pct - startl_pct );
    double sect_wid = width * ( endw_pct - startw_pct );

    split.SetMaterial( material_prefix + material );
    split.SetTexParams( pg0, sect_wid, sect_len, heading );
    split.SetTexLimits( minu, minv, maxu, maxv );
    split.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, 0.0, 0.0, 1.0, 1.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    rwy_polys.push_back( split );
}

void Runway::gen_rw_designation( tgPolygon poly, double heading, string rwname,
                                 double &start_pct, double &end_pct,
                                 tgpolygon_list& rwy_polys,
                                 tgcontour_list& slivers,
                                 tgAccumulator& accum,
                                 std::string& shapefile_name )
{
    if (rwname != "XX") { /* Do not create a designation block if the runway name is set to none */
        string letter = "";
        double length = rwy.length / 2.0;
        for ( unsigned int i = 0; i < rwname.length(); ++i ) {
            string tmp = rwname.substr(i, 1);
            if ( tmp == "L" || tmp == "R" || tmp == "C" ) {
                rwname = rwname.substr(0, i);
                letter = tmp;
            }
        }
        TG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation letter = " << letter);

        // create runway designation letter
        if ( !letter.empty() ) {
            start_pct = end_pct;
            end_pct = start_pct + ( 90.0 * SG_FEET_TO_METER / length );
            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                letter,
                                rwy_polys,
                                shoulder_polys,
                                slivers,
                                accum,
                                shapefile_name );
        }


        // create runway designation number(s)
        if (rwname == "0") {
            rwname = "36";
        }

        TG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation = " << rwname);

        char tex1[32]; tex1[0] = '\0';
        char tex2[32]; tex2[0] = '\0';

        start_pct = end_pct;
        end_pct = start_pct + ( 80.0 * SG_FEET_TO_METER / length );

        if (rwname.length() == 2) {
            sprintf( tex1, "%c%c", rwname[0], 'l');
            sprintf( tex2, "%c%c", rwname[1], 'r');

            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 0.5,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex1,
                                rwy_polys, 
                                shoulder_polys,
                                slivers,
                                accum,
                                shapefile_name );
            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.5, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex2,
                                rwy_polys,
                                shoulder_polys,
                                slivers,
                                accum,
                                shapefile_name );

        } else if (rwname.length() == 1) {
            sprintf( tex1, "%c%c", rwname[0], 'c');

            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex1,
                                rwy_polys,
                                shoulder_polys,
                                slivers,
                                accum,
                                shapefile_name );
        }
    }
}

// generate a runway.  The routine modifies
// rwy_polys, texparams, and accum.  For specific details and
// dimensions of precision runway markings, please refer to FAA
// document AC 150/5340-1H
void Runway::gen_rwy( tgpolygon_list& rwy_polys,
                      tgcontour_list& slivers,
                      tgAccumulator& accum,
                      std::string& shapefile_name )
{
    TG_LOG( SG_GENERAL, SG_DEBUG, "Building runway = " << rwy.rwnum[0] << " / " << rwy.rwnum[1]);
    std::string section_name = "";
    bool debug = shapefile_name.size() != 0;

    //
    // Generate the basic runway outlines
    //
    tgContour runway = gen_runway_w_mid( 0, 0 );
    tgPolygon runway_half;

    for ( int rwhalf = 0; rwhalf < 2; ++rwhalf ) {

        double start1_pct = 0.0;
        double end1_pct = 0.0;
        double heading = 0.0;
        double length = rwy.length / 2.0;

        if (rwhalf == 0) {
            heading = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);

            //Create the first half of the runway (first entry in apt.dat)
            runway_half.Erase();
            runway_half.AddNode( 0, runway.GetNode(3) );
            runway_half.AddNode( 0, runway.GetNode(4) );
            runway_half.AddNode( 0, runway.GetNode(5) );
            runway_half.AddNode( 0, runway.GetNode(2) );
        }
    
        else {
            heading = rwy.heading;

            //Create the second runway half from apt.dat
            runway_half.Erase();
            runway_half.AddNode( 0, runway.GetNode(0) );
            runway_half.AddNode( 0, runway.GetNode(1) );
            runway_half.AddNode( 0, runway.GetNode(2) );
            runway_half.AddNode( 0, runway.GetNode(5) );
        }

        // Make sure our runway is long enough for the desired marking variant
        if ( (rwy.marking[rwhalf]==2 || rwy.marking[rwhalf]==4) && length < 1150 * SG_FEET_TO_METER ) {
            TG_LOG( SG_GENERAL, SG_ALERT,
                "Runway " << rwy.rwnum[rwhalf] << " is not long enough ("
                << rwy.length << "m) for non-precision markings!  Setting runway markings to visual!");
            rwy.marking[rwhalf]=1;
        }
        if ( (rwy.marking[rwhalf]==3 || rwy.marking[rwhalf]==5) && length < 3075 * SG_FEET_TO_METER ) {
            TG_LOG( SG_GENERAL, SG_ALERT,
                "Runway " << rwy.rwnum[rwhalf] << " is not long enough ("
                << rwy.length << "m) for precision markings!  Setting runway markings to visual!");
            rwy.marking[rwhalf]=1;
        }
        TG_LOG( SG_GENERAL, SG_DEBUG, "runway marking = " << rwy.marking[rwhalf] );


        //
        // Displaced threshold if it exists
        //
        if ( rwy.threshold[rwhalf] > 0.0 ) {
            TG_LOG( SG_GENERAL, SG_DEBUG, "Displaced threshold for RW side " << rwhalf << " is " << rwy.threshold[rwhalf] );

            double final_arrow = rwy.threshold[rwhalf];

            // If a runway has a displaced threshold we have to make sure
            // it is long enough to build all the features
            if (rwy.threshold[rwhalf] > 24.0)
            {
                // reserve 24m for final arrows
                final_arrow = 24.0;
                double thresh = rwy.threshold[rwhalf] - final_arrow;

                // number of full center arrows, each 60m long
                int count = (int)( thresh / 60.0 );

                // length of starting partial arrow
                double part_len = thresh - ( count * 60.0);
                double tex_pct = (60.0 - part_len) / 60.0;

                // starting (possibly partial chunk)
                start1_pct = end1_pct;
                end1_pct = start1_pct + ( part_len / length );
                if ( debug ) { section_name = shapefile_name + "_disp_start"; }
                gen_runway_section( runway_half,
                                    start1_pct, end1_pct,
                                    0.0, 1.0,
                                    0.0, 1.0, tex_pct, 1.0,
                                    heading,
                                    "dspl_thresh",
                                    rwy_polys,
                                    shoulder_polys,
                                    slivers,
                                    accum,
                                    section_name );

                // main chunks
                for ( int i = 0; i < count; ++i ) {
                    start1_pct = end1_pct;
                    end1_pct = start1_pct + ( 60.0 / length );
                    if ( debug ) { section_name = shapefile_name + "_disp_" + boost::lexical_cast<std::string>(i); }
                    gen_runway_section( runway_half,
                                        start1_pct, end1_pct,
                                        0.0, 1.0,
                                        0.0, 1.0, 0.0, 1.0,
                                        heading,
                                        "dspl_thresh",
                                        rwy_polys,
                                        shoulder_polys,
                                        slivers,
                                        accum,
                                        section_name );
                }
            }

            // final arrows
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( final_arrow / length );
            if ( debug ) { section_name = shapefile_name + "_disp_end"; }
            gen_runway_section( runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            "dspl_arrows",
                            rwy_polys,
                            shoulder_polys,
                            slivers,
                            accum,
                            section_name );
        }

        if (rwy.marking[rwhalf] == 0) {
            // No marking
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( 10 / length );
            if ( debug ) { section_name = shapefile_name + "_nothresh"; }
            gen_runway_section( runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                "no_threshold",
                                rwy_polys,
                                shoulder_polys,
                                slivers,
                                accum,
                                section_name );
        } else {
            // Thresholds for all others
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( 202.0 * SG_FEET_TO_METER / length );
            if ( debug ) { section_name = shapefile_name + "thresh"; }
            gen_runway_section( runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                "threshold",
                                rwy_polys,
                                shoulder_polys,
                                slivers,
                                accum,
                                shapefile_name );
        }

        // Runway designation block
        gen_rw_designation( runway_half, heading,
                            rwy.rwnum[rwhalf], start1_pct, end1_pct,
                            rwy_polys, slivers,
                            accum,
                            shapefile_name );

        // Generate remaining markings depending on type of runway
        if (rwy.marking[rwhalf] > 1) {
            std::vector<sections> rw_marking_list;
            rw_marking_list.clear();

            switch ( rwy.marking[rwhalf] ) {
                case 2:
    	            rw_marking_list.insert(  rw_marking_list.begin(), nprec, nprec + sizeof(nprec) / sizeof(nprec[0]) );
                    break;

                case 3:
                    rw_marking_list.insert(  rw_marking_list.begin(), prec, prec + sizeof(prec) / sizeof(prec[0]) );
                    break;

                case 4:
                    rw_marking_list.insert(  rw_marking_list.begin(), uk_nprec, uk_nprec + sizeof(uk_nprec) / sizeof(uk_nprec[0]) );
                    break;

                case 5:
                    rw_marking_list.insert(  rw_marking_list.begin(), uk_prec, uk_prec + sizeof(uk_prec) / sizeof(uk_prec[0]) );
                    break;
            }

            // Now create the marking sections of the runway type
            for ( unsigned int i=0; i < rw_marking_list.size(); ++i) {
	            TG_LOG(SG_GENERAL, SG_DEBUG, "Runway section texture = " << rw_marking_list[i].tex << " start_pct: " << start1_pct << " end_pct: " << end1_pct << " length: " << rw_marking_list[i].size);

                if ( end1_pct < 1.0 ) {
                    start1_pct = end1_pct;
                    end1_pct = start1_pct + ( rw_marking_list[i].size / length );
                    if ( debug ) { section_name = shapefile_name + "_" + rw_marking_list[i].tex; }
                    gen_runway_section( runway_half,
                                        start1_pct, end1_pct,
                                        0.0, 1.0,
                                        0.0, 1.0, 0.0, 1.0,
                                        heading,
                                        rw_marking_list[i].tex,
                                        rwy_polys,
                                        shoulder_polys,
                                        slivers,
                                        accum,
                                        section_name );
                }
            }
        }

        //
        // The rest ...
        //

        // fit the 'rest' texture in as many times as will go evenly into
        // the remaining distance so we don't end up with a super short
        // section at the end.
        double ideal_rest_inc = ( 200.0 * SG_FEET_TO_METER / length );
        int divs = (int)((1.0 - end1_pct) / ideal_rest_inc) + 1;
        double rest1_inc = (1.0 - end1_pct) / divs;

        while ( end1_pct < 1.0 ) {
            start1_pct = end1_pct;
            end1_pct = start1_pct + rest1_inc;
            if ( debug ) { section_name = shapefile_name + "rest_" + boost::lexical_cast<std::string>(end1_pct); }
            gen_runway_section( runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                "rest",
                                rwy_polys,
                                shoulder_polys,
                                slivers,
                                accum,
                                section_name );
        }

        start1_pct = 0.0;
        end1_pct = 0.0;
        
        if (rwy.overrun[rwhalf] > 0.0) {
            /* Generate approach end overrun */
            int count = (int) (rwy.overrun[rwhalf] * 2.0/ rwy.width);
            if (count < 1) {
                count = 1;
            }

            double part_len = rwy.overrun[rwhalf] / (double)count;
            for (int i = 0; i < count; ++i) {
                start1_pct = end1_pct;
                end1_pct = start1_pct + ( part_len / length );
                if ( debug ) { section_name = shapefile_name + "stopway"; }
                gen_runway_section( runway_half,
                                    -end1_pct, -start1_pct,
                                    0.0, 1.0,
                                    0.0, 1.0, 0.0, 1.0, //last number is lengthwise
                                    heading,
                                    "stopway",
                                    rwy_polys,
                                    shoulder_polys,
                                    slivers,
                                    accum,
                                    section_name );
            }
        }
    }
}

void Runway::BuildShoulder( tgpolygon_list& rwy_polys,
                            tgcontour_list& slivers,
                            tgAccumulator& accum )
{
    tgPolygon base, safe_base;
    tgPolygon shoulder;

    for (unsigned int i=0; i<shoulder_polys.size(); i++) {
        shoulder = shoulder_polys[i];

        // Clip the new polygon against what ever has already been created.
        tgPolygon clipped = accum.Diff( shoulder );
        //tgPolygon::RemoveSlivers( clipped, slivers );

        // Split long edges to create an object that can better flow with
        // the surface terrain
        tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );
        shoulder_polys[i] = split;

        rwy_polys.push_back( shoulder_polys[i] );
        accum.Add( shoulder );
    }
}
