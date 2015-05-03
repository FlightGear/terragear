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


#define DEBUG   1

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

#define RUNWAY_FEATS 1

using std::string;
struct sections
{
    const char* tex;
    double size;
};

#define PAR_LF  (0)

struct rwy_sections
{
    unsigned int method;    // method ( paralell polygon, paralell feature, perpendicular polygon, etc )
    unsigned int marking;   // linear feature / polygon to draw
    double       offset_l;  // how far down from desgnation
    bool from_centerline;   // offset_w is from centerline (t) or runway edge (f)
    double       offset_w;  // how far from cenerline or edge (-) left, (+) right 
    double       length;    // how long is it
    double       width;     // how wide is it ( for polygons )
};

// touchdown zone 3 : lines start at 500' - lines are 75' long
// they are 4' wide and offset from the edge of the runway
#define TZ_THREE            \
{ PAR_LF, RWY_TZONE, 500*SG_FEET_TO_METER, false, -12*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 500*SG_FEET_TO_METER, false, -20*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 500*SG_FEET_TO_METER, false, -28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 500*SG_FEET_TO_METER, false,  12*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 500*SG_FEET_TO_METER, false,  20*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 500*SG_FEET_TO_METER, false,  28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }

// touchdown zone 2 : starts at 1500', and 2000' - lines are 75' long
#define TZ_TWO_A            \
{ PAR_LF, RWY_TZONE, 1500*SG_FEET_TO_METER, false, -20*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 1500*SG_FEET_TO_METER, false, -28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 1500*SG_FEET_TO_METER, false,  20*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 1500*SG_FEET_TO_METER, false,  28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }

#define TZ_TWO_B            \
{ PAR_LF, RWY_TZONE, 2000*SG_FEET_TO_METER, false, -20*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 2000*SG_FEET_TO_METER, false, -28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 2000*SG_FEET_TO_METER, false,  20*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 2000*SG_FEET_TO_METER, false,  28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }

// touchdown zone 1 : starts at 2500', and 3000' - lines are 75' long
#define TZ_ONE_A            \
{ PAR_LF, RWY_TZONE, 2500*SG_FEET_TO_METER, false, -28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 2500*SG_FEET_TO_METER, false,  28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }

#define TZ_ONE_B            \
{ PAR_LF, RWY_TZONE, 3000*SG_FEET_TO_METER, false, -28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_TZONE, 3000*SG_FEET_TO_METER, false,  28*SG_FEET_TO_METER, 75*SG_FEET_TO_METER, 0 }

// aim point : starts at 1000' - it's 150' lonng and 20' wide
#define AIM                 \
{ PAR_LF, RWY_AIM, 1000*SG_FEET_TO_METER, false, -27*SG_FEET_TO_METER, 150*SG_FEET_TO_METER, 0 }, \
{ PAR_LF, RWY_AIM, 1000*SG_FEET_TO_METER, false,  27*SG_FEET_TO_METER, 150*SG_FEET_TO_METER, 0 }


static const struct rwy_sections precision[] = {
    TZ_THREE,
    AIM,
    TZ_TWO_A,
    TZ_TWO_B,
    TZ_ONE_A,
    TZ_ONE_B
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
    poly.Snap(gSnap);

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
void Runway::gen_section( const tgPolygon& runway,
                          double startl_pct, double endl_pct,
                          double startw_pct, double endw_pct,
                          double minu, double maxu, double minv, double maxv,
                          double heading,
                          const string& material,
                          bool with_shoulders )
{
    //double width = rwy.width;
    //double length = rwy.length;
    double lshoulder_width = 0.0f;
    double rshoulder_width = 0.0f;
    std::string shoulder_surface = "";

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

    if ( with_shoulders ) {
        // calculate if we are going to be creating shoulder polys
        if ( (rwy.shoulder > 0) && (rwy.surface < 3) ){
            if (rwy.shoulder == 1){
                shoulder_surface = "pa_shoulder";
            } else if (rwy.shoulder == 2){
                shoulder_surface = "pc_shoulder";
            }

            if ( startw_pct == 0.0f ) {
                lshoulder_width = 11.0;
            }
            if ( endw_pct == 1.0f ) {
                rshoulder_width = 11.0;
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
    section.Snap( gSnap );

    section.SetMaterial( "pa_tiedown" );
    section.SetTexParams( pg0, 5.0, 5.0, heading );
    section.SetTexLimits( 0.0, 0.0, 1.0, 1.0 );
    section.SetTexMethod( TG_TEX_BY_TPS_NOCLIP );

    runway_polys.push_back( section );
}

#if 0
void Runway::gen_feature( const tgPolygon& runway,
                          double startl_pct, double endl_pct,
                          double startw_pct, double endw_pct,
                          double minu, double maxu, double minv, double maxv,
                          double heading,
                          const string& material )
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
    
    double len = length / 2.0;
    double sect_len = len * ( endl_pct - startl_pct );
    double sect_wid = width * ( endw_pct - startw_pct );
    
    // ( for the old texturing )
    section.SetMaterial( material_prefix + material );    
    section.SetTexParams( pg0, sect_wid, sect_len, heading );
    section.SetTexLimits( minu, minv, maxu, maxv );
    section.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, 0.0, 0.0, 1.0, 1.0 );
    
    marking_polys.push_back( section );
}
#endif

static void SplitLongSegment( const SGGeod& p0, const SGGeod& p1, 
                              double max_len,
                              std::vector<SGGeod>& result ) {
    result.clear();
    double heading, dummy, dist;
    
    SGGeodesy::inverse(p0, p1, heading, dummy, dist);
    
    if ( fabs(p0.getLatitudeDeg()) < (90.0 - SG_EPSILON) &&
         fabs(p1.getLatitudeDeg()) < (90.0 - SG_EPSILON) ) {
        
        if ( dist > max_len ) {
            unsigned int segs = (int)(dist / max_len) + 1;
            
            double dx = (p1.getLongitudeDeg() - p0.getLongitudeDeg()) / segs;
            double dy = (p1.getLatitudeDeg()  - p0.getLatitudeDeg())  / segs;
            
            for ( unsigned int j = 0; j < segs; j++ ) {
                SGGeod tmp = SGGeod::fromDeg( p0.getLongitudeDeg() + dx * j, p0.getLatitudeDeg() + dy * j );
                result.push_back( tmp );
            }
        } else {
            result.push_back( p0 );
        }
    } else {
        result.push_back( p0 );
    }
        
    result.push_back( p1 );    
}
                              
tgContour Runway::GetSectionBB( const tgPolygon& runway,
                                double startl_pct, double endl_pct,
                                double startw_pct, double endw_pct,
                                double heading  )
{
    tgContour result;
    
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
    
    result.AddNode(pg0);
    result.AddNode(pg1);
    result.AddNode(pg3);
    result.AddNode(pg2);
    
    return result;
}

void Runway::gen_designation_polygon( const SGGeod& start_ref, double heading, double start_dist, double length, double width, double offset, const std::string& mark )
{
    // Atlas positions
    #define DESG_START_X    (0.0l)
    #define DESG_START_Y    (0.0l)
    #define DESG_WIDTH      (0.25l)
    #define DESG_HEIGHT     (0.25l)
    
    tgPolygon poly;

    double    offset_heading = SGMiscd::normalizePeriodic(0, 360, heading+90);
    double    minx, miny, maxx, maxy;
    
    SGGeod    bottom_ref = SGGeodesy::direct( start_ref,  heading, start_dist );
    bottom_ref = SGGeodesy::direct( bottom_ref, offset_heading, offset );
    SGGeod    top_ref        = SGGeodesy::direct( bottom_ref, heading, length );
        
    // first, generate the poly
    poly.AddNode( 0, SGGeodesy::direct( bottom_ref, offset_heading, -width/2 ));
    poly.AddNode( 0, SGGeodesy::direct( bottom_ref, offset_heading,  width/2 ));
    poly.AddNode( 0, SGGeodesy::direct( top_ref,    offset_heading,  width/2 ));    
    poly.AddNode( 0, SGGeodesy::direct( top_ref,    offset_heading, -width/2 ));
        
    // calculate atlas tcs describing the area we want to use
    if ( mark == "0" ) {
        minx = DESG_START_X+1*DESG_WIDTH;
        miny = DESG_START_Y+1*DESG_HEIGHT;
    } else if ( mark == "1" ) {
        minx = DESG_START_X+0*DESG_WIDTH;
        miny = DESG_START_Y+3*DESG_HEIGHT;
    } else if ( mark == "2" ) {
        minx = DESG_START_X+1*DESG_WIDTH;
        miny = DESG_START_Y+3*DESG_HEIGHT;
    } else if ( mark == "3" ) {
        minx = DESG_START_X+2*DESG_WIDTH;
        miny = DESG_START_Y+3*DESG_HEIGHT;
    } else if ( mark == "4" ) {
        minx = DESG_START_X+3*DESG_WIDTH;
        miny = DESG_START_Y+3*DESG_HEIGHT;        
    } else if ( mark == "5" ) {
        minx = DESG_START_X+0*DESG_WIDTH;
        miny = DESG_START_Y+2*DESG_HEIGHT;        
    } else if ( mark == "6" ) {
        minx = DESG_START_X+1*DESG_WIDTH;
        miny = DESG_START_Y+2*DESG_HEIGHT;        
    } else if ( mark == "7" ) {
        minx = DESG_START_X+2*DESG_WIDTH;
        miny = DESG_START_Y+2*DESG_HEIGHT;        
    } else if ( mark == "8" ) {
        minx = DESG_START_X+3*DESG_WIDTH;
        miny = DESG_START_Y+2*DESG_HEIGHT;        
    } else if ( mark == "9" ) {
        minx = DESG_START_X+0*DESG_WIDTH;
        miny = DESG_START_Y+1*DESG_HEIGHT;        
    } else if ( mark == "L" ) {
        minx = DESG_START_X+1*DESG_WIDTH;
        miny = DESG_START_Y+0*DESG_HEIGHT;        
    } else if ( mark == "C" ) {
        minx = DESG_START_X+2*DESG_WIDTH;
        miny = DESG_START_Y+0*DESG_HEIGHT;        
    } else if ( mark == "R" ) {
        minx = DESG_START_X+3*DESG_WIDTH;
        miny = DESG_START_Y+0*DESG_HEIGHT;        
    } else {
        TG_LOG(SG_GENERAL, SG_ALERT, "Unknown designation mark " << mark);
        return;
    }

    maxx = minx + DESG_WIDTH;
    maxy = miny + DESG_HEIGHT;    
    
    poly.SetMaterial( "rwy_designations" );
    poly.SetTexParams( poly.GetNode(0,0), width, length, heading );
    poly.SetTexMethod( TG_TEX_1X1_ATLAS );
    poly.SetTexLimits( minx, miny, maxx, maxy );
    poly.SetVertexAttributeInt(TG_VA_CONSTANT, 0, 1);
    
    cap_polys.push_back( poly );
}

LinearFeature* Runway::gen_perpendicular_marking_feature( Airport* ap, const SGGeod& start_ref, double heading, double start_dist, double length, double width, int mark )
{
    LinearFeature* feat = new LinearFeature((const char *)"feat", 0 );
    BezNode* node = NULL;
    std::vector<SGGeod> nodelist;
    
    double left_heading = SGMiscd::normalizePeriodic(0, 360, heading-90);
    double right_heading = SGMiscd::normalizePeriodic(0, 360, heading+90);
    
    // first, travel heading from start to start distance (+1/2 of length for feature)
    SGGeod start = SGGeodesy::direct( start_ref, heading, start_dist + (0.5*length) );
    
    // Start the feature from the right to the left
    SGGeod left  = SGGeodesy::direct( start, left_heading,  width/2 );
    SGGeod right = SGGeodesy::direct( start, right_heading, width/2 );
    
    // break into segments
    SplitLongSegment( left, right, 50, nodelist );    
    
    for ( unsigned int i=0; i<nodelist.size()-1; i++ ) {
        node = new BezNode(nodelist[i]);
        node->SetMarking( mark );
        feat->AddNode( node );
    }
    
    node = new BezNode( nodelist[nodelist.size()-1] );
    node->SetMarking( mark );
    node->SetTerm(true);
    feat->AddNode( node );
    
    feat->Finish( ap, false);
    
    return feat;
}

LinearFeature* Runway::gen_paralell_marking_feature( Airport* ap, const SGGeod& start_ref, double heading, double start_dist, double length, double offset, int mark )
{
    LinearFeature* feat = new LinearFeature((const char *)"feat", 0 );
    BezNode* node = NULL;
    std::vector<SGGeod> nodelist;
    
    double right_heading = SGMiscd::normalizePeriodic(0, 360, heading+90);
    
    // first, travel heading from start to start distance
    SGGeod start = SGGeodesy::direct( start_ref, heading, start_dist );
    
    // Start the feature from the start to the end paralell to heading, offset by offset
    start  = SGGeodesy::direct( start, right_heading,  offset );
    SGGeod end    = SGGeodesy::direct( start, heading, length );
    
    // break into segments
    SplitLongSegment( start, end, 50, nodelist );    
    
    for ( unsigned int i=0; i<nodelist.size()-1; i++ ) {
        node = new BezNode(nodelist[i]);
        node->SetMarking( mark );
        feat->AddNode( node );
    }
    
    node = new BezNode( nodelist[nodelist.size()-1] );
    node->SetMarking( mark );
    node->SetTerm(true);
    feat->AddNode( node );
    feat->Finish( ap, false);
    
    return feat;
}

LinearFeature* Runway::gen_chevron_feature( Airport* ap, const SGGeod& start_ref, double heading, double start_dist, double length, double width, double offset, int mark )
{
    // need to make a contour of two lines.
    LinearFeature* feat = new LinearFeature((const char *)"feat", 0 );
    BezNode* node = NULL;
    
    double right_heading = SGMiscd::normalizePeriodic(0, 360, heading+90);
    
    // first, travel heading from start to start distance
    SGGeod start = SGGeodesy::direct( start_ref, heading, start_dist );
    
    // Start the feature from the start to the end paralell to heading, offset by offset
    start  = SGGeodesy::direct( start, right_heading,  offset );
    SGGeod end    = SGGeodesy::direct( start, heading, length );
    
    node = new BezNode( SGGeodesy::direct(start, right_heading, -width/2) );
    node->SetMarking( mark );
    feat->AddNode( node );

    node = new BezNode( end );
    node->SetMarking( mark );
    feat->AddNode( node );

    node = new BezNode( SGGeodesy::direct(start, right_heading, width/2) );
    node->SetMarking( mark );
    node->SetTerm(true);
    feat->AddNode( node );
    
    feat->Finish( ap, false);
    
    return feat;    
}

void Runway::gen_threshold( Airport* ap, const SGGeod& start, double heading )
{
    int num_marks;
    
    // first, draw the thresholdbar
    LinearFeature* threshold_bar = gen_perpendicular_marking_feature( ap, start, heading, 0, 10*SG_FEET_TO_METER, rwy.width, RWY_THRESH ); 
    features.push_back( threshold_bar );
    
    // then generate the threshold marking starting from 20'. 50' long
    if ( rwy.width >= 200*SG_FEET_TO_METER ) {
        num_marks = 16;
    } else if ( rwy.width >= 150*SG_FEET_TO_METER ) {
        num_marks = 12;
    } else if ( rwy.width >= 100*SG_FEET_TO_METER ) {
        num_marks = 8;
    } else if ( rwy.width >= 75*SG_FEET_TO_METER ) {
        num_marks = 6;
    } else if ( rwy.width >= 60*SG_FEET_TO_METER ) {
        num_marks = 6;
    } else {
        TG_LOG(SG_GENERAL, SG_INFO, "Runway too narrow for threshold markings ");
        num_marks = 0;
    }

    if ( num_marks ) {
        // now determine the spaceing between num_marks ( add one imaginary mark for the middle, and 1 for each side )
        num_marks += 3;
        
        double mark_spacing = (rwy.width - (num_marks * 5.75 * SG_FEET_TO_METER) ) / num_marks;
        int    center_mark = floor(num_marks/2); // mark in the center of runway
        
        // now we can start creating the mark_spacing
        LinearFeature* threshold_mark = NULL;
        
        for ( int m=1; m<num_marks-1; m++ ) {
            if ( m != center_mark ) {
                int    mark_offset = m - center_mark;    // negetive to the left, positive to the right
                double feat_offset = mark_offset * ( (5.75*SG_FEET_TO_METER) + mark_spacing ); // mark width + spacing

                threshold_mark = gen_paralell_marking_feature( ap, start, heading, 20*SG_FEET_TO_METER, 150*SG_FEET_TO_METER, feat_offset, RWY_THRESH );
                features.push_back( threshold_mark );
            }
        }
    }
}
    
SGGeod Runway::gen_designation( const SGGeod& start_ref, int rwhalf, double heading )
{
    string rwname = rwy.rwnum[rwhalf];
    double numeral_dist = 0.0f;
    SGGeod end_ref;

    TG_LOG(SG_GENERAL, SG_INFO, "Runway name is " << rwy.rwnum[rwhalf] );
    
    if (rwname != "XX") { /* Do not create a designation block if the runway name is set to none */
        string letter = "";
        
        for ( unsigned int i = 0; i < rwname.length(); ++i ) {
            string tmp = rwname.substr(i, 1);
            if ( tmp == "L" || tmp == "R" || tmp == "C" ) {
                rwname = rwname.substr(0, i);
                letter = tmp;
            }
        }
        TG_LOG(SG_GENERAL, SG_INFO, "Runway designation letter = " << letter);

        // create runway designation letter
        if ( !letter.empty() ) {
            gen_designation_polygon( start_ref, heading, 200*SG_FEET_TO_METER, 60*SG_FEET_TO_METER, 60*SG_FEET_TO_METER, 0, letter );
            numeral_dist = 280*SG_FEET_TO_METER;
        } else {
            numeral_dist = 200*SG_FEET_TO_METER;
        }

        // create runway designation number(s)
        if (rwname == "0") {
            rwname = "36";
        }

        TG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation = " << rwname);

        if (rwname.length() == 2) {
            gen_designation_polygon( start_ref, heading, numeral_dist, 60*SG_FEET_TO_METER, 60*SG_FEET_TO_METER, -30*SG_FEET_TO_METER, rwname.substr(0,1));
            gen_designation_polygon( start_ref, heading, numeral_dist, 60*SG_FEET_TO_METER, 60*SG_FEET_TO_METER,  30*SG_FEET_TO_METER, rwname.substr(1,1));            
        } else if (rwname.length() == 1) {
            gen_designation_polygon( start_ref, heading, numeral_dist, 60*SG_FEET_TO_METER, 60*SG_FEET_TO_METER,   0*SG_FEET_TO_METER, rwname.substr(0,1));
        }
        
        end_ref = SGGeodesy::direct( start_ref, heading, numeral_dist + 100*SG_FEET_TO_METER );
    }
    
    return end_ref;
}

void Runway::gen_border( Airport* ap, const SGGeod& start, const SGGeod& end, double heading, double dist )
{
    LinearFeature*      border;
    BezNode*            node;
    std::vector<SGGeod> splitSeg;
    SGGeod              top_left, bot_left, top_right, bot_right;
    double              border_offset = 0.75;
    double              thresh_offset = 10*SG_FEET_TO_METER - 0.25;
    
    border = new LinearFeature( (const char *)"rwy_border", 0 );
    
    // Start the contour from top left to bottom left
    top_left  = SGGeodesy::direct( end, SGMiscd::normalizePeriodic(0, 360, heading-90), rwy.width/2 - border_offset );
    top_right = SGGeodesy::direct( end, SGMiscd::normalizePeriodic(0, 360, heading+90), rwy.width/2 - border_offset );

    bot_left  = SGGeodesy::direct( start, SGMiscd::normalizePeriodic(0, 360, heading-90), rwy.width/2 - border_offset );
    bot_left  = SGGeodesy::direct( bot_left, heading, thresh_offset );
    bot_right = SGGeodesy::direct( start, SGMiscd::normalizePeriodic(0, 360, heading+90), rwy.width/2 - border_offset );
    bot_right = SGGeodesy::direct( bot_right, heading, thresh_offset );
    
    // Draw the left side border
    SplitLongSegment(top_left, bot_left, 50, splitSeg);
    for( unsigned int i=0; i<splitSeg.size(); i++ ) {
        node = new BezNode( splitSeg[i] );
        node->SetMarking(RWY_BORDER);
        if ( i == splitSeg.size()-1 ) {
            node->SetTerm(true);            
        }
        border->AddNode( node );
    }
    
    // terminate and add the feature
    border->Finish( ap, false );    
    features.push_back( border );

    border = new LinearFeature( (const char *)"rwy_border", 0 );
    
    // Draw the right size border
    SplitLongSegment(bot_right, top_right, 50, splitSeg);
    for( unsigned int i=0; i<splitSeg.size(); i++ ) {
        node = new BezNode( splitSeg[i] );
        node->SetMarking(RWY_BORDER);
        if ( i == splitSeg.size()-1 ) {
            node->SetTerm(true);            
        }
        border->AddNode( node );
    }

    // terminate and add the feature
    border->Finish( ap, false );    
    features.push_back( border );
}


// threshold bar
// 10 feet wide
// threshold markings
// 5.75 feet wide
//


void Runway::gen_base( Airport* ap, const SGGeod& start, const SGGeod& end, double heading, double dist, bool with_shoulders )
{
    tgPolygon   runway, left_shoulder, right_shoulder;
    double      offset_heading;
    double      offset_width;
    double      shoulder_width = 0.0f;
    std::string shoulder_surface = "";
    SGGeod      ref;
    
    // if we want shoulders, generate them based on input poly
    if ( with_shoulders ) {
        // calculate if we are going to be creating shoulder polys
        if ( (rwy.shoulder > 0) && (rwy.surface < 3) ){
            if (rwy.shoulder == 1){
                shoulder_surface = "pa_shoulder";
            } else if (rwy.shoulder == 2){
                shoulder_surface = "pc_shoulder";
            }
            
            shoulder_width = 11.0;
        } else {  
            // We add a fake shoulder if the runway has an asphalt or concrete surface
            if ( (rwy.surface == 1) || (rwy.surface == 2) ) {
                if (rwy.surface == 1) {
                    shoulder_surface = "pa_shoulder_f";
                } else if (rwy.surface == 2){
                    shoulder_surface = "pc_shoulder_f";
                }
                
                shoulder_width = 1.0;
            }
        }
    }
        
    // now split the line segment to generate the polys
    std::vector<SGGeod> midline;
    SplitLongSegment( start, end, 50, midline );
    
    // Generate the runway poly
    offset_heading = SGMiscd::normalizePeriodic(0, 360, heading + 90);
    offset_width   = rwy.width/2;
    for ( int n=0; n<(int)midline.size(); n++) {
        SGGeod pt = SGGeodesy::direct( midline[n], offset_heading, offset_width );
        runway.AddNode(0, pt);
    }
    offset_heading = SGMiscd::normalizePeriodic(0, 360, heading - 90);    
    for ( int n=midline.size()-1; n>=0; n--) {
        SGGeod pt = SGGeodesy::direct( midline[n], offset_heading, offset_width );
        runway.AddNode(0, pt);
    }
    runway.Snap( gSnap );
    runway.SetMaterial( "pa_tiedown" );
    runway.SetTexParams( runway.GetNode(0,0), 5.0, 5.0, heading );
    runway.SetTexLimits( 0.0, 0.0, 1.0, 1.0 );
    runway.SetTexMethod( TG_TEX_BY_TPS_NOCLIP );
    runway_polys.push_back( runway );
    
#if DEBUG
    tgShapefile::FromPolygon( runway, true, false, "./dbg", "Runway", "runway" );
#endif
    
    /* Now add the white border around the whole runway (0.5M wide) */
    // generate right poly

#if RUNWAY_FEATS
    gen_border( ap, start, end, heading, dist );
#endif
    
    if ( with_shoulders ) {
        // generate right shoulder
        offset_heading = SGMiscd::normalizePeriodic(0, 360, heading + 90);
        for ( int n=0; n<(int)midline.size(); n++) {
            SGGeod pt = SGGeodesy::direct( midline[n], offset_heading, offset_width+shoulder_width );
            right_shoulder.AddNode(0, pt);

            // this is the primary texture reference point
            if ( n == 0 ) { ref = pt; }            
        }
        for ( int n=midline.size()-1; n>=0; n--) {
            SGGeod pt = SGGeodesy::direct( midline[n], offset_heading, offset_width );
            right_shoulder.AddNode(0, pt);
        }
        
        right_shoulder.Snap( gSnap );
        right_shoulder.SetMaterial( shoulder_surface );
        right_shoulder.SetTexParams( ref, shoulder_width, 5, heading );
        right_shoulder.SetTexLimits( 0,0,1,1 );
        right_shoulder.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, 0.0, 1.0, 0.0 );
        shoulder_polys.push_back( right_shoulder );

#if DEBUG
        tgShapefile::FromPolygon( right_shoulder, true, false, "./dbg", "right_shoulder", "right_shoulder" );
#endif
        
        // generate left shoulder
        offset_heading = SGMiscd::normalizePeriodic(0, 360, heading - 90);
        for ( int n=midline.size()-1; n>=0; n--) {
            SGGeod pt = SGGeodesy::direct( midline[n], offset_heading, offset_width+shoulder_width );
            left_shoulder.AddNode(0, pt);
        }
        for ( int n=0; n<(int)midline.size(); n++) {
            SGGeod pt = SGGeodesy::direct( midline[n], offset_heading, offset_width );
            left_shoulder.AddNode(0, pt);
            // this is the primary texture reference point
            if ( n == (int)midline.size()-1 ) { ref = pt; }                
        }
        left_shoulder.Snap( gSnap );
        left_shoulder.SetMaterial( shoulder_surface );
        left_shoulder.SetTexParams( ref, shoulder_width, 5, SGMiscd::normalizePeriodic(0, 360, heading+180 ) );
        left_shoulder.SetTexLimits( 0,0,1,1 );
        left_shoulder.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, 0.0, 1.0, 0.0 );
        shoulder_polys.push_back( left_shoulder );    

#if DEBUG
        tgShapefile::FromPolygon( left_shoulder, true, false, "./dbg", "left_shoulder", "left_shoulder" );
#endif        
    }
}

SGGeod Runway::gen_disp_thresh( Airport* ap, const SGGeod& start, double displacement, double heading )
{
    SGGeod thresh = SGGeodesy::direct( start, heading, displacement );
    
    // first, calculate the number of chevrons
    int num_chevrons;
    double chevron_spacing, border_spacing;
    
    if ( rwy.width >= 100 ) {
        num_chevrons = 4;
        chevron_spacing = rwy.width/4;
        border_spacing = rwy.width/8;
    } else if ( rwy.width >= 60 ) {
        num_chevrons = 3;
        chevron_spacing = rwy.width/3;
        border_spacing = rwy.width/6;
    } else {
        num_chevrons = 2;
        chevron_spacing = rwy.width/2;
        border_spacing = rwy.width/4;
    }

    TG_LOG(SG_GENERAL, SG_INFO, "Runway is " << rwy.width << " meters wide: draw " << num_chevrons << " chevrons " << chevron_spacing << " meters apart " );
    
    // start on the left side (-), border spacing from the border - box is 15' wide
    double cur_offset = -rwy.width/2 + border_spacing;
    
    // each chevron is painted 3' wide in a 15'x45' box 5' behind the threshold
    for ( int i=0; i<num_chevrons; i++ ) {
        LinearFeature* chevron = gen_chevron_feature( ap, start, heading, 
                                                      displacement-(55*SG_FEET_TO_METER), 
                                                      45*SG_FEET_TO_METER, 15*SG_FEET_TO_METER, 
                                                      cur_offset, RWY_BORDER );
        features.push_back( chevron );
        cur_offset += chevron_spacing;
    }
    
    // now generate the center arrows.
    int num_arrows = (displacement-24)/60;
    double arrow_offset = -37.5;    // 24 meters + 13.5 chevron length
    
    TG_LOG(SG_GENERAL, SG_INFO, "Runway displaced threshold is " << displacement << " meters long: draw " << num_arrows << " arrows " );
    
    for ( int i=0; i<num_arrows; i++ ) {
        LinearFeature* chevron = gen_chevron_feature( ap, thresh, heading, 
                                                      arrow_offset, 
                                                      45*SG_FEET_TO_METER, 15*SG_FEET_TO_METER, 
                                                      0, RWY_BORDER );
        features.push_back( chevron );
        
        LinearFeature* tail = gen_paralell_marking_feature( ap, thresh, heading, 
                                                            arrow_offset-22.5, 24, 
                                                            0.0, RWY_DISP_TAIL );        
        features.push_back( tail );
        
        arrow_offset -= 60;
    }
        
    return thresh;
}

void Runway::gen_stopway( Airport* ap, const SGGeod& start, double length, double heading )
{
    tgPolygon   stopway;
    double      offset_heading;
    double      offset_width;
    double      width = rwy.width;

    TG_LOG(SG_GENERAL, SG_INFO, "stopway is " << length << " meters" );
    
    // first, just generate the pavement - stopway or blastpad?
    // if we have shoulders - draw a blastpad, otherwise stopway
    if ( (rwy.shoulder > 0) && (rwy.surface < 3) ){
        width += 22.0;
    } else {  
        width += 2;
    }
    
    // star the stopway length away in the opposite direction of the runway
    SGGeod ref = SGGeodesy::direct( start, heading, -length );
    
    // Generate the runway poly
    offset_heading = SGMiscd::normalizePeriodic(0, 360, heading + 90);
    offset_width   = width/2;

    stopway.AddNode(0, SGGeodesy::direct( ref,   offset_heading, -offset_width ));
    stopway.AddNode(0, SGGeodesy::direct( ref,   offset_heading,  offset_width ));
    stopway.AddNode(0, SGGeodesy::direct( start, offset_heading,  offset_width ));
    stopway.AddNode(0, SGGeodesy::direct( start, offset_heading, -offset_width ));
    
    stopway.Snap( gSnap );
    stopway.SetMaterial( "pa_tiedown" );
    stopway.SetTexParams( stopway.GetNode(0,0), 5.0, 5.0, heading );
    stopway.SetTexLimits( 0.0, 0.0, 1.0, 1.0 );
    stopway.SetTexMethod( TG_TEX_BY_TPS_NOCLIP );
    
#if DEBUG
    tgShapefile::FromPolygon( stopway, true, false, "./dbg", "Stopway", "stopway" );
#endif
    
    runway_polys.push_back( stopway );
    

    // now add the chevrons
    // The first one is partial and starts 15m from start of threshold
    // ( make sure threshold is drawn first, so we clip correctly )
    LinearFeature* partial;
    SGGeod part_start, part_end;
    BezNode* node;
    
    // don't go all the way to the edges
    offset_width -= 1.5;
    
    part_start = SGGeodesy::direct( start, offset_heading, -15 );
    part_end   = SGGeodesy::direct( start, heading, -((offset_width)-15) );
    part_end   = SGGeodesy::direct( part_end, offset_heading, -(offset_width) );

    partial = new LinearFeature( "feat", 0 );    
    node = new BezNode(part_start);
    node->SetMarking(RWY_BORDER);
    partial->AddNode(node);
    node = new BezNode(part_end);
    node->SetMarking(RWY_BORDER);
    node->SetTerm(true);
    partial->AddNode(node);
    partial->Finish(ap, false);
    features.push_back( partial );
    
    part_start = SGGeodesy::direct( start, offset_heading, 15 );
    part_end   = SGGeodesy::direct( start, heading, -((offset_width)-15) );
    part_end   = SGGeodesy::direct( part_end, offset_heading, (offset_width) );
    
    partial = new LinearFeature( "feat", 0 );    
    node = new BezNode(part_start);
    node->SetMarking(RWY_BORDER);
    partial->AddNode(node);
    node = new BezNode(part_end);
    node->SetMarking(RWY_BORDER);
    node->SetTerm(true);
    partial->AddNode(node);
    partial->Finish( ap, false );
    features.push_back( partial );
    
    // now calculate the number of full chevrons
    int num_chevrons = (length-15) / 30;
    double cur_offset = -(15 + offset_width);
    
    // each chevron is painted 3' wide in a 15'x45' box 5' behind the threshold
    for ( int i=0; i<num_chevrons; i++ ) {
        LinearFeature* chevron = gen_chevron_feature( ap, start, heading, 
                                                      cur_offset, 
                                                      offset_width, offset_width*2, 
                                                      0, RWY_BORDER );
        features.push_back( chevron );
        cur_offset -= 30;
    }
    
    // then the last partial ( if applicable )
    double last_partial_length = length-15-(30*num_chevrons)-1.5;
    if ( last_partial_length > 1.0 ) {
        part_start = SGGeodesy::direct( start, heading, -(length-1.5) );
        offset_width *= (last_partial_length/30);
        LinearFeature* chevron = gen_chevron_feature( ap, part_start, heading, 
                                                      0, 
                                                      offset_width, offset_width*2, 
                                                      0, RWY_BORDER );
        features.push_back( chevron );
    }
}

// generate a runway.  The routine modifies
// rwy_polys, texparams, and accum.  For specific details and
// dimensions of precision runway markings, please refer to FAA
// document AC 150/5340-1H
void Runway::gen_full_rwy(Airport* ap)
{
    TG_LOG( SG_GENERAL, SG_DEBUG, "Building runway = " << rwy.rwnum[0] << " / " << rwy.rwnum[1]);

    // we really want to build runway and shoulders at the same time.  We just need 2 
    // directed line segments from each end - and end at the midpoint
    SGGeod center = SGGeodesy::direct(GetStart(), rwy.heading, rwy.length/2 );
    double heading;
    SGGeod start_ref;
    
    for ( int rwhalf = 0; rwhalf < 2; ++rwhalf ) {
        if (rwhalf == 0) {
            start_ref = GetStart();
            heading = rwy.heading;            
        } else {
            start_ref = GetEnd();
            heading = SGMiscd::normalizePeriodic(0, 360, rwy.heading + 180);
        }

        // data sanity checks
        
        // Make sure our runway is long enough for the desired marking variant
        if ( (rwy.marking[rwhalf]==2 || rwy.marking[rwhalf]==4) && ( (rwy.length/2) < 1150 * SG_FEET_TO_METER) ) {
            TG_LOG( SG_GENERAL, SG_ALERT,
                    "Runway " << rwy.rwnum[rwhalf] << " is not long enough ("
                    << rwy.length << "m) for non-precision markings!  Setting runway markings to visual!");
            rwy.marking[rwhalf]=1;
        }
        
        if ( (rwy.marking[rwhalf]==3 || rwy.marking[rwhalf]==5) && ( (rwy.length/2) < 3075 * SG_FEET_TO_METER) ) {
            TG_LOG( SG_GENERAL, SG_ALERT,
                    "Runway " << rwy.rwnum[rwhalf] << " is not long enough ("
                    << rwy.length << "m) for precision markings!  Setting runway markings to visual!");
            rwy.marking[rwhalf]=1;
        }

        // create the runway polys and borders ( simple without markings )
        gen_base( ap, start_ref, center, heading, rwy.length/2, true );

#if RUNWAY_FEATS
        if (rwy.overrun[rwhalf] > 0.0) {
            TG_LOG( SG_GENERAL, SG_INFO, "runway heading = " << heading << " designation " << rwy.rwnum[rwhalf] << " has overrun " << rwy.overrun[rwhalf] );
            
            gen_stopway( ap, start_ref, rwy.overrun[rwhalf], heading );
        }
        
        if ( rwy.threshold[rwhalf] > 0.0 ) {
            TG_LOG( SG_GENERAL, SG_INFO, "Displaced threshold for RW side " << rwhalf << " is " << rwy.threshold[rwhalf] );
            start_ref = gen_disp_thresh( ap, start_ref, rwy.threshold[rwhalf], heading );
        }
        
        if ( rwy.marking[rwhalf] != 0) {                
            gen_threshold( ap, start_ref, heading );
        }
        
        // current start_ref (threshold) is the reference point for all the precision markings
        SGGeod mark_ref = SGGeodesy::direct( start_ref, heading, (10)*SG_FEET_TO_METER );
        
        // Runway designation markings : return is the start of the centerline 
        start_ref = gen_designation( start_ref, rwhalf, heading );

        // draw the cenerline.  We draw each segment individually instead of on long dotted segment
        // because runways are long, the curvature of the earth has an efect, and the lines won't
        // meet in the center if we start at the rwy designation
        // see 150_5340_1l section 2.4.e note 3.
        //
        // To accommodate varying runway lengths, all adjustments to the uniform
        // pattern of runway centerline stripes and gaps are made near the runway midpoint (defined as the
        // distance between the two thresholds or displaced thresholds). Under such cases, reduce the
        // lengths of both the stripes and gaps starting from midpoint and proceed toward the runway
        // thresholds. Reduced stripes must be at least 80 feet (24 m) in length, and the reduced gaps must
        // be at least 40 feet (12.3 m) in length. The affected stripes and gaps within the section should
        // show a uniform pattern.
        
        // half a gap, and 1/3 a stripe
        #define SEGMENT_LENGTH  (SG_FEET_TO_METER*40)    
        #define STRIPE_LENGTH   (SEGMENT_LENGTH*3)
        #define GAP_LENGTH      (SEGMENT_LENGTH*2)
        
        // to conform with this, we take the length from the desgnation to 14*segment length from the midpoint.
        // we then divide this length into stripe/gap combos and round up.
        // the remaining segments are compressed by remaining/560 

        double offset_length = SGGeodesy::distanceM(start_ref, center) - (14*SEGMENT_LENGTH);
        unsigned int num_full_sections = (offset_length / (STRIPE_LENGTH+GAP_LENGTH)) + 1;
        double uncompressed_length = num_full_sections * (STRIPE_LENGTH+GAP_LENGTH);
        double compressed_length = SGGeodesy::distanceM(start_ref, center) - uncompressed_length;
        double compress_ratio = compressed_length/(14*SEGMENT_LENGTH);

        TG_LOG( SG_GENERAL, SG_ALERT, " RWY CENTERLINE: offset_length is " << offset_length << " full_segs is " << num_full_sections );
        TG_LOG( SG_GENERAL, SG_ALERT, "                 uncompressed_length is " << uncompressed_length << " compressed_length " << compressed_length );
        TG_LOG( SG_GENERAL, SG_ALERT, "                 compress_ratio is " << compress_ratio  );
        
        for ( unsigned int i=0; i<num_full_sections; i++ ) {
            // now loop from start to end...
            double next_heading = SGGeodesy::courseDeg( start_ref, center );            
            LinearFeature* centerline = gen_paralell_marking_feature( ap, start_ref, next_heading, 
                                                                      0, STRIPE_LENGTH, 
                                                                      0.0, RWY_BORDER );
            features.push_back(centerline);
            
            // generate next start ref
            start_ref = SGGeodesy::direct( start_ref, next_heading, (STRIPE_LENGTH+GAP_LENGTH) );
        }
        
        // now generate the 3 compressed sections
        for ( unsigned int i=0; i<3; i++ ) {
            double next_heading = SGGeodesy::courseDeg( start_ref, center );
            LinearFeature* centerline = gen_paralell_marking_feature( ap, start_ref, next_heading, 
                                                                      0, STRIPE_LENGTH*compress_ratio, 
                                                                      0.0, RWY_BORDER );
            features.push_back(centerline);
            
            // generate next start ref
            start_ref = SGGeodesy::direct( start_ref, next_heading, (STRIPE_LENGTH+GAP_LENGTH)*compress_ratio );
        }

        // Draw the runway markings
        if (rwy.marking[rwhalf] > 1) {
            std::vector<rwy_sections> rw_marking_list;
            rw_marking_list.clear();

            switch ( rwy.marking[rwhalf] ) {
                case 2:
                    // rw_marking_list.insert(  rw_marking_list.begin(), nprec, nprec + sizeof(nprec) / sizeof(nprec[0]) );
                    break;
                
                case 3:
                    rw_marking_list.insert(  rw_marking_list.begin(), precision, precision + sizeof(precision) / sizeof(precision[0]) );
                    break;
                
                case 4:
                    // rw_marking_list.insert(  rw_marking_list.begin(), uk_nprec, uk_nprec + sizeof(uk_nprec) / sizeof(uk_nprec[0]) );
                    break;
                
                case 5:
                    // rw_marking_list.insert(  rw_marking_list.begin(), uk_prec, uk_prec + sizeof(uk_prec) / sizeof(uk_prec[0]) );
                    break;
            }

            // Now create the marking sections of the runway type
            for ( unsigned int i=0; i < rw_marking_list.size(); ++i) {
                switch( rw_marking_list[i].method ) {
                    case PAR_LF:
                    {
                        double offset_w;
                        if ( rw_marking_list[i].from_centerline ) {
                            offset_w = rw_marking_list[i].offset_w;
                        } else if ( rw_marking_list[i].offset_w > 0 ) {
                            offset_w = (rwy.width/2) - rw_marking_list[i].offset_w;
                        } else {
                            offset_w = -1 * ((rwy.width/2) - -rw_marking_list[i].offset_w);
                        }
                        
                        LinearFeature* lf = gen_paralell_marking_feature(ap, mark_ref, heading, 
                                                                        rw_marking_list[i].offset_l,
                                                                        rw_marking_list[i].length,
                                                                        offset_w,
                                                                        rw_marking_list[i].marking
                                                                        );
                        features.push_back( lf );
                    }
                    break;
                }
            }
        }
#endif

    }
}
