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

#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include "global.hxx"
#include "apt_math.hxx"
#include "helipad.hxx"
#include "runway.hxx"
#include "debug.hxx"

#include <cstdio>
#include <stdlib.h>
#include <boost/concept_check.hpp>

Helipad::Helipad(char* definition)
{

    // format:
    // helipad  designator   lat  lon  heading      length   width  surface     markings    shoulder    smoothness    edge lighting

    // int fscanf(FILE *stream, const char *format, ...);
    sscanf(definition, "%s %lf %lf %lf %lf %lf %d %d %d %lf %d",
        designator, &lat, &lon, &heading, &length, &width, &surface,
        &marking, &shoulder, &smoothness, &edge_lights);

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read helipad: (" << lon << "," << lat << ") heading: " << heading << " length: " << length << " width: " << width );
}

tglightcontour_list Helipad::gen_helipad_lights(double maxsize) {
    tglightcontour_list result;

    // Vector calculation
    SGVec3f vec = normalize(SGVec3f::fromGeod(GetLoc()));

    // Create yellow edge lights, 5m spacing
    cgalPoly_Polygon area = gen_helipad_area_w_extend(0.0, 0.0);
    tgLightContour yellow;

    yellow.SetType( "RWY_YELLOW_LIGHTS" );
    for ( unsigned int i = 0; i < area.size(); ++i ) {
        double         dist, course, cs;
        unsigned int   srcIdx = i;
        unsigned int   trgIdx = (i==3) ? 0 : i+1;
        SGGeod sgSrc = SGGeod::fromDeg( CGAL::to_double( area[srcIdx].x() ), CGAL::to_double( area[srcIdx].y() ) );
        SGGeod sgTrg = SGGeod::fromDeg( CGAL::to_double( area[trgIdx].x() ), CGAL::to_double( area[trgIdx].y() ) );
        
        SGGeodesy::inverse(sgSrc, sgTrg, course, cs, dist );
        int    divs = (int)(dist / 5.0);
        double step = dist/divs;
        for (int j = 0; j < divs; ++j) {
            SGGeod sgLightPt = SGGeodesy::direct(sgSrc, course, step );
            yellow.AddLight( sgLightPt, vec );
        }
    }

    // Create a circle of yellow lights where the white texture circle is
    for (int deg = 0; deg < 360; deg += 45){
        yellow.AddLight( SGGeodesy::direct(GetLoc(), deg, maxsize * 0.46), vec );
    }

    result.push_back( yellow );

    return result;
}

void Helipad::build_helipad_shoulders( const cgalPoly_Polygon& outer_area )
{
    double shoulder_width, shoulder_heading;
    std::string shoulder_mat;
    char desc[256];

    // Now build the shoulders
    if (shoulder == 1) {
        shoulder_mat = "pa_shoulder";
        shoulder_width = 6; // shoulder size in m
    } else if (shoulder == 2) {
        shoulder_mat = "pc_shoulder";
        shoulder_width = 6; // shoulder size in m
    } else if (surface == 1) {
        shoulder_mat  = "pa_shoulder_f";
        shoulder_width = 1; // shoulder size in m
    } else {
        shoulder_mat  = "pc_shoulder_f";
        shoulder_width = 1; // shoulder size in m
    }
    shoulder_heading = heading;

    cgalPoly_Polygon shoulderArea = gen_helipad_area_w_extend(shoulder_width, shoulder_width);

    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, shoulder_mat );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_TPS_CLIPU, 0.0l, 0.0l, 1.0l, 1.0l );
    meta.setTextureLimits( 1.0l, 1.0l, 0.0l, 0.0l );
    
    for (int i = 0; i < 4; ++i) {
        shoulder_heading = SGMiscd::normalizePeriodic(0,360,shoulder_heading-90);
        
        cgalPoly_Polygon shoulderCur;
        SGGeod           pt;

        shoulderCur.clear();
        shoulderCur.push_back( shoulderArea[i] );
        shoulderCur.push_back( shoulderArea[i == 3 ? 0 : i+1] );
        shoulderCur.push_back( outer_area[i == 3 ? 0 : i+1] );
        shoulderCur.push_back( outer_area[i] );

        meta.setTextureRef( shoulderCur[1], shoulder_width, 5.0l, shoulder_heading );
        
        snprintf( desc, 256, "%s_shoulder_%d", designator, i );
        meta.setDescription( desc );
        
        shoulderPolys.push_back( tgPolygonSet( shoulderCur, meta ) );
    }
}

tgPolygonSetList& Helipad::GetMainPolys( void )
{
    //
    // Generate the basic helipad outlines
    //
    double heli_size;
    char desc[256];
    
    // helipad constructed as a square with width or length added around it
    if ( width == length ) {
        heli_size = length;
    } else if ( width > length ) {
        heli_size = length;
    } else {
        heli_size = width;
    }
    
    cgalPoly_Polygon    heliPoly = gen_wgs84_area( GetLoc(), heli_size, 0, 0, heli_size, heading, false);
    cgalPoly_PolygonSet ps( heliPoly );
    
//    tgPolygonSet helipad;
        
    std::string heli_mat, extra_mat;
    if (surface == 1) {
        heli_mat  = "pa_heli";
        extra_mat = "pa_tiedown";
    } else {
        heli_mat  = "pc_heli";
        extra_mat = "pc_tiedown";
    }
    
    snprintf( desc, 256, "heli_%s", designator );
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, heli_mat, desc );
    
    meta.setTextureRef( heliPoly[0], heli_size, heli_size, heading );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_TPS_CLIPUV, -1.0l, -1.0l, 1.0l, 1.0l );
    meta.setTextureLimits( 0.0l, 0.0l, 1.0l, 1.0l );

    helipadPolys.push_back( tgPolygonSet(ps, meta ) );
    
    // Now generate the actual rectangle, and clip it with the square
    // need areference point at the origin of the direction, in the middle of width
    SGGeod ref = SGGeodesy::direct( SGGeod::fromDeg(lon, lat),
                                    SGMiscd::normalizePeriodic(0, 360, heading + 180 ),
                                    length/2 );
    
    cgalPoly_Polygon tiedown_area = gen_wgs84_rect( ref, heading, length, width );
    cgalPoly_PolygonSet ps2( tiedown_area );
    
    // Create the final output and push on to the runway super_polygon
    // list
    meta.setMaterial( extra_mat );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_TPS_NOCLIP );
    meta.setTextureRef( tiedown_area[0], 5.0l, 5.0l, heading );
    meta.setTextureLimits( 1.0l, 1.0l, 0.0l, 0.0l );
    snprintf( desc, 256, "heli_extra_%s", designator );
    
    helipadPolys.push_back( tgPolygonSet(ps2, meta) );
    
    // and build the shoulders 
    build_helipad_shoulders( tiedown_area );
    
    return helipadPolys;
}

tgPolygonSetList& Helipad::GetShoulderPolys( void )
{
    return shoulderPolys;
}

tgPolygonSetList& Helipad::GetInnerBasePolys( void )
{
    double    l, w;
    char      description[64];
    
    l = length;
    w = width;
    if ( (shoulder == 1) || (shoulder == 2 ) ) {
        l += 12.0;
        w += 12.0;
    } else {
        l += 2.0;
        w += 2.0;
    }
    
    cgalPoly_Polygon ib = gen_helipad_area_w_extend( l * 0.25, w * 0.25 );

    sprintf( description, "%s_inner_base", designator );
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, "Grass", description );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );

    // add this to the airport clearing
    innerBasePolys.push_back( tgPolygonSet( ib, meta ) );
    
    return innerBasePolys;
}

tgPolygonSetList& Helipad::GetOuterBasePolys( void )
{
    double    l, w;
    char      description[64];
    
    l = length;
    w = width;
    if ( (shoulder == 1) || (shoulder == 2 ) ) {
        l += 12.0;
        w += 12.0;
    } else {
        l += 2.0;
        w += 2.0;
    }
    
    cgalPoly_Polygon ob = gen_helipad_area_w_extend( l * 0.5, w * 0.5 );

    sprintf( description, "%s_outer_base", designator );
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, "Grass", description );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );
    
    // add this to the airport clearing
    outerBasePolys.push_back( tgPolygonSet( ob, meta ) );
    
    return outerBasePolys;
}

void Helipad::GetLights( tglightcontour_list& lights )
{
    if (edge_lights)
    {
        double heli_size;
        
        // helipad constructed as a square with width or length added around it
        if ( width == length ) {
            heli_size = length;
        } else if ( width > length ) {
            heli_size = length;
        } else {
            heli_size = width;
        }
        
        // Now generate the helipad lights
        tglightcontour_list s = gen_helipad_lights(heli_size);
        for ( unsigned int i = 0; i < s.size(); ++i ) {
            lights.push_back( s[i] );
        }
    }
}




#if 0
void Helipad::BuildBtg( tgpolygon_list& rwy_polys,
                        tglightcontour_list& rwy_lights,
                        tgcontour_list& slivers,
                        tgAccumulator& accum )
{
    //
    // Generate the basic helipad outlines
    //
    double heli_size;

    // helipad constructed as a square with width or length added around it
    if ( heli.width == heli.length ) {
        heli_size = heli.length;
    } else if ( heli.width > heli.length ) {
        heli_size = heli.length;
    } else {
        heli_size = heli.width;
    }

    tgContour helipad = gen_wgs84_area( GetLoc(), heli_size, 0, 0, heli_size, heli.heading, false);
    helipad = tgContour::Snap( helipad, gSnap );

    std::string heli_mat, extra_mat;
    if (heli.surface == 1) {
        heli_mat  = "pa_heli";
        extra_mat = "pa_tiedown";
    } else {
        heli_mat  = "pc_heli";
        extra_mat = "pc_tiedown";
    }

    // Clip the new polygon against what ever has already been created.
    tgPolygon clipped = accum.Diff( helipad );
    tgPolygon::RemoveSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    tgPolygon heli_poly = tgPolygon::SplitLongEdges( clipped, 400.0 );

    accum.Add( helipad );

    heli_poly.SetMaterial( heli_mat );
    heli_poly.SetTexParams( helipad.GetNode(0), heli_size, heli_size, heli.heading );
    heli_poly.SetTexLimits( 0,0,1,1 );
    heli_poly.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, -1.0, -1.0, 1.0, 1.0 );
    rwy_polys.push_back( heli_poly );

    // Now generate the actual rectangle, and clip it with the square
    // need areference point at the origin of the direction, in the middle of width
    SGGeod ref = SGGeodesy::direct( SGGeod::fromDeg(heli.lon, heli.lat),
                                    SGMiscd::normalizePeriodic(0, 360, heli.heading + 180 ),
                                    heli.length/2 );

    tgContour outer_area = gen_wgs84_rect( ref, heli.heading, heli.length, heli.width );
    outer_area = tgContour::Snap( outer_area, gSnap );

    tgPolygon outer_poly = tgContour::Diff( outer_area, heli_poly );
    clipped = accum.Diff( outer_poly );
    tgPolygon::RemoveSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    tgPolygon extra_heli = tgPolygon::SplitLongEdges( clipped, 400.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    extra_heli.SetMaterial( extra_mat );
    extra_heli.SetTexParams( outer_area.GetNode(0), 5.0, 5.0, heli.heading );
    extra_heli.SetTexLimits( 1,1,0,0 );
    extra_heli.SetTexMethod( TG_TEX_BY_TPS_NOCLIP );
    rwy_polys.push_back( extra_heli );
    accum.Add( extra_heli );

    build_helipad_shoulders( outer_area );

    if (heli.edge_lights)
    {
        // Now generate the helipad lights
        tglightcontour_list s = gen_helipad_lights(heli_size);
        for ( unsigned int i = 0; i < s.size(); ++i ) {
            rwy_lights.push_back( s[i] );
        }
    }
}

void Helipad::BuildBtg( tgpolygon_list& rwy_polys,
                        tglightcontour_list& rwy_lights,
                        tgcontour_list& slivers,
                        tgpolygon_list& apt_base_polys,
                        tgpolygon_list& apt_clearing_polys,
                        tgAccumulator& accum )
{
    BuildBtg( rwy_polys, rwy_lights, slivers, accum );

    // generate area around helipad
    double    length, width;
    tgContour base_contour, safe_base_contour;
    tgPolygon base, safe_base;

    length = heli.length;
    width  = heli.width;
    if ( (heli.shoulder == 1) || (heli.shoulder == 2 ) ) {
        length += 12.0;
        width += 12.0;
    } else {
        length += 2.0;
        width += 2.0;
    }

    base_contour = gen_helipad_area_w_extend(length * 0.25 , width * 0.25 );
    base_contour = tgContour::Snap( base_contour, gSnap );
    base.AddContour( base_contour );

    // also clear a safe area around the pad
    safe_base_contour = gen_helipad_area_w_extend( length * 0.5, width * 0.5 );
    safe_base_contour = tgContour::Snap( safe_base_contour, gSnap );
    safe_base.AddContour( safe_base_contour );

    // add this to the airport clearing
    apt_clearing_polys.push_back( safe_base );

    // and add the clearing to the base
    apt_base_polys.push_back( base );
}

void Helipad::BuildShoulder( tgpolygon_list& rwy_polys,
                             tgcontour_list& slivers,
                             tgAccumulator& accum )
{
    tgPolygon shoulder;

    for (unsigned int i=0; i<shoulder_polys.size(); i++) {
        shoulder = shoulder_polys[i];

        // Clip the new polygon against what ever has already been created.
        tgPolygon clipped = accum.Diff( shoulder );
        tgPolygon::RemoveSlivers( clipped, slivers );

        // Split long edges to create an object that can better flow with
        // the surface terrain
        tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );
        shoulder_polys[i] = split;

        rwy_polys.push_back( shoulder_polys[i] );

        accum.Add( shoulder );
    }
}
#endif