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


Helipad::Helipad(char* definition)
{
    // helipad format:
    //      designator   lat  lon  heading      length   width  surface     markings    shoulder    smoothness    edge-lighting
    // example:
    //      "H1  21.30433555 -157.85586778   0.00 10.70 10.70 2 0 0 0.25 0\r"

    std::istringstream ss(definition);
    ss  >> heli.designator
        >> heli.lat
        >> heli.lon
        >> heli.heading
        >> heli.length
        >> heli.width
        >> heli.surface
        >> heli.marking
        >> heli.shoulder
        >> heli.smoothness
        >> heli.edge_lights;

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read helipad: (" << heli.lon << "," << heli.lat << ") heading: " << heli.heading << " length: " << heli.length << " width: " << heli.width );
}

tglightcontour_list Helipad::gen_helipad_lights(double maxsize) {
    tglightcontour_list result;

    // Vector calculation
    SGVec3f vec = normalize(SGVec3f::fromGeod(GetLoc()));

    // Create yellow edge lights, 5m spacing
    tgContour area = gen_helipad_area_w_extend(0.0, 0.0);
    tgLightContour yellow;

    yellow.SetType( "RWY_YELLOW_LIGHTS" );
    for ( unsigned int i = 0; i < area.GetSize(); ++i ) {
        double dist, course, cs;
        SGGeodesy::inverse(area.GetNode(i), area.GetNode(i==3 ? 0 : i+1), course, cs, dist );
        int divs = (int)(dist / 5.0);
        double step = dist/divs;
        SGGeod pt = area.GetNode(i);
        for (int j = 0; j < divs; ++j) {
            pt = SGGeodesy::direct(pt, course, step );
            yellow.AddLight( pt, vec );
        }
    }

    // Create a circle of yellow lights where the white texture circle is
    for (int deg = 0; deg < 360; deg += 45){
        yellow.AddLight( SGGeodesy::direct(GetLoc(), deg, maxsize * 0.46), vec );
    }

    result.push_back( yellow );

    return result;
}

void Helipad::build_helipad_shoulders( const tgContour& outer_area )
{
    double shoulder_width, shoulder_heading;
    std::string shoulder_mat;

    // Now build the shoulders
    if (heli.shoulder == 1) {
        shoulder_mat = "pa_shoulder";
        shoulder_width = 6; // shoulder size in m
    } else if (heli.shoulder == 2) {
        shoulder_mat = "pc_shoulder";
        shoulder_width = 6; // shoulder size in m
    } else if (heli.surface == 1) {
        shoulder_mat  = "pa_shoulder_f";
        shoulder_width = 1; // shoulder size in m
    } else {
        shoulder_mat  = "pc_shoulder_f";
        shoulder_width = 1; // shoulder size in m
    }
    shoulder_heading = heli.heading;

    tgContour shoulder = gen_helipad_area_w_extend(shoulder_width, shoulder_width);
    tgPolygon shoulder_poly;

    for (int i = 0; i < 4; ++i) {
        shoulder_heading = SGMiscd::normalizePeriodic(0,360,shoulder_heading-90);
        shoulder_poly.Erase();
        shoulder_poly.AddNode( 0, shoulder.GetNode( i ) );
        shoulder_poly.AddNode( 0, shoulder.GetNode( i == 3 ? 0 : i+1 ) );
        shoulder_poly.AddNode( 0, outer_area.GetNode( i == 3 ? 0 : i+1 ) );
        shoulder_poly.AddNode( 0, outer_area.GetNode( i ) );
        shoulder_poly.SetMaterial( shoulder_mat );
        shoulder_poly.SetTexParams( shoulder_poly.GetNode(0, 1), shoulder_width, 5.0, shoulder_heading );
        shoulder_poly.SetTexLimits( 1,1,0,0 );
        shoulder_poly.SetTexMethod( TG_TEX_BY_TPS_CLIPU, 0.0, 0.0, 1.0, 1.0 );
        shoulder_poly = tgPolygon::Snap( shoulder_poly, gSnap );
        shoulder_polys.push_back( shoulder_poly );
    }
}

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
    // tgPolygon::RemoveSlivers( clipped, slivers );

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
    // tgPolygon::RemoveSlivers( clipped, slivers );

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
        // tgPolygon::RemoveSlivers( clipped, slivers );

        // Split long edges to create an object that can better flow with
        // the surface terrain
        tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );
        shoulder_polys[i] = split;

        rwy_polys.push_back( shoulder_polys[i] );

        accum.Add( shoulder );
    }
}
