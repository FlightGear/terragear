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

#include <Geometry/poly_support.hxx>

#include "global.hxx"
#include "apt_math.hxx"
#include "helipad.hxx"
#include "runway.hxx"

#include <stdlib.h>

Helipad::Helipad(char* definition)
{

    // format:
    // helipad  designator   lat  lon  heading      length   width  surface     markings    shoulder    smoothness    edge lighting

    // int fscanf(FILE *stream, const char *format, ...);
    sscanf(definition, "%s %lf %lf %lf %lf %lf %d %d %d %lf %d",
        heli.designator, &heli.lat, &heli.lon, &heli.heading, &heli.length, &heli.width, &heli.surface,
        &heli.marking, &heli.shoulder, &heli.smoothness, &heli.edge_lights);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read helipad: (" << heli.lon << "," << heli.lat << ") heading: " << heli.heading << " length: " << heli.length << " width: " << heli.width );
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

tgPolygon Helipad::WriteGeom( const tgContour& area, string material,
                              tgpolygon_list& rwy_polys,
                              tgcontour_list& slivers )
{
    // Clip the new polygon against what ever has already been created.
    tgPolygon clipped = tgContour::DiffWithAccumulator( area );
    tgPolygon::RemoveSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    split.SetMaterial( material );

    tgContour::AddToAccumulator( area );

    return split;
}

void Helipad::BuildBtg( tgpolygon_list& rwy_polys,
                        tglightcontour_list& rwy_lights,
                        tgcontour_list& slivers )
{
    //
    // Generate the basic helipad outlines
    //

    double maxsize = heli.width - heli.length;
    bool area_top = false;
    bool area_side = false;
    if (maxsize == 0) {
        maxsize = heli.width;
    } else if (maxsize < 0) {
        maxsize = heli.width;
        area_top = true;
    } else {
        maxsize = heli.length;
        area_side = true;
    }

    tgContour helipad = gen_wgs84_area( GetLoc(), maxsize, 0, 0, maxsize, heli.heading, false);
    helipad = tgContour::Snap( helipad, gSnap );
    string material, shoulder_mat;
    if (heli.surface == 1)
        material = "pa_";
    else
        material = "pc_";

    // write out
    tgPolygon result = WriteGeom( helipad, material + "heli", rwy_polys, slivers);
    result.SetMaterial( material + "heli" );
    result.SetTexParams( helipad.GetNode(0), maxsize, maxsize, heli.heading );
    result.SetTexLimits( 0,0,1,1 );
    result.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, -1.0, -1.0, 1.0, 1.0 );

    rwy_polys.push_back( result );

    int i = 0;
    double heading = 0, areahight = 0;
    tgContour heli_area = gen_helipad_area_w_extend(0.0, 0.0);
    heli_area = tgContour::Snap( heli_area, gSnap );

    tgcontour_list  area_contours;
    tgContour       area;

    if (area_top || area_side) {

        if (area_top) {
            areahight = (heli.length - maxsize) /2;
            heading = SGMiscd::normalizePeriodic( 0, 360, heli.heading-90 );
            i = 0;
        } else {
            areahight = (heli.width - maxsize) /2;
            heading = SGMiscd::normalizePeriodic( 0, 360, heli.heading-90 );
            i = 1;
        }

        for (;i<4; ++i) {
            area.Erase();
            area.AddNode( heli_area.GetNode( i ) );
            area.AddNode( heli_area.GetNode( i == 3 ? 0 : i+1 ) );
            area.AddNode( helipad.GetNode( i == 3 ? 0 : i+1) );
            area.AddNode( helipad.GetNode( i ) );
            area.SetHole( false );
            area_contours.push_back( area );
            i++;
        }

        tgPolygon result;
        for (i = 0; i < 2; ++i) {
            result = WriteGeom( area_contours[i], material + "tiedown", rwy_polys, slivers);
            result.SetTexParams( area_contours[i].GetNode(0), maxsize, areahight, heading );
            result.SetTexLimits( 1,1,0,0 );
            result.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, 0.0, 0.0, 1.0, 1.0 );
            rwy_polys.push_back( result );
            heading = SGMiscd::normalizePeriodic(0, 360, heading + 180 );
        }
    }

    if (heli.shoulder == 1) {
        shoulder_mat = "pa_shoulder";
        areahight = 6; // shoulder size in m
    } else if (heli.shoulder == 2) {
        shoulder_mat = "pc_shoulder";
        areahight = 6; // shoulder size in m
    } else {
        shoulder_mat = material + "shoulder_f";
        areahight = 1; // fake shoulder size in m
    }

    double shoulder_width = heli.length;
    heading = heli.heading;

    if (area_side) {
        shoulder_width = heli.width;
    }

    tgContour shoulder = gen_helipad_area_w_extend(areahight, areahight);
    shoulder = tgContour::Snap( shoulder, gSnap );

    for (i = 0; i < 4; ++i) {
        heading = SGMiscd::normalizePeriodic(0,360,heading-90);
        area.Erase();
        area.AddNode( shoulder.GetNode( i ) );
        area.AddNode( shoulder.GetNode( i == 3 ? 0 : i+1 ) );
        area.AddNode( heli_area.GetNode( i == 3 ? 0 : i+1 ) );
        area.AddNode( heli_area.GetNode( i ) );
        area.SetHole(false);

        result.Erase();
        result.AddContour( area );
        result.SetMaterial( shoulder_mat );
        result.SetTexParams( area.GetNode(1), areahight, shoulder_width, heading );
        result.SetTexLimits( 1,1,0,0 );
        result.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, 0.0, 0.0, 1.0, 1.0 );
        shoulder_polys.push_back( result );
    }

    if (heli.edge_lights)
    {
        // Now generate the helipad lights
        tglightcontour_list s = gen_helipad_lights(maxsize);
        for ( unsigned int i = 0; i < s.size(); ++i ) {
            rwy_lights.push_back( s[i] );
        }
    }
}

void Helipad::BuildBtg( tgpolygon_list& rwy_polys,
                        tglightcontour_list& rwy_lights,
                        tgcontour_list& slivers,
                        tgPolygon& apt_base,
                        tgPolygon& apt_clearing )
{
    BuildBtg( rwy_polys, rwy_lights, slivers );

    // generate area around helipad
    double    length, width;
    tgContour base, safe_base;

    length = heli.length;
    width  = heli.width;
    if ( (heli.shoulder == 1) || (heli.shoulder == 2 ) ) {
        length += 12.0;
        width += 12.0;
    } else {
        length += 2.0;
        width += 2.0;
    }

    base      = gen_helipad_area_w_extend(length * 0.25 , width * 0.25 );
    base      = tgContour::Snap( base, gSnap );

    // also clear a safe area around the pad
    safe_base = gen_helipad_area_w_extend( length * 0.5, width * 0.5 );
    safe_base = tgContour::Snap( safe_base, gSnap );

    // add this to the airport clearing
    apt_clearing = tgPolygon::Union(safe_base, apt_clearing);

    // and add the clearing to the base
    apt_base = tgPolygon::Union( base, apt_base );
}

void Helipad::BuildShoulder( tgpolygon_list& rwy_polys,
                             tgcontour_list& slivers,
                             tgPolygon& apt_base,
                             tgPolygon& apt_clearing )
{
    tgPolygon shoulder;

    for (unsigned int i=0; i<shoulder_polys.size(); i++) {
        shoulder = shoulder_polys[i];

        // Clip the new polygon against what ever has already been created.
        tgPolygon clipped = tgPolygon::DiffWithAccumulator( shoulder );
        tgPolygon::RemoveSlivers( clipped, slivers );

        // Split long edges to create an object that can better flow with
        // the surface terrain
        tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );
        shoulder_polys[i] = split;

        rwy_polys.push_back( shoulder_polys[i] );

        tgPolygon::AddToAccumulator( shoulder );
    }

    // base and clearing calculated when generating the shoulders
}

void Helipad::BuildShoulder( tgpolygon_list& rwy_polys,
                             tgcontour_list& slivers )
{
    tgPolygon shoulder;

    for (unsigned int i=0; i<shoulder_polys.size(); i++) {
        shoulder = shoulder_polys[i];

        // Clip the new polygon against what ever has already been created.
        tgPolygon clipped = tgPolygon::DiffWithAccumulator( shoulder );
        tgPolygon::RemoveSlivers( clipped, slivers );

        // Split long edges to create an object that can better flow with
        // the surface terrain
        tgPolygon split = tgPolygon::SplitLongEdges( clipped, 400.0 );
        shoulder_polys[i] = split;

        rwy_polys.push_back( shoulder_polys[i] );

        tgPolygon::AddToAccumulator( shoulder );
    }
}
