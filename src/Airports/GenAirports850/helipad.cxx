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

superpoly_list Helipad::gen_helipad_lights(double maxsize){

    point_list g_lights; g_lights.clear();
    point_list g_normals; g_normals.clear();

    //vector calculation
    Point3D vec = sgGeodToCart( GetLoc() * SG_DEGREES_TO_RADIANS );
    double length = vec.distance3D( Point3D(0.0) );
    vec = vec / length;

    // Create green edge lights
    TGPolygon area = gen_runway_area_w_extend(0.0, 0.0, 0.0, 0.0, 0.0);
    for ( int i = 0; i < area.contour_size( 0 ); ++i ) {
        g_lights.push_back( area.get_pt( 0, i ) );
        g_normals.push_back( vec );
    }

    // Create a circle of lights approx. where the white texture circle is
    double lat = heli.lat, lon = heli.lon, az;
    for (int deg = 0; deg < 360; deg += 10){
        geo_direct_wgs_84(0, heli.lat, heli.lon, deg ,
                          maxsize * 0.46 , &lat, &lon, &az );

        g_lights.push_back( Point3D( lon, lat, 0.0 ) );
        g_normals.push_back( vec );
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( g_lights, false );
    normals_poly.add_contour( g_normals, false );

    TGSuperPoly green;
    green.set_poly( lights_poly );
    green.set_normals( normals_poly );
    green.set_material( "RWY_GREEN_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( green );

    return result;
}

void Helipad::BuildBtg( superpoly_list *rwy_polys,
                        texparams_list *texparams,
                        superpoly_list *rwy_lights,
                        ClipPolyType *accum, 
                        poly_list& slivers, 
                        TGPolygon* apt_base, 
                        TGPolygon* apt_clearing )
{
    //
    // Generate the basic helipad outlines
    //

    double maxsize = heli.width - heli.length;
    if (maxsize <= 0)
        maxsize = heli.width;
    else
        maxsize = heli.length;

    TGPolygon helipad = gen_wgs84_area( GetLoc(), maxsize, 0, 0, maxsize, heli.heading, false);
    helipad = snap( helipad, gSnap );

    string material;
    if (heli.surface == 1)
        material = "pa_";
    else
        material = "pc_";

    // Clip the new polygon against what ever has already been created.
    TGPolygon clipped = tgPolygonDiffClipper( helipad, *accum );
    tgPolygonFindSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    TGPolygon split = tgPolygonSplitLongEdges( clipped, 400.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    TGSuperPoly sp;
    sp.erase();
    sp.set_poly( split );
    sp.set_material( material + "heli" );
    rwy_polys->push_back( sp );

    *accum = tgPolygonUnionClipper( helipad, *accum );

    TGTexParams tp;
    tp = TGTexParams( helipad.get_pt(0,0), maxsize, maxsize, heli.heading );
    tp.set_minu( 1 );
    tp.set_maxu( 0 );
    tp.set_minv( 1 );
    tp.set_maxv( 0 );
    texparams->push_back( tp );


    // generate area around helipad
    if (apt_base)
    {
        TGPolygon base, safe_base;
        base      = gen_runway_area_w_extend( 0.0, heli.length * 0.25 , 0.0, 0.0, heli.width * 0.25 );
        base      = snap( base, gSnap ); 

        // also clear a safe area around the pad
        safe_base = gen_runway_area_w_extend( 0.0, heli.length * 0.5, 0.0, 0.0, heli.width * 0.5 );
        safe_base = snap( safe_base, gSnap );

        // add this to the airport clearing
        *apt_clearing = tgPolygonUnionClipper(safe_base, *apt_clearing);

        // and add the clearing to the base
        *apt_base = tgPolygonUnionClipper( base, *apt_base );
    }

    if (heli.edge_lights)
    {
        // Now generate the helipad lights
        superpoly_list s = gen_helipad_lights(maxsize);
        for ( unsigned int i = 0; i < s.size(); ++i ) {
            rwy_lights->push_back( s[i] );
        }
    }
}
