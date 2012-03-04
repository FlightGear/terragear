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

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>

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

superpoly_list Helipad::gen_helipad_lights(void){

    point_list c_lights; c_lights.clear();
    point_list c_normals; c_normals.clear();
    double lat2 = 0, lon2 = 0, az2;

    // Create a circle of lights approx. where the white circle is
    for (int deg=0; deg<360; deg+=10){
        geo_direct_wgs_84(0, heli.lat, heli.lon, deg ,
                          heli.width * 0.46 , &lat2, &lon2, &az2 );

        c_lights.push_back( Point3D( lon2, lat2, 0.0 ) );

        Point3D tmp = Point3D( lon2, lat2, 0.0 );
        Point3D vec = sgGeodToCart( tmp * SG_DEGREES_TO_RADIANS );
        double length = vec.distance3D( Point3D(0.0) );
        vec = vec / length;
        c_normals.push_back( vec );
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( c_lights, false );
    normals_poly.add_contour( c_normals, false );

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
    SG_LOG( SG_GENERAL, SG_INFO, "Building helipad = " << heli.designator );

    //
    // Generate the basic helipad outlines
    //
    Point3D helicenter = Point3D( heli.lon, heli.lat, 0.0);


    TGPolygon helipad = gen_wgs84_area( helicenter, heli.length, 0, 0, heli.width, heli.heading, false);

    double start1_pct = 0.0;
    double end1_pct = 0.0;
    double maxsize = heli.width - heli.length;

    if (maxsize <= 0)
        maxsize = heli.width;
    else if (maxsize > 0)
        maxsize = heli.length;

    double percent = (maxsize / heli.length) /2;

    start1_pct = 0.5 - percent;
    end1_pct = 0.5 + percent;
    gen_tex_section( helipad,
                        start1_pct, end1_pct,
                        0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
                        heli.heading, heli.width, heli.length,
                        "pc_", "heli",
                        rwy_polys, texparams, accum, slivers );


    // generate area around helipad
    if (apt_base)
    {
        TGPolygon base, safe_base;
        base      = gen_runway_area_w_extend( 0.0, maxsize * 0.25 , 0.0, 0.0, maxsize * 0.25 );

        // also clear a safe area around the pad
        safe_base = gen_runway_area_w_extend( 0.0, maxsize * 0.5, 0.0, 0.0, maxsize * 0.5 );

        // add this to the airport clearing
        *apt_clearing = tgPolygonUnionClipper(safe_base, *apt_clearing);

        // and add the clearing to the base
        *apt_base = tgPolygonUnionClipper( base, *apt_base );
    }

    if (heli.edge_lights)
    {
        // Now generate the helipad lights
        superpoly_list s = gen_helipad_lights();
        for ( unsigned int i = 0; i < s.size(); ++i ) {
            rwy_lights->push_back( s[i] );
        }
    }
}
