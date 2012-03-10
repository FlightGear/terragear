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

// generate a section of texture
void Helipad::gen_helipad( const TGPolygon& runway,
                          double startl_pct, double endl_pct,
                          double startw_pct, double endw_pct,
                          double minu, double maxu, double minv, double maxv,
                          double heading,
                          const string& prefix,
                          const string& material,
                          superpoly_list *rwy_polys,
                          texparams_list *texparams,
                          ClipPolyType *accum,
                          poly_list& slivers ) 
{
    int j, k;
    double width = heli.width;
    double length = heli.length;

    Point3D a0 = runway.get_pt(0, 1);
    Point3D a1 = runway.get_pt(0, 2);
    Point3D a2 = runway.get_pt(0, 0);
    Point3D a3 = runway.get_pt(0, 3);

#if 0
    if ( startl_pct > 0.0 ) {
        startl_pct -= nudge * SG_EPSILON;
    }
    if ( endl_pct < 1.0 ) {
        endl_pct += nudge * SG_EPSILON;
    }
#endif

    if ( startl_pct < 0.0 ) {
        startl_pct = 0.0;
    }

    if ( endl_pct > 1.0 ) {
        endl_pct = 1.0;
    }

    // partial "w" percentages could introduce "T" intersections which
    // we compensate for later, but could still cause problems now
    // with our polygon clipping code.  This attempts to compensate
    // for that by nudging the areas a bit bigger so we don't end up
    // with polygon slivers.
#if 0
    if ( startw_pct > 0.0 || endw_pct < 1.0 ) {
        if ( startw_pct > 0.0 ) {
            startw_pct -= nudge * SG_EPSILON;
        }
        if ( endw_pct < 1.0 ) {
            endw_pct += nudge * SG_EPSILON;
        }
    }
#endif

    SG_LOG(SG_GENERAL, SG_DEBUG, "start len % = " << startl_pct
           << " end len % = " << endl_pct);

    double dlx, dly;

    dlx = a1.x() - a0.x();
    dly = a1.y() - a0.y();

    Point3D t0 = Point3D( a0.x() + dlx * startl_pct,
                          a0.y() + dly * startl_pct, 0);
    Point3D t1 = Point3D( a0.x() + dlx * endl_pct,
                          a0.y() + dly * endl_pct, 0);

    dlx = a3.x() - a2.x();
    dly = a3.y() - a2.y();

    Point3D t2 = Point3D( a2.x() + dlx * startl_pct,
                          a2.y() + dly * startl_pct, 0);

    Point3D t3 = Point3D( a2.x() + dlx * endl_pct,
                          a2.y() + dly * endl_pct, 0);

    SG_LOG(SG_GENERAL, SG_DEBUG, "start wid % = " << startw_pct
           << " end wid % = " << endw_pct);

    double dwx, dwy;

    dwx = t0.x() - t2.x();
    dwy = t0.y() - t2.y();

    Point3D p0 = Point3D( t2.x() + dwx * startw_pct,
                          t2.y() + dwy * startw_pct, 0);

    Point3D p1 = Point3D( t2.x() + dwx * endw_pct,
                          t2.y() + dwy * endw_pct, 0);

    dwx = t1.x() - t3.x();
    dwy = t1.y() - t3.y();

    Point3D p2 = Point3D( t3.x() + dwx * startw_pct,
                          t3.y() + dwy * startw_pct, 0);

    Point3D p3 = Point3D( t3.x() + dwx * endw_pct,
                          t3.y() + dwy * endw_pct, 0);

    TGPolygon section;
    section.erase();

    section.add_node( 0, p2 );
    section.add_node( 0, p0 );
    section.add_node( 0, p1 );
    section.add_node( 0, p3 );
    section = snap( section, gSnap );

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "pre clipped runway pts " << prefix << material);
    for ( j = 0; j < section.contours(); ++j ) {
        for ( k = 0; k < section.contour_size( j ); ++k ) {
            Point3D p = section.get_pt(j, k);
            SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
        }
    }

    // Clip the new polygon against what ever has already been created.
    TGPolygon clipped = tgPolygonDiffClipper( section, *accum );
    tgPolygonFindSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    TGPolygon split = tgPolygonSplitLongEdges( clipped, 400.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    TGSuperPoly sp;
    sp.erase();
    sp.set_poly( split );
    sp.set_material( prefix + material );
    rwy_polys->push_back( sp );
    SG_LOG(SG_GENERAL, SG_DEBUG, "section = " << clipped.contours());

    *accum = tgPolygonUnionClipper( section, *accum );

    // Store away what we need to know for texture coordinate
    // calculation.  (CLO 10/20/02: why can't we calculate texture
    // coordinates here?  Oh, becuase later we need to massage the
    // polygons to avoid "T" intersections and clean up other
    // potential artifacts and we may add or remove points and need to
    // do new texture coordinate calcs later.

    double len = length / 2.0;
    double sect_len = len * ( endl_pct - startl_pct );

    double sect_wid = width * ( endw_pct - startw_pct );

    TGTexParams tp;
    tp = TGTexParams( p0,
                      sect_wid,
                      sect_len,
                      heading );
    tp.set_minu( minu );
    tp.set_maxu( maxu );
    tp.set_minv( minv );
    tp.set_maxv( maxv );
    texparams->push_back( tp );

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped runway pts " << prefix + material);
    for ( j = 0; j < clipped.contours(); ++j ) {
        for ( k = 0; k < clipped.contour_size( j ); ++k ) {
            Point3D p = clipped.get_pt(j, k);
            SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
        }
    }
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
    gen_helipad( helipad,
                 start1_pct, end1_pct,
                 0.0, 1.0,
                 0.0, 1.0, 0.0, 1.0,
                 heli.heading,
                 "pc_", "heli",
                 rwy_polys, texparams, accum, slivers );


    // generate area around helipad
    if (apt_base)
    {
        TGPolygon base, safe_base;
        base      = gen_runway_area_w_extend( 0.0, maxsize * 0.25 , 0.0, 0.0, maxsize * 0.25 );
        base      = snap( base, gSnap ); 

        // also clear a safe area around the pad
        safe_base = gen_runway_area_w_extend( 0.0, maxsize * 0.5, 0.0, 0.0, maxsize * 0.5 );
        safe_base = snap( safe_base, gSnap );

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
