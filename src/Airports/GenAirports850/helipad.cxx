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

    point_list y_lights; y_lights.clear();
    point_list y_normals; y_normals.clear();

    // Vector calculation
    SGVec3d vec = normalize(SGVec3d::fromGeod(GetLoc()));

    // Create yellow edge lights, 5m spacing
    int divs = (int)(maxsize / 5.0);
    TGPolygon area = gen_runway_area_w_extend(0.0, 0.0, 0.0, 0.0, 0.0);
    Point3D pt, inc;

    for ( int i = 0; i < area.contour_size( 0 ); ++i ) {
        pt = area.get_pt( 0, i );
        inc = (area.get_pt(0, i==3 ? 0 : i+1) - area.get_pt(0,i)) / divs;
        for ( int j = 0; j < divs; ++j) {
            y_lights.push_back( pt);
            y_normals.push_back(Point3D::fromSGVec3(vec));
            pt += inc;
        }
    }

    // Create a circle of yellow lights where the white texture circle is
    for (int deg = 0; deg < 360; deg += 45){
        y_lights.push_back( Point3D::fromSGGeod( SGGeodesy::direct(GetLoc(), deg, maxsize * 0.46)) );
        y_normals.push_back( Point3D::fromSGVec3(vec) );
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( y_lights, false );
    normals_poly.add_contour( y_normals, false );

    TGSuperPoly yellow;
    yellow.set_poly( lights_poly );
    yellow.set_normals( normals_poly );
    yellow.set_material( "RWY_YELLOW_LIGHTS" );

    superpoly_list result; result.clear();

    result.push_back( yellow );

    return result;
}

void Helipad::WriteGeom( TGPolygon polygon, string material,
                        superpoly_list *rwy_polys,
                        ClipPolyType *accum,
                        poly_list& slivers )
{
    // Clip the new polygon against what ever has already been created.
    TGPolygon clipped = tgPolygonDiffClipper( polygon, *accum );
    tgPolygonFindSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    TGPolygon split = tgPolygonSplitLongEdges( clipped, 400.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    TGSuperPoly sp;
    sp.erase();
    sp.set_poly( split );
    sp.set_material( material );
    rwy_polys->push_back( sp );

    *accum = tgPolygonUnionClipper( polygon, *accum );
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

    TGPolygon helipad = gen_wgs84_area( GetLoc(), maxsize, 0, 0, maxsize, heli.heading, false);
    helipad = snap( helipad, gSnap );
    string material, shoulder_mat;
    if (heli.surface == 1)
        material = "pa_";
    else
        material = "pc_";
    // write out
    WriteGeom( helipad, material + "heli", rwy_polys, accum, slivers);

    TGTexParams tp;
    tp = TGTexParams( helipad.get_pt(0,0).toSGGeod(), maxsize, maxsize, heli.heading );
    tp.set_minu( 1 );
    tp.set_maxu( 0 );
    tp.set_minv( 1 );
    tp.set_maxv( 0 );
    texparams->push_back( tp );
    int i = 0;
    double heading = 0, areahight = 0;
    TGPolygon heli_area = gen_runway_area_w_extend(0.0, 0.0, 0.0, 0.0, 0.0);
    heli_area = snap( heli_area, gSnap );
    TGPolygon area_poly; area_poly.erase();
    point_list area; area.clear();

    if (area_top || area_side) {

        if (area_top) {
            areahight = (heli.length - maxsize) /2;
            heading = heli.heading;
            i = 0;
        } else {
            areahight = (heli.width - maxsize) /2;
            heading = heli.heading - 90;
            if ( heading < 0.0 ) { heading += 360.0; }
            i = 1;
        }

        for (;i<4; ++i) {
            area.push_back( heli_area.get_pt( 0,i ) );
            area.push_back( heli_area.get_pt( 0,i == 3? 0 : i+1 ) );
            area.push_back( helipad.get_pt( 0,i == 3? 0 : i+1) );
            area.push_back( helipad.get_pt( 0,i ) );
            area_poly.add_contour( area, false );
            area.clear();
            i++;
        }

        TGPolygon area_geom;
        for (i = 0; i < 2; ++i) {
            area_geom.add_contour(area_poly.get_contour(i), false);
            WriteGeom( area_geom, material + "tiedown", rwy_polys, accum, slivers);

            tp = TGTexParams( area_poly.get_pt(i,0).toSGGeod(), maxsize, areahight, heading );
            tp.set_minu( 1 );
            tp.set_maxu( 0 );
            tp.set_minv( 1 );
            tp.set_maxv( 0 );
            texparams->push_back( tp );
            area_geom.erase();
            heading +=  180;
            if ( heading > 360.0 ) { heading -= 360.0; }
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

    TGSuperPoly sp;
    double shoulder_width = heli.length;
    heading = heli.heading;

    if (area_side) {
        shoulder_width = heli.width;
    }

    TGPolygon shoulder = gen_runway_area_w_extend(0.0, areahight, 0.0, 0.0, areahight);
    shoulder = snap( shoulder, gSnap );

    for (i = 0; i < 4; ++i) {
        heading -= 90;
        if ( heading < 0.0 ) { heading += 360.0; }
        area_poly.erase();
        area.clear();

        area.push_back( shoulder.get_pt( 0,i ) );
        area.push_back( shoulder.get_pt( 0,i == 3? 0 : i+1 ) );
        area.push_back( heli_area.get_pt( 0,i == 3? 0 : i+1 ) );
        area.push_back( heli_area.get_pt( 0,i ) );
        area_poly.add_contour( area, false );

        sp.erase();
        sp.set_poly( area_poly );
        sp.set_material( shoulder_mat );
        sp.set_flag( "heli-shoulder" );
        shoulder_polys.push_back( sp );

        tp = TGTexParams( area_poly.get_pt(0,1).toSGGeod(), areahight, shoulder_width, heading );
        tp.set_minu( 1 );
        tp.set_maxu( 0 );
        tp.set_minv( 1 );
        tp.set_maxv( 0 );
        shoulder_tps.push_back( tp );
    }

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

void Helipad::BuildShoulder( superpoly_list *rwy_polys,
                             texparams_list *texparams,
                             ClipPolyType *accum,
                             poly_list& slivers,
                             TGPolygon* apt_base,
                             TGPolygon* apt_clearing )
{
    TGPolygon base, safe_base;
    TGPolygon shoulder;

    for (unsigned int i=0; i<shoulder_polys.size(); i++) {
        shoulder = shoulder_polys[i].get_poly();

        // Clip the new polygon against what ever has already been created.
        TGPolygon clipped = tgPolygonDiffClipper( shoulder, *accum );
        tgPolygonFindSlivers( clipped, slivers );

        // Split long edges to create an object that can better flow with
        // the surface terrain
        TGPolygon split = tgPolygonSplitLongEdges( clipped, 400.0 );
        shoulder_polys[i].set_poly( split );

        rwy_polys->push_back( shoulder_polys[i] );
        texparams->push_back( shoulder_tps[i] );

        *accum = tgPolygonUnionClipper( shoulder, *accum );

        if (apt_base)
        {
            // also clear a safe area around the runway
            base      = tgPolygonExpand( shoulder, 20.0);
            safe_base = tgPolygonExpand( shoulder, 50.0);

            // add this to the airport clearing
            *apt_clearing = tgPolygonUnionClipper( safe_base, *apt_clearing );

            // and add the clearing to the base
            *apt_base = tgPolygonUnionClipper( base, *apt_base );
        }
    }
}
