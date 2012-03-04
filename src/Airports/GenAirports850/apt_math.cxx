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

#include "apt_math.hxx"
#include "global.hxx"

#include <stdlib.h>

using std::string;

TGPolygon gen_wgs84_area( Point3D origin,
                                    double length_m,
                                    double displ1, double displ2,
                                    double width_m,
                                    double heading_deg,
                                    double alt_m,
                                    bool   add_mid )
{
    TGPolygon result_list;
    double length_hdg = heading_deg;
    double left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    // move to the +l end/center of the runway
    Point3D ref = origin;
    double lon = 0, lat = 0, r = 0;
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg,
                        length_m / 2.0 - displ2, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the l,-w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    Point3D p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the l,w corner
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,w point (then we add points in a clockwise direction)

        ref = origin;
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                            width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    // move to the -l end/center of the runway
    ref = origin;
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg,
                        displ1 - length_m/2.0, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the -l,w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the -l,-w corner
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,-w point (then we add points in a clockwise direction)

        ref = origin;
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                            -width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    return result_list;
}

TGPolygon gen_wgs84_area( Point3D end1, Point3D end2,
                          double length_m,
                          double displ1, double displ2,
                          double width_m,
                          double heading_deg,
                          double alt_m,
                          bool   add_mid )
{
    TGPolygon result_list;
    double left_hdg = heading_deg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    // move from end2 to the displaced threshold
    Point3D ref = end2;
    double lon = 0, lat = 0, r = 0;
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), heading_deg,
                        length_m / 2.0 - displ2, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the l,-w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    Point3D p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the l,w corner
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,w point (then we add points in a clockwise direction)

        ref = Point3D( (end1.lon()+end2.lon())/2.0f, (end1.lat()+end2.lat())/2.0f, 0.0f);
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                            width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    // move to the end1 center to the displ. threshold
    ref = end1;
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), heading_deg,
                        displ1 - length_m / 2.0, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the -l,w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the -l,-w corner
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,-w point (then we add points in a clockwise direction)

        ref = Point3D( (end1.lon()+end2.lon())/2.0f, (end1.lat()+end2.lat())/2.0f, 0.0f);
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                            -width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    return result_list;
}

TGPolygon gen_wgs84_rect( double lat, double lon, double heading, double length, double width )
{
    TGPolygon result_list;
    double ptlat = 0.0f;
    double ptlon = 0.0f;
    double r     = 0.0f;
    Point3D p;

    // starting point is in the middle of the rectangle width, at the beginning - stretch to heading
    
    // Point 1 is -90deg, 1/2 width away
    double left_hdg = heading -90;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    geo_direct_wgs_84 ( 0.0, lat, lon, left_hdg, width / 2.0, &ptlat, &ptlon, &r );
    p = Point3D( ptlon, ptlat, 0.0 );
    result_list.add_node( 0, p );

    // Point 2 is heading, length away from point 1
    geo_direct_wgs_84 ( 0.0, ptlat, ptlon, heading, length, &ptlat, &ptlon, &r );
    p = Point3D( ptlon, ptlat, 0.0 );
    result_list.add_node( 0, p );

    // Point 3 is -90deg, -width away from point 2
    geo_direct_wgs_84 ( 0.0, ptlat, ptlon, left_hdg, -width, &ptlat, &ptlon, &r );
    p = Point3D( ptlon, ptlat, 0.0 );
    result_list.add_node( 0, p );

    // last point is heading, -length from point 3
    geo_direct_wgs_84 ( 0.0, ptlat, ptlon, heading, -length, &ptlat, &ptlon, &r );
    p = Point3D( ptlon, ptlat, 0.0 );
    result_list.add_node( 0, p );

    return result_list;
}

// generate a section of texture
void gen_tex_section( const TGPolygon& runway,
                         double startl_pct, double endl_pct,
                         double startw_pct, double endw_pct,
                         double minu, double maxu, double minv, double maxv,
                         double heading, double width, double length,
                         const string& prefix,
                         const string& material,
                         superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         ClipPolyType *accum,
                         poly_list& slivers ) {

    int j, k;

    Point3D a0 = runway.get_pt(0, 1);
    Point3D a1 = runway.get_pt(0, 2);
    Point3D a2 = runway.get_pt(0, 0);
    Point3D a3 = runway.get_pt(0, 3);

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

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "pre clipped runway pts " << prefix << material);
    for ( j = 0; j < section.contours(); ++j ) {
        for ( k = 0; k < section.contour_size( j ); ++k ) {
            Point3D p = section.get_pt(j, k);
            SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
        }
    }

    // Clip the new polygon against what ever has already been created.
#if 0
    TGPolygon clipped = tgPolygonDiff( section, *accum );
#else
    TGPolygon clipped = tgPolygonDiffClipper( section, *accum );
#endif

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

#if 0
    *accum = tgPolygonUnion( section, *accum );
#else
    *accum = tgPolygonUnionClipper( section, *accum );
#endif

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
