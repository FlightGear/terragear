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
                                    bool   add_mid )
{
    TGPolygon result_list;
    double length_hdg = heading_deg;
    double left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    // move to the +l end/center of the runway
    Point3D ref = origin;
    double lon = 0, lat = 0, r = 0;
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), length_hdg,
                        length_m / 2.0 - displ2, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the l,-w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    Point3D p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the l,w corner
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,w point (then we add points in a clockwise direction)

        ref = origin;
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                            width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    // move to the -l end/center of the runway
    ref = origin;
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), length_hdg,
                        displ1 - length_m/2.0, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the -l,w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the -l,-w corner
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,-w point (then we add points in a clockwise direction)

        ref = origin;
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
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
                          bool   add_mid )
{
    TGPolygon result_list;
    double left_hdg = heading_deg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    // move from end2 to the displaced threshold
    Point3D ref = end2;
    double lon = 0, lat = 0, r = 0;
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), heading_deg,
                        length_m / 2.0 - displ2, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the l,-w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    Point3D p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the l,w corner
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,w point (then we add points in a clockwise direction)

        ref = Point3D( (end1.lon()+end2.lon())/2.0f, (end1.lat()+end2.lat())/2.0f, 0.0f);
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                            width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    // move to the end1 center to the displ. threshold
    ref = end1;
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), heading_deg,
                        displ1 - length_m / 2.0, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the -l,w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the -l,-w corner
    geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                        -width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,-w point (then we add points in a clockwise direction)

        ref = Point3D( (end1.lon()+end2.lon())/2.0f, (end1.lat()+end2.lat())/2.0f, 0.0f);
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
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
