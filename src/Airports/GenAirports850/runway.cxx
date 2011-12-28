// area.c -- routines to assist with inserting "areas" into FG terrain
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: runway.cxx,v 1.18 2004-11-19 22:25:49 curt Exp $
//


#include <math.h>
#include <stdio.h>

#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include "runway.hxx"
#include "point2d.hxx"


// given a runway center point, length, width, and heading, and
// altitude (meters) generate the lon and lat 4 corners using wgs84
// math.
TGPolygon gen_wgs84_area( Point3D origin,
                          double length_m,
                          double displ1, double displ2,
                          double width_m,
                          double heading_deg,
                          double alt_m,
                          bool add_mid )
{
    TGPolygon result_list;
    double length_hdg = heading_deg;
    double left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    // move to the +l end/center of the runway
    Point3D ref = origin;
    double lon, lat, r;
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


// generate an area for a runway with expantion specified as a scale
// factor (return result points in degrees)
TGPolygon gen_runway_area_w_scale( const TGRunway& runway,
                                   double alt_m,
				   double length_scale,
				   double width_scale ) {

    TGPolygon result_list;
    Point3D origin(runway.lon, runway.lat, 0);

    result_list = gen_wgs84_area( origin,
                                  runway.length*length_scale * SG_FEET_TO_METER,
                                  0.0, 0.0,
                                  runway.width*width_scale * SG_FEET_TO_METER,
                                  runway.heading, alt_m, false );

    // display points
    SG_LOG(SG_GENERAL, SG_DEBUG, "Results w/ scale (new way)");
    for ( int i = 0; i < result_list.contour_size( 0 ); ++i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  " << result_list.get_pt(0, i));
    }

    return result_list;
}


// generate an area for a runway with expansion specified in meters
// (return result points in degrees)
TGPolygon gen_runway_area_w_extend( const TGRunway& runway,
                                    double alt_m,
				    double length_extend,
                                    double displ1, double displ2,
				    double width_extend ) {

    TGPolygon result_list;
    Point3D origin(runway.lon, runway.lat, 0);

    result_list
        = gen_wgs84_area( origin,
                          runway.length*SG_FEET_TO_METER + 2.0*length_extend,
                          displ1, displ2,
                          runway.width*SG_FEET_TO_METER + 2.0*width_extend,
                          runway.heading, alt_m, false );

    // display points
    SG_LOG(SG_GENERAL, SG_DEBUG, "Results w/ extend (new way)");
    for ( int i = 0; i < result_list.contour_size( 0 ); ++i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  " << result_list.get_pt(0, i));
    }

    return result_list;
}


// generate an area for a runway and include midpoints
TGPolygon gen_runway_w_mid( const TGRunway& runway, 
                            double alt_m,
			    double length_extend_m,
			    double width_extend_m ) {
    TGPolygon result_list;
    Point3D origin(runway.lon, runway.lat, 0);

    result_list = gen_wgs84_area( origin,
                                  runway.length * SG_FEET_TO_METER
                                    + 2.0*length_extend_m,
                                  0.0, 0.0,
                                  runway.width * SG_FEET_TO_METER
                                    + 2.0 * width_extend_m,
                                  runway.heading, alt_m, true );

    // display points
    SG_LOG(SG_GENERAL, SG_DEBUG, "Results w/ mid (new way)");
    for ( int i = 0; i < result_list.contour_size( 0 ); ++i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  " << result_list.get_pt(0, i));
    }

    return result_list;
}
