// tgconstruct_elevation.cxx -- handle elevation functionality for tgconstruct
//
// Written by Curtis Olson, started May 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: construct.cxx,v 1.4 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/debug/logstream.hxx>

#include "tgconstruct.hxx"

using std::string;

// Load elevation data from an Array file (a regular grid of elevation data)
// and return list of fitted nodes.
void TGConstruct::LoadElevationArray( bool add_nodes ) {
    string base = bucket.gen_base_path();
    int i;

    for ( i = 0; i < (int)load_dirs.size(); ++i ) {
        string array_path = work_base + "/" + load_dirs[i] + "/" + base + "/" + bucket.gen_index_str();

        if ( array.open(array_path) ) {
            break;
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Failed to open Array file " << array_path);
        }
    }

    array.parse( bucket );
    array.remove_voids( );

    if ( add_nodes ) {
    point_list corner_list = array.get_corner_list();
    for (unsigned int i=0; i<corner_list.size(); i++) {
        nodes.unique_add(corner_list[i]);
    }

    point_list fit_list = array.get_fitted_list();
    for (unsigned int i=0; i<fit_list.size(); i++) {
        nodes.unique_add(fit_list[i]);
    }
}
}

static void calc_gc_course_dist( const Point3D& start, const Point3D& dest,
                                       double *course, double *dist ) {
    SGGeoc gs = start.toSGGeoc();
    SGGeoc gd = dest.toSGGeoc();
    *course = SGGeoc::courseRad(gs, gd);
    *dist = SGGeoc::distanceM(gs, gd);
}

// calculate spherical distance between two points (lon, lat specified
// in degrees, result returned in meters)
static double distanceSphere( const Point3D p1, const Point3D p2 ) {
    Point3D r1( p1.x() * SG_DEGREES_TO_RADIANS,
                p1.y() * SG_DEGREES_TO_RADIANS,
                p1.z() );
    Point3D r2( p2.x() * SG_DEGREES_TO_RADIANS,
                p2.y() * SG_DEGREES_TO_RADIANS,
                p2.z() );

    double course, dist_m;
    calc_gc_course_dist( r1, r2, &course, &dist_m );

    return dist_m;
}

// fix the elevations of the geodetic nodes
// This should be done in the nodes class itself, except for the need for the triangle type
// hopefully, this will get better when we have the area lookup via superpoly...
void TGConstruct::CalcElevations( void )
{
    TGPolyNodes tri_nodes;
    double e1, e2, e3, min;
    int    n1, n2, n3;
    point_list  raw_nodes;
    Point3D p;

    SG_LOG(SG_GENERAL, SG_ALERT, "fixing node heights");

    for (int i = 0; i < (int)nodes.size(); ++i) {
        TGNode node = nodes.get_node( i );
        Point3D pos = node.GetPosition();

        if ( !node.GetFixedPosition() ) {
            // set elevation as interpolated point from DEM data.
            nodes.SetElevation( i, array.altitude_from_grid(pos.x() * 3600.0, pos.y() * 3600.0) );
        }
    }

    raw_nodes = nodes.get_geod_nodes();
    // now flatten some stuff
    for (unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++) {
        if ( is_lake_area( (AreaType)area ) ) {
            for (int shape = 0; shape < (int)polys_clipped.area_size(area); ++shape ) {
                for (int segment = 0; segment < (int)polys_clipped.shape_size(area, shape); ++segment ) {

                    SG_LOG( SG_CLIPPER, SG_INFO, "Flattening " << get_area_name( (AreaType)area ) << ":" << shape+1 << "-" << segment << " of " << (int)polys_clipped.area_size(area) );
                    tri_nodes = polys_clipped.get_tri_idxs( area, shape, segment );

                    for (int tri=0; tri < tri_nodes.contours(); tri++) {
                        if (tri_nodes.contour_size( tri ) != 3) {
                            SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_nodes.contour_size( tri ) );
                            exit(0);
                        }

                        n1 = tri_nodes.get_pt( tri, 0 );
                        e1 = nodes.get_node(n1).GetPosition().z();
                        n2 = tri_nodes.get_pt( tri, 1 );
                        e2 = nodes.get_node(n2).GetPosition().z();
                        n3 = tri_nodes.get_pt( tri, 2 );
                        e3 = nodes.get_node(n3).GetPosition().z();

                        min = e1;
                        if ( e2 < min ) { min = e2; }
                        if ( e3 < min ) { min = e3; }

                        nodes.SetElevation( n1, min );
                        nodes.SetElevation( n2, min );
                        nodes.SetElevation( n3, min );
                    }
                }
            }
        }

        if ( is_stream_area( (AreaType)area ) ) {
            for (int shape = 0; shape < (int)polys_clipped.area_size(area); ++shape ) {
                for (int segment = 0; segment < (int)polys_clipped.shape_size(area, shape); ++segment ) {

                    SG_LOG( SG_CLIPPER, SG_INFO, "Flattening " << get_area_name( (AreaType)area ) << ":" << shape+1 << "-" << segment << " of " << (int)polys_clipped.area_size(area) );
                    tri_nodes = polys_clipped.get_tri_idxs( area, shape, segment );

                    for (int tri=0; tri < tri_nodes.contours(); tri++) {
                        if (tri_nodes.contour_size( tri ) != 3) {
                            SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_nodes.contour_size( tri ) );
                            exit(0);
                        }


                        n1 = tri_nodes.get_pt( tri, 0 );
                        e1 = nodes.get_node(n1).GetPosition().z();
                        n2 = tri_nodes.get_pt( tri, 1 );
                        e2 = nodes.get_node(n2).GetPosition().z();
                        n3 = tri_nodes.get_pt( tri, 2 );
                        e3 = nodes.get_node(n3).GetPosition().z();

                        min = e1;
                        p   = raw_nodes[n1];

                        if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }
                        if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }

                        double d1 = distanceSphere( p, raw_nodes[n1] );
                        double d2 = distanceSphere( p, raw_nodes[n2] );
                        double d3 = distanceSphere( p, raw_nodes[n3] );

                        double max1 = d1 * 0.20 + min;
                        double max2 = d2 * 0.20 + min;
                        double max3 = d3 * 0.20 + min;

                        if ( max1 < e1 ) { nodes.SetElevation( n1, max1 ); }
                        if ( max2 < e2 ) { nodes.SetElevation( n2, max2 ); }
                        if ( max3 < e3 ) { nodes.SetElevation( n3, max3 ); }
                    }
                }
            }
        }

        if ( is_road_area( (AreaType)area ) ) {
            for (int shape = 0; shape < (int)polys_clipped.area_size(area); ++shape ) {
                for (int segment = 0; segment < (int)polys_clipped.shape_size(area, shape); ++segment ) {

                    SG_LOG( SG_CLIPPER, SG_INFO, "Flattening " << get_area_name( (AreaType)area ) << ":" << shape+1 << "-" << segment << " of " << (int)polys_clipped.area_size(area) );
                    tri_nodes = polys_clipped.get_tri_idxs( area, shape, segment );

                    for (int tri=0; tri < tri_nodes.contours(); tri++) {
                        if (tri_nodes.contour_size( tri ) != 3) {
                            SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_nodes.contour_size( tri ) );
                            exit(0);
                        }


                        n1 = tri_nodes.get_pt( tri, 0 );
                        e1 = nodes.get_node(n1).GetPosition().z();
                        n2 = tri_nodes.get_pt( tri, 1 );
                        e2 = nodes.get_node(n2).GetPosition().z();
                        n3 = tri_nodes.get_pt( tri, 2 );
                        e3 = nodes.get_node(n3).GetPosition().z();

                        min = e1;
                        p   = raw_nodes[n1];

                        if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }
                        if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }

                        double d1 = distanceSphere( p, raw_nodes[n1] );
                        double d2 = distanceSphere( p, raw_nodes[n2] );
                        double d3 = distanceSphere( p, raw_nodes[n3] );

                        double max1 = d1 * 0.30 + min;
                        double max2 = d2 * 0.30 + min;
                        double max3 = d3 * 0.30 + min;

                        if ( max1 < e1 ) { nodes.SetElevation( n1, max1 ); }
                        if ( max2 < e2 ) { nodes.SetElevation( n2, max2 ); }
                        if ( max3 < e3 ) { nodes.SetElevation( n3, max3 ); }
                    }
                }
            }
        }

        if ( is_ocean_area( (AreaType)area ) ) {
            for (int shape = 0; shape < (int)polys_clipped.area_size(area); ++shape ) {
                for (int segment = 0; segment < (int)polys_clipped.shape_size(area, shape); ++segment ) {

                    SG_LOG( SG_CLIPPER, SG_INFO, "Flattening " << get_area_name( (AreaType)area ) << ":" << shape+1 << "-" << segment << " of " << (int)polys_clipped.area_size(area) );
                    tri_nodes = polys_clipped.get_tri_idxs( area, shape, segment );

                    for (int tri=0; tri < tri_nodes.contours(); tri++) {
                        if (tri_nodes.contour_size( tri ) != 3) {
                            SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_nodes.contour_size( tri ) );
                            exit(0);
                        }

                        n1 = tri_nodes.get_pt( tri, 0 );
                        n2 = tri_nodes.get_pt( tri, 1 );
                        n3 = tri_nodes.get_pt( tri, 2 );

                        nodes.SetElevation( n1, 0.0 );
                        nodes.SetElevation( n2, 0.0 );
                        nodes.SetElevation( n3, 0.0 );
                    }
                }
            }
        }
    }
}