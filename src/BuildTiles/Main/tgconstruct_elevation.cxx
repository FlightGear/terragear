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

    for ( int i = 0; i < (int)load_dirs.size(); ++i ) {
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
        std::vector<SGGeod> const& corner_list = array.get_corner_list();
        for (unsigned int i = 0; i < corner_list.size(); ++i) {
            nodes.unique_add( corner_list[i] );
        }

        std::vector<SGGeod> const& fit_list = array.get_fitted_list();
        for (unsigned int i = 0; i < fit_list.size(); ++i) {
            SGGeod node = fit_list[i];
            if ( CheckMatchingNode( node, false, false ) ) {
                nodes.unique_add( node );
            }
        }
    }
}

// fix the elevations of the geodetic nodes
// This should be done in the nodes class itself, except for the need for the triangle type
// hopefully, this will get better when we have the area lookup via superpoly...
void TGConstruct::CalcElevations( void )
{
    std::vector<SGGeod> raw_nodes;
    double e1, e2, e3, min;
    int    n1, n2, n3;

    for (int i = 0; i < (int)nodes.size(); ++i) {
        TGNode const& node = nodes.get_node( i );
        SGGeod pos = node.GetPosition();

        if ( !node.GetFixedPosition() ) {
            // set elevation as interpolated point from DEM data.
            nodes.SetElevation( i, array.altitude_from_grid(pos.getLongitudeDeg() * 3600.0, pos.getLatitudeDeg() * 3600.0) );
        }
    }

    nodes.get_geod_nodes(raw_nodes);

    // now flatten some stuff
    for (unsigned int area = 0; area < area_defs.size(); area++) {
        if ( area_defs.is_lake_area(area) ) {
            for (int p = 0; p < (int)polys_clipped.area_size(area); ++p ) {
                SG_LOG( SG_CLIPPER, SG_DEBUG, "Flattening " << area_defs.get_area_name(area) << ":" << p+1 << " of " << polys_clipped.area_size(area) );
                tgPolygon poly = polys_clipped.get_poly( area, p );

                for (unsigned int tri=0; tri < poly.Triangles(); tri++) {
                    n1 = poly.GetTriIdx( tri, 0 );
                    e1 = nodes.get_node(n1).GetPosition().getElevationM();
                    n2 = poly.GetTriIdx( tri, 1 );
                    e2 = nodes.get_node(n2).GetPosition().getElevationM();
                    n3 = poly.GetTriIdx( tri, 2 );
                    e3 = nodes.get_node(n3).GetPosition().getElevationM();

                    min = e1;
                    if ( e2 < min ) { min = e2; }
                    if ( e3 < min ) { min = e3; }

                    nodes.SetElevation( n1, min );
                    nodes.SetElevation( n2, min );
                    nodes.SetElevation( n3, min );
                }
            }
        }

        if ( area_defs.is_stream_area(area) ) {
            for (unsigned int p = 0; p < polys_clipped.area_size(area); ++p ) {
                SG_LOG( SG_CLIPPER, SG_DEBUG, "Flattening " << area_defs.get_area_name(area) << ":" << p+1 << " of " << polys_clipped.area_size(area) );
                tgPolygon poly = polys_clipped.get_poly( area, p );

                for (unsigned int tri=0; tri < poly.Triangles(); tri++) {
                    n1 = poly.GetTriIdx( tri, 0 );
                    e1 = nodes.get_node(n1).GetPosition().getElevationM();
                    n2 = poly.GetTriIdx( tri, 1 );
                    e2 = nodes.get_node(n2).GetPosition().getElevationM();
                    n3 = poly.GetTriIdx( tri, 2 );
                    e3 = nodes.get_node(n3).GetPosition().getElevationM();

                    min = e1;
                    SGGeod src = raw_nodes[n1];

                    if ( e2 < min ) { min = e2; src = raw_nodes[n2]; }
                    if ( e3 < min ) { min = e3; src = raw_nodes[n3]; }

                    double d1, d2, d3;
                    if ( min == e1 ) {
                        d1 = 0.0f;
                    } else {
                        d1 = SGGeodesy::distanceM( src, raw_nodes[n1] );
                    }
                    if ( min == e2 ) {
                        d2 = 0.0f;
                    } else {
                        d2 = SGGeodesy::distanceM( src, raw_nodes[n2] );
                    }
                    if ( min == e3 ) {
                        d3 = 0.0f;
                    } else {
                        d3 = SGGeodesy::distanceM( src, raw_nodes[n3] );
                    }

                    double max1 = d1 * 0.20 + min;
                    double max2 = d2 * 0.20 + min;
                    double max3 = d3 * 0.20 + min;

                    if ( max1 < e1 ) { nodes.SetElevation( n1, max1 ); }
                    if ( max2 < e2 ) { nodes.SetElevation( n2, max2 ); }
                    if ( max3 < e3 ) { nodes.SetElevation( n3, max3 ); }
                }
            }
        }

        if ( area_defs.is_road_area(area) ) {
            for (int p = 0; p < (int)polys_clipped.area_size(area); ++p ) {
                SG_LOG( SG_CLIPPER, SG_DEBUG, "Flattening " << area_defs.get_area_name(area) << ":" << p+1 << " of " << polys_clipped.area_size(area) );
                tgPolygon poly = polys_clipped.get_poly( area, p );

                for (unsigned int tri=0; tri < poly.Triangles(); tri++) {
                    n1 = poly.GetTriIdx( tri, 0 );
                    e1 = nodes.get_node(n1).GetPosition().getElevationM();
                    n2 = poly.GetTriIdx( tri, 1 );
                    e2 = nodes.get_node(n2).GetPosition().getElevationM();
                    n3 = poly.GetTriIdx( tri, 2 );
                    e3 = nodes.get_node(n3).GetPosition().getElevationM();

                    min = e1;
                    SGGeod src = raw_nodes[n1];

                    if ( e2 < min ) { min = e2; src = raw_nodes[n2]; }
                    if ( e3 < min ) { min = e3; src = raw_nodes[n3]; }

                    double d1, d2, d3;
                    if ( min == e1 ) {
                        d1 = 0.0f;
                    } else {
                        d1 = SGGeodesy::distanceM( src, raw_nodes[n1] );
                    }
                    if ( min == e2 ) {
                        d2 = 0.0f;
                    } else {
                        d2 = SGGeodesy::distanceM( src, raw_nodes[n2] );
                    }
                    if ( min == e3 ) {
                        d3 = 0.0f;
                    } else {
                        d3 = SGGeodesy::distanceM( src, raw_nodes[n3] );
                    }

                    double max1 = d1 * 0.30 + min;
                    double max2 = d2 * 0.30 + min;
                    double max3 = d3 * 0.30 + min;

                    if ( max1 < e1 ) { nodes.SetElevation( n1, max1 ); }
                    if ( max2 < e2 ) { nodes.SetElevation( n2, max2 ); }
                    if ( max3 < e3 ) { nodes.SetElevation( n3, max3 ); }
                }
            }
        }

        if ( area_defs.is_ocean_area(area) ) {
            for (int p = 0; p < (int)polys_clipped.area_size(area); ++p ) {
                SG_LOG( SG_CLIPPER, SG_DEBUG, "Flattening " << area_defs.get_area_name(area) << ":" << p+1 << " of " << polys_clipped.area_size(area) );
                tgPolygon poly = polys_clipped.get_poly( area, p );

                for (unsigned int tri=0; tri < poly.Triangles(); tri++) {
                    n1 = poly.GetTriIdx( tri, 0 );
                    n2 = poly.GetTriIdx( tri, 1 );
                    n3 = poly.GetTriIdx( tri, 2 );

                    nodes.SetElevation( n1, 0.0 );
                    nodes.SetElevation( n2, 0.0 );
                    nodes.SetElevation( n3, 0.0 );
                }
            }
        }
    }
}
