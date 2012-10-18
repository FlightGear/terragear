// construct.cxx -- Class to manage the primary data used in the
//                  construction process
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

//#include <iostream>

//#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>

#include <Geometry/poly_support.hxx>
#include <Geometry/poly_extra.hxx>

#include "tgconstruct.hxx"

using std::string;

void TGConstruct::FixTJunctions( void ) {
    int before, after;

    // traverse each poly, and add intermediate nodes
    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        for( unsigned int j = 0; j < polys_clipped.area_size(i); ++j ) {
            for( unsigned int k = 0; k < polys_clipped.shape_size(i, j); ++k ) {
                TGPolygon current = polys_clipped.get_poly(i, j, k);

                before  = current.total_size();
                current = add_tgnodes_to_poly( current, &nodes );
                after   = current.total_size();

                if (before != after) {
                   SG_LOG( SG_CLIPPER, SG_INFO, "Fixed T-Junctions in " << get_area_name( (AreaType)i ) << ":" << j+1 << "-" << k << " of " << (int)polys_clipped.area_size(i) << " nodes increased from " << before << " to " << after );
                }

                /* Save it back */
                polys_clipped.set_poly( i, j, k, current );
            }
        }
    }
}

// Attempt to merge slivers into a list of polygons.
//
// For each sliver contour, see if a union with another polygon yields
// a polygon with no increased contours (i.e. the sliver is adjacent
// and can be merged.)  If so, replace the clipped polygon with the
// new polygon that has the sliver merged in.
void TGConstruct::merge_slivers( TGLandclass& clipped,  poly_list& slivers_list ) {
    TGPolygon poly, result, slivers, sliver;
    point_list contour;
    int original_contours, result_contours;
    bool done;
    int area, shape, segment, i, j;
    int merged = 0;
    int total = 0;

    for ( i = 0; i < (int)slivers_list.size(); i++ ) {
        slivers = slivers_list[i];

        for ( j = 0; j < slivers.contours(); ++j ) {
            // make the sliver polygon
            contour = slivers.get_contour( j );
            total++;

            sliver.erase();
            sliver.add_contour( contour, 0 );
            done = false;

            for ( area = 0; area < TG_MAX_AREA_TYPES && !done; ++area ) {
                if ( is_hole_area( area ) ) {
                    // don't merge a non-hole sliver in with a hole
                    continue;
                }

                for ( shape = 0; shape < (int)clipped.area_size(area) && !done; ++shape ) {
                    unsigned int shape_id = clipped.get_shape( area, shape ).id;

                    for ( segment = 0; segment < (int)clipped.shape_size(area, shape) && !done; ++segment ) {

                        poly = clipped.get_poly( area, shape, segment );
                        original_contours = poly.contours();
                        result = tgPolygonUnion( poly, sliver );
                        result_contours = result.contours();

                        if ( original_contours == result_contours ) {
                            SG_LOG(SG_GENERAL, SG_INFO, "MERGED SLIVER " << i << ", " << j << " into area " << get_area_name( (AreaType)area ) << " id: " << shape_id << " segment: " << segment  );

                            clipped.set_poly( area, shape, segment, result );
                            merged++;

                            /* add the sliver to the clip_mask, too */
                            TGPolygon mask = clipped.get_mask( area, shape );
                            result = tgPolygonUnion( mask, sliver );
                            clipped.set_mask( area, shape, result );

                            if ( IsDebugShape( shape_id ) ) {
                                WriteDebugShape( "with_slivers", clipped.get_shape( area, shape ) );
                            }

                            done = true;
                        }
                    }
                }
            }
        }
    }

    slivers_list.clear();

    SG_LOG(SG_GENERAL, SG_INFO, " UNMERGED SLIVERS: " << total - merged );
}

void TGConstruct::CleanClippedPolys() {

    // Clean the polys
    for ( unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++ ) {
        for( unsigned int shape = 0; shape < polys_clipped.area_size(area); shape++ ) {
            unsigned int id = polys_clipped.get_shape( area, shape ).id;

            // step 1 : snap
            for ( unsigned int segment = 0; segment < polys_clipped.shape_size(area, shape); segment++ ) {
                TGPolygon poly = polys_clipped.get_poly(area, shape, segment);
                poly = snap(poly, gSnap);
                polys_clipped.set_poly( area, shape, segment, poly );
            }

            if ( IsDebugShape( id ) ) {
                WriteDebugShape( "snapped", polys_clipped.get_shape( area, shape ) );
            }

            // step 2 : remove_dups
            for ( unsigned int segment = 0; segment < polys_clipped.shape_size(area, shape); segment++ ) {
                TGPolygon poly = polys_clipped.get_poly(area, shape, segment);
                poly = remove_dups( poly );
                polys_clipped.set_poly( area, shape, segment, poly );
            }

            if ( IsDebugShape( id ) ) {
                WriteDebugShape( "rem dupes", polys_clipped.get_shape( area, shape ) );
            }

            // step 3 : remove_bad_contours
            for ( unsigned int segment = 0; segment < polys_clipped.shape_size(area, shape); segment++ ) {
                TGPolygon poly = polys_clipped.get_poly(area, shape, segment);
                poly = remove_bad_contours( poly );
                polys_clipped.set_poly( area, shape, segment, poly );
            }

            if ( IsDebugShape( id ) ) {
                WriteDebugShape( "rem bad contours", polys_clipped.get_shape( area, shape ) );
            }

// todo - add up all segments in a shape for printout
#if 0
            after = poly.total_size();
            if (before != after) {
                SG_LOG( SG_CLIPPER, SG_INFO, "Cleanined poly " << get_area_name( (AreaType)area ) <<
                                                                                ":" << shape+1 << "-" << segment << " of " << polys_clipped.area_size(area) << " before: " << before << " after: " << after );
            }
#endif

        }
    }
}

void TGConstruct::AverageEdgeElevations( void )
{
    for ( unsigned int i = 0; i < neighbor_faces.size(); i++ ) {
        TGNeighborFaces faces = neighbor_faces[i];
        double elevation = 0.0;
        unsigned int num_elevations = faces.elevations.size();

        for ( unsigned int j = 0; j < num_elevations; j++ ) {
            elevation += faces.elevations[j];
        }

        elevation = elevation / num_elevations;

        /* Find this node, and update it's elevation */
        int idx = nodes.find( faces.node );

        if (idx != -1) {
            TGNode node = nodes.get_node( idx );

            if ( !node.GetFixedPosition() ) {
                // set elevation as the average between all tiles that have it
                nodes.SetElevation( idx, elevation );
            }
        }
    }
}
