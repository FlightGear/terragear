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
    std::vector<SGGeod> points;
    tg::Rectangle bb;

    // traverse each poly, and add intermediate nodes
    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        for( unsigned int j = 0; j < polys_clipped.area_size(i); ++j ) {
            tgPolygon current = polys_clipped.get_poly(i, j);
            bb = current.GetBoundingBox();
            nodes.get_geod_inside( bb.getMin(), bb.getMax(), points );

            before  = current.TotalNodes();
            current = tgPolygon::AddColinearNodes( current, points );
            after   = current.TotalNodes();

            if (before != after) {
               SG_LOG( SG_CLIPPER, SG_DEBUG, "Fixed T-Junctions in " << get_area_name( (AreaType)i ) << ":" << j+1 << " of " << (int)polys_clipped.area_size(i) << " nodes increased from " << before << " to " << after );
            }

            /* Save it back */
            polys_clipped.set_poly( i, j, current );
        }
    }
}

// Attempt to merge slivers into a list of polygons.
//
// For each sliver contour, see if a union with another polygon yields
// a polygon with no increased contours (i.e. the sliver is adjacent
// and can be merged.)  If so, replace the clipped polygon with the
// new polygon that has the sliver merged in.
void TGConstruct::merge_slivers( TGLandclass& clipped,  tgcontour_list& sliver_list ) {
    for ( unsigned int area = 0; area < TG_MAX_AREA_TYPES && sliver_list.size(); ++area ) {
        if ( is_hole_area( area ) ) {
            // don't merge a non-hole sliver in with a hole
            continue;
        }

        sliver_list = tgPolygon::MergeSlivers( polys_clipped.get_polys(area), sliver_list );
    }

    SG_LOG(SG_GENERAL, SG_INFO, " UNMERGED SLIVERS: " << sliver_list.size() );

    sliver_list.clear();
}

void TGConstruct::CleanClippedPolys() {
    // Clean the polys
    for ( unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++ ) {
        for( unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            // step 1 : snap
            tgPolygon poly = polys_clipped.get_poly(area, p);
            poly = tgPolygon::Snap(poly, gSnap);

            if ( IsDebugShape( poly.GetId() ) ) {
                tgPolygon::ToShapefile( poly, ds_name, "snapped", "" );
            }

            // step 2 : remove_dups
            poly = tgPolygon::RemoveDups( poly );

            if ( IsDebugShape( poly.GetId() ) ) {
                tgPolygon::ToShapefile( poly, ds_name, "rem_dups", "" );
            }

            // step 3 : remove_bad_contours
            poly = tgPolygon::RemoveBadContours( poly );

            if ( IsDebugShape( poly.GetId() ) ) {
                tgPolygon::ToShapefile( poly, ds_name, "rem_bcs", "" );
            }

            polys_clipped.set_poly(area, p, poly);
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
            TGNode const& node = nodes.get_node( idx );

            if ( !node.GetFixedPosition() ) {
                // set elevation as the average between all tiles that have it
                nodes.SetElevation( idx, elevation );
            }
        }
    }
}
