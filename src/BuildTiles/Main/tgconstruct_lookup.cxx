// tgconstruct_lookup.cxx -- Lookup function to store precomputed pointers
//                           greatly optimize certain stages
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

// This function populates the Superpoly tri_idx polygon.
// This polygon is a mirror of tris, except the verticies are
// indexes into the node array (cast as unsigned long)
void TGConstruct::LookupNodesPerVertex( void )
{
    SG_LOG(SG_GENERAL, SG_ALERT, "LookupNodesPerVertex");

    // for each node, traverse all the triangles - and create face lists
    for ( unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++ ) {
        for( unsigned int shape = 0; shape < polys_clipped.area_size(area); shape++ ) {
            for ( unsigned int segment = 0; segment < polys_clipped.shape_size(area, shape); segment++ ) {
                TGPolygon   tris = polys_clipped.get_tris( area, shape, segment );
                TGPolyNodes tri_nodes;
                int         idx;

                for (int tri=0; tri < tris.contours(); tri++) {
                    for (int vertex = 0; vertex < tris.contour_size(tri); vertex++) {
                        idx = nodes.find( tris.get_pt( tri, vertex ).toSGGeod() );
                        if (idx >= 0) {
                            tri_nodes.add_node( tri, idx );
                        } else {
                            SG_LOG(SG_GENERAL, SG_ALERT, "didn't find vertex! " << tris.get_pt( tri, vertex ) );
                            exit(0);
                        }
                    }
                }
                polys_clipped.set_tri_idxs(area, shape, segment, tri_nodes);
            }
        }
    }
}

void TGConstruct::LookupFacesPerNode( void )
{
    SG_LOG(SG_GENERAL, SG_ALERT, "LookupFacesPerNode");

    // Add each face that includes a node to the node's face list
    for ( unsigned int area = 0; area < TG_MAX_AREA_TYPES; area++ ) {
        for( unsigned int shape = 0; shape < polys_clipped.area_size(area); shape++ ) {
            for ( unsigned int segment = 0; segment < polys_clipped.shape_size(area, shape); segment++ ) {
                TGPolygon   tris = polys_clipped.get_tris(area, shape, segment);

                for (int tri=0; tri < tris.contours(); tri++) {
                    for (int sub = 0; sub < tris.contour_size(tri); sub++) {
                        int n = nodes.find( tris.get_pt( tri, sub ).toSGGeod() );
                        nodes.AddFace( n, area, shape, segment, tri );
                    }
                }
            }
        }
    }
}
