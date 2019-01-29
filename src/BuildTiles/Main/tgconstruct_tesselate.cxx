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

#include <simgear/debug/logstream.hxx>

#include <terragear/tg_shapefile.hxx>

#include "tgconstruct.hxx"

void TGConstruct::TesselatePolys( void )
{
    // tesselate the polygons and prepair them for final output
    std::vector<SGGeod> poly_extra;
    SGGeod min, max;

    for (unsigned int area = 0; area < area_defs.size(); area++) {
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon poly = polys_clipped.get_poly(area, p );

            if ( IsDebugShape( poly.GetId() ) ) {
                char layer[32];
                sprintf(layer, "pretess_%u_%u", area, p );
                tgShapefile::FromPolygon( poly, ds_name, layer, "poly" );
            }

            tgRectangle rect = poly.GetBoundingBox();
            nodes.get_geod_inside( rect.getMin(), rect.getMax(), poly_extra );

            SG_LOG( SG_CLIPPER, SG_DEBUG, "Tesselating " << area_defs.get_area_name(area) << "(" << area << "): " <<
                    p+1 << " of " << polys_clipped.area_size(area) << ": id = " << poly.GetId() );

            if ( IsDebugShape( poly.GetId() ) ) {
                SG_LOG( SG_CLIPPER, SG_INFO, poly );
            }

            poly.Tesselate( poly_extra );

            polys_clipped.set_poly( area, p, poly );
        }
    }

    for (unsigned int area = 0; area < area_defs.size(); area++) {
        bool isRoad = area_defs.is_road_area( area );        
        for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
            tgPolygon& poly = polys_clipped.get_poly(area, p );

            // ensure all added nodes are accounted for
            for (unsigned int k=0; k < poly.Triangles(); k++) {
                for (int l = 0; l < 3; l++) {
                    // ensure we have all nodes...
                    SGGeod node = poly.GetTriNode( k, l );
                    if ( CheckMatchingNode( node, isRoad, false ) ) {
                        nodes.unique_add( node );
                    } else {
                        SG_LOG( SG_GENERAL, SG_INFO, "after tesselation, we can't add a node - quit");
                        exit(1);
                    }
                }
            }
        }
    }
}