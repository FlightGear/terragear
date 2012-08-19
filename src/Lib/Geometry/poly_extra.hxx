// poly_extra.hxx -- Extra polygon manipulation routines
//
// Written by Curtis Olson, started February 2002.
//
// Copyright (C) 2002  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: poly_extra.hxx,v 1.5 2004-11-19 22:25:49 curt Exp $
//


#ifndef _POLY_EXTRA_HXX
#define _POLY_EXTRA_HXX


#include <Geometry/point3d.hxx>

#include <Geometry/trinodes.hxx>
#include <Geometry/tg_nodes.hxx>

#include <Polygon/polygon.hxx>


// Divide segment if there are other existing points on it, return the
// new polygon
void add_intermediate_nodes( int contour, const Point3D& start, 
                             const Point3D& end, const point_list& tmp_nodes,
                             TGPolygon *result,
                             const double bbEpsilon = SG_EPSILON*10,
                             const double errEpsilon = SG_EPSILON*4
                           );


// Search each segment for additional vertex points that may have been
// created elsewhere that lie on the segment and split it there to
// avoid "T" intersections.

TGPolygon add_nodes_to_poly( const TGPolygon& poly, 
                             const TGTriNodes& tmp_nodes );

TGPolygon add_tgnodes_to_poly( const TGPolygon& poly, 
                               const TGNodes* nodes );


#endif // _POLY_EXTRA_HXX
