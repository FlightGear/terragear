// poly_extra.hxx -- Extra polygon manipulation routines
//
// Written by Curtis Olson, started February 2002.
//
// Copyright (C) 2002  Curtis L. Olson  - curt@flightgear.org
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
// $Id$
//


#ifndef _POLY_EXTRA_HXX
#define _POLY_EXTRA_HXX


#include <simgear/math/point3d.hxx>

#include <Geometry/trinodes.hxx>
#include <Polygon/polygon.hxx>


// Divide segment if there are other existing points on it, return the
// new polygon
void add_intermediate_nodes( int contour, const Point3D& start, 
			     const Point3D& end, const TGTriNodes& tmp_nodes,
			     FGPolygon *result );


// Search each segment for additional vertex points that may have been
// created elsewhere that lie on the segment and split it there to
// avoid "T" intersections.

FGPolygon add_nodes_to_poly( const FGPolygon& poly, 
				    const TGTriNodes& tmp_nodes );


// Traverse a polygon and split edges until they are less than max_len
// (specified in meters)
FGPolygon split_long_edges( const FGPolygon &poly, double max_len );


// Traverse a polygon and toss all the internal holes
FGPolygon strip_out_holes( const FGPolygon &poly );


#endif // _POLY_EXTRA_HXX
