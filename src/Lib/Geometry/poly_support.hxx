// poly_support.hxx -- additional supporting routines for the FGPolygon class
//                     specific to the object building process.
//
// Written by Curtis Olson, started October 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$


#ifndef _POLY_SUPPORT_HXX
#define _POLY_SUPPORT_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <Polygon/polygon.hxx>
#include <Triangulate/trieles.hxx>

#include "trinodes.hxx"


// basic triangulation of a polygon with out adding points or
// splitting edges
void polygon_tesselate( const FGPolygon &p,
			triele_list &elelist,
			point_list &out_pts );

// Alternate basic triangulation of a polygon with out adding points
// or splitting edges and without regard for holes.  Returns a polygon
// with one contour per tesselated triangle.  This is mostly just a
// wrapper for the polygon_tesselate() function.  Note, this routine
// will modify the points_inside list for your polygon.
FGPolygon polygon_tesselate_alt( FGPolygon &p );

// calculate some "arbitrary" point inside the specified contour for
// assigning attribute areas.  This requires data structures outside
// of "FGPolygon" which is why it is living over here in "Lib/Geometry"
Point3D calc_point_inside_old( const FGPolygon& p, const int contour, 
			       const FGTriNodes& trinodes );

// calculate some "arbitrary" point inside each of the polygons contours
void calc_points_inside( FGPolygon& p );

// remove duplicate nodes in a polygon should they exist.  Returns the
// fixed polygon
FGPolygon remove_dups( const FGPolygon &poly );


// Search each segment of each contour for degenerate points (i.e. out
// of order points that lie coincident on other segments
FGPolygon reduce_degeneracy( const FGPolygon& poly );


// Find a point in the given node list that lies between start and
// end, return true if something found, false if nothing found.
bool find_intermediate_node( const Point3D& start, const Point3D& end,
			     const point_list& nodes, Point3D *result );

// remove any degenerate contours
FGPolygon remove_bad_contours( const FGPolygon &poly );


#endif // _POLY_SUPPORT_HXX


