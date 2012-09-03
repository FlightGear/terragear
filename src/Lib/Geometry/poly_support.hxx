// poly_support.hxx -- additional supporting routines for the TGPolygon class
//                     specific to the object building process.
//
// Written by Curtis Olson, started October 1999.
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
// $Id: poly_support.hxx,v 1.14 2004-11-19 22:25:50 curt Exp $


#ifndef _POLY_SUPPORT_HXX
#define _POLY_SUPPORT_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <Polygon/polygon.hxx>

#include "trieles.hxx"
#include "trinodes.hxx"


// Calculate the area of a triangle
inline double triangle_area( const Point3D p1,
			     const Point3D p2,
			     const Point3D p3 )
{
    return fabs(0.5 * ( p1.x() * p2.y() - p2.x() * p1.y() +
			p2.x() * p3.y() - p3.x() * p2.y() +
			p3.x() * p1.y() - p1.x() * p3.y() ));
}


// basic triangulation of a polygon with out adding points or
// splitting edges
int polygon_tesselate( const TGPolygon &p,
            const point_list &extra_nodes,
			triele_list &elelist,
			point_list &out_pts,
            std::string flags );

// Alternate basic triangulation of a polygon with out adding points
// or splitting edges and without regard for holes.  Returns a polygon
// with one contour per tesselated triangle.  This is mostly just a
// wrapper for the polygon_tesselate() function.  Note, this routine
// will modify the points_inside list for your polygon.
TGPolygon polygon_tesselate_alt( TGPolygon &p, bool verbose );

TGPolygon polygon_tesselate_alt_with_extra( TGPolygon &p, 
            const point_list &extra_nodes, bool verbose );

TGPolygon polygon_tesselate_alt_with_extra_cgal( TGPolygon &p,
            const point_list &extra_nodes, bool verbose );

TGPolygon polygon_tesselate_alt_cgal( TGPolygon &p, bool verbose );

// calculate some "arbitrary" point inside each of the polygons contours
void calc_points_inside( TGPolygon& p );

// snap all points to a grid
TGPolygon snap( const TGPolygon &poly, double grid_size );

// remove duplicate nodes in a polygon should they exist.  Returns the
// fixed polygon
TGPolygon remove_dups( const TGPolygon &poly );


// Search each segment of each contour for degenerate points (i.e. out
// of order points that lie coincident on other segments
TGPolygon reduce_degeneracy( const TGPolygon& poly );


// Occasionally the outline of the clipped polygon can take a side
// track, then double back on return to the start of the side track
// branch and continue normally.  Attempt to detect and clear this
// extraneous nonsense.
TGPolygon remove_cycles( const TGPolygon& poly );

// Occasionally the outline of the clipped polygon can have long spikes 
// that come close to doubling back on the same segment - this kills 
// triangulation
TGPolygon remove_spikes( const TGPolygon& poly );


// Find a point in the given node list that lies between start and
// end, return true if something found, false if nothing found.
bool find_intermediate_node( const Point3D& start, const Point3D& end,
                             const point_list& nodes, Point3D *result,
                             const double bbEpsilon = SG_EPSILON*10,
                             const double errEpsilon = SG_EPSILON*4
                           );

// remove any degenerate contours
TGPolygon remove_bad_contours( const TGPolygon &poly );

// remove any too small contours
TGPolygon remove_tiny_contours( const TGPolygon &poly );
TGPolygon remove_small_contours( const TGPolygon &poly );


// Write Polygons to Shapefile Support
void    tgShapefileInit( void );
void*   tgShapefileOpenDatasource( const char* datasource_name );
void*   tgShapefileOpenLayer( void* ds_id, const char* layer_name );
void    tgShapefileCreateFeature( void* ds_id, void* l_id, const TGPolygon &poly, const char* feature_name, bool has_point_inside = false );
void    tgShapefileCloseLayer( void* l_id );
void*   tgShapefileCloseDatasource( void* ds_id );


#endif // _POLY_SUPPORT_HXX


