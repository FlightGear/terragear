// polygon.hxx -- polygon (with holes) management class
//
// Written by Curtis Olson, started March 1999.
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


#ifndef _POLYGON_HXX
#define _POLYGON_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <iostream>
#include <string>
#include <vector>

FG_USING_STD(ostream);
FG_USING_STD(string);
FG_USING_STD(vector);


#define FG_MAX_VERTICES 1500000


typedef vector < point_list > polytype;
typedef polytype::iterator polytype_iterator;
typedef polytype::const_iterator const_polytype_iterator;


class FGPolygon {

private:

    polytype poly;           // polygons
    point_list inside_list;  // point inside list
    int_list hole_list;      // hole flag list

public:

    // Constructor and destructor
    FGPolygon( void );
    ~FGPolygon( void );

    // Add a contour
    inline void add_contour( const point_list contour, const int hole_flag ) {
	poly.push_back( contour );
	inside_list.push_back( Point3D( 0.0 ) );
	hole_list.push_back( hole_flag );
    }

    // Get a contour
    inline point_list get_contour( const int i ) const {
	return poly[i];
    }

    // Delete a contour
    inline void delete_contour( const int i ) {
	polytype_iterator start_poly = poly.begin();
	poly.erase( start_poly + i );

	point_list_iterator start_inside = inside_list.begin();
	inside_list.erase( start_inside + i );

	int_list_iterator start_hole = hole_list.begin();
	hole_list.erase( start_hole + i );
    }

    // Add the specified node (index) to the polygon
    inline void add_node( int contour, Point3D p ) {
	if ( contour >= (int)poly.size() ) {
	    // extend polygon
	    point_list empty_contour;
	    empty_contour.clear();
	    for ( int i = 0; i < contour - (int)poly.size() + 1; ++i ) {
		poly.push_back( empty_contour );
		inside_list.push_back( Point3D(0.0) );
		hole_list.push_back( 0 );
	    }
	}
	poly[contour].push_back( p );
    }

    // return size
    inline int contours() const { return poly.size(); }
    inline int contour_size( int contour ) const { 
	return poly[contour].size();
    }
    inline int total_size() const {
	int size = 0;
	for ( int i = 0; i < contours(); ++i ) {
	    size += poly[i].size();
	}
	return size;
    }

    // return the ith point from the specified contour
    inline Point3D get_pt( int contour, int i ) const { 
	return poly[contour][i];
    }

    // update the value of a point
    inline void set_pt( int contour, int i, const Point3D& p ) { 
	poly[contour][i] = p;
    }

    // get and set an arbitrary point inside the specified polygon contour
    inline Point3D get_point_inside( const int contour ) const { 
	return inside_list[contour];
    }
    inline void set_point_inside( int contour, const Point3D& p ) { 
	inside_list[contour] = p;
    }

    // get and set hole flag
    inline int get_hole_flag( const int contour ) const { 
	return hole_list[contour];
    }
    inline void set_hole_flag( const int contour, const int flag ) {
	hole_list[contour] = flag;
    }
    inline bool has_holes () const {
        for (int i = 0; i < contours(); i++) {
	    if (get_hole_flag(i))
	        return true;
	}
	return false;
    }

    // shift every point in the polygon by lon, lat
    void shift( double lon, double lat );

    // erase
    inline void erase() { poly.clear(); }

    // informational

    // return the area of a contour (assumes simple polygons,
    // i.e. non-self intersecting.)
    //
    // negative areas indicate counter clockwise winding
    // positive areas indicate clockwise winding.
    double area_contour( const int contour ) const;

    // return the smallest interior angle of the contour
    double minangle_contour( const int contour );

    // return true if contour B is inside countour A
    bool is_inside( int a, int b ) const;

    // output
    void write( const string& file ) const;

    // output
    void write_contour( const int contour, const string& file ) const;
};


typedef vector < FGPolygon > poly_list;
typedef poly_list::iterator poly_list_iterator;
typedef poly_list::const_iterator const_poly_list_iterator;


// canonify the polygon winding, outer contour must be anti-clockwise,
// all inner contours must be clockwise.
FGPolygon polygon_canonify( const FGPolygon& in_poly );


// Wrapper for the fast Polygon Triangulation based on Seidel's
// Algorithm by Atul Narkhede and Dinesh Manocha
// http://www.cs.unc.edu/~dm/CODE/GEM/chapter.html

FGPolygon polygon_to_tristrip( const FGPolygon& poly );


// wrapper functions for gpc polygon clip routines

// Difference
FGPolygon polygon_diff(	const FGPolygon& subject, const FGPolygon& clip );

// Intersection
FGPolygon polygon_int( const FGPolygon& subject, const FGPolygon& clip );

// Exclusive or
FGPolygon polygon_xor( const FGPolygon& subject, const FGPolygon& clip );

// Union
FGPolygon polygon_union( const FGPolygon& subject, const FGPolygon& clip );

// Output
ostream &operator<< (ostream &output, const FGPolygon &poly);


#endif // _POLYGON_HXX


