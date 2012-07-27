// superpoly.hxx -- Manage all aspects of a rendered polygon
//
// Written by Curtis Olson, started June 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: superpoly.hxx,v 1.9 2004-11-19 22:25:50 curt Exp $


#ifndef _SUPERPOLY_HXX
#define _SUPERPOLY_HXX


#ifndef __cplusplus
# error This library requires C++
#endif


#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <string>
#include <vector>

#include "polygon.hxx"

// TODO : Needs to be its own class
typedef std::vector < int > int_list;
typedef std::vector < int_list > idx_list;
typedef std::vector < double > double_list;
typedef idx_list::iterator idx_list_iterator;
typedef idx_list::const_iterator const_idx_list_iterator;

class TGPolyNodes {

private:

    idx_list poly;                  // polygon node indexes

public:

    // Constructor and destructor
    TGPolyNodes( void ) {}
    ~TGPolyNodes( void ) {}

    // Add a contour
    inline void add_contour( const int_list contour )
    {
        poly.push_back( contour );
    }

    // Get a contour
    inline int_list get_contour( const int i ) const
    {
        return poly[i];
    }

    // Delete a contour
    inline void delete_contour( const int i )
    {
        idx_list_iterator start_poly = poly.begin();
        poly.erase( start_poly + i );
    }

    // Add the specified node (index) to the polygon
    inline void add_node( int contour, int n )
    {
        if ( contour >= (int)poly.size() ) {
            // extend polygon
            int_list empty_contour;
            empty_contour.clear();
            for ( int i = 0; i < contour - (int)poly.size() + 1; ++i ) {
                poly.push_back( empty_contour );
            }
        }
        poly[contour].push_back( n );
    }

    // return size
    inline int contours() const
    {
        return poly.size();
    }
    inline int contour_size( int contour ) const
    {
        return poly[contour].size();
    }
    inline int total_size() const
    {
        int size = 0;

        for ( int i = 0; i < contours(); ++i )
            size += poly[i].size();
        return size;
    }

    // return the ith point from the specified contour
    inline int get_pt( int contour, int i ) const
    {
        return poly[contour][i];
    }

    // update the value of a point
    inline void set_pt( int contour, int i, const int n )
    {
        poly[contour][i] = n;
    }
};
// END TODO



class TGSuperPoly {

private:

std::string material;       // material/texture name
TGPolygon   poly;           // master polygon
TGPolygon   normals;        // corresponding normals
TGPolygon   texcoords;      // corresponding texture coordinates

TGPolygon   tris;           // triangulation
TGPolyNodes tri_idxs;       // triangle node indexes

point_list  face_normals;   // triangle normals
double_list face_areas;     // triangle areas
std::string flag;           // For various potential record keeping needs

public:

// Constructor and destructor
TGSuperPoly( void );
~TGSuperPoly( void );

inline std::string get_material() const
{
    return material;
}
inline void set_material( const std::string &m )
{
    material = m;
}

inline TGPolygon get_poly() const
{
    return poly;
}
inline void set_poly( const TGPolygon &p )
{
    poly = p;
}

inline TGPolygon get_normals() const
{
    return normals;
}
inline void set_normals( const TGPolygon &p )
{
    normals = p;
}

inline TGPolygon get_texcoords() const
{
    return texcoords;
}
inline void set_texcoords( const TGPolygon &p )
{
    texcoords = p;
}

inline TGPolygon get_tris() const
{
    return tris;
}
inline void set_tris( const TGPolygon &p )
{
    tris = p;
}

inline TGPolyNodes get_tri_idxs() const
{
    return tri_idxs;
}
inline void set_tri_idxs( const TGPolyNodes &p )
{
    tri_idxs = p;
}

inline Point3D get_face_normal( int tri ) const
{
    return face_normals[tri];
}

inline point_list get_face_normals() const
{
    return face_normals;
}
inline void set_face_normals( const point_list &fns )
{
    face_normals = fns;
}

inline double get_face_area( int tri ) const
{
    return face_areas[tri];
}

inline double_list get_face_areas() const
{
    return face_areas;
}
inline void set_face_areas( const double_list &fas )
{
    face_areas = fas;
}

inline std::string get_flag() const
{
    return flag;
}
inline void set_flag( const std::string f )
{
    flag = f;
}

// erase the polygon
void erase();

};


typedef std::vector < TGSuperPoly > superpoly_list;
typedef superpoly_list::iterator superpoly_list_iterator;
typedef superpoly_list::const_iterator const_superpoly_list_iterator;


#endif // _SUPERPOLY_HXX
