// superpoly.hxx -- Manage all aspects of a rendered polygon
//
// Written by Curtis Olson, started June 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - curt@flightgear.org
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


#ifndef _SUPERPOLY_HXX
#define _SUPERPOLY_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/fg_types.hxx>

#include <string>
#include <vector>

#include <GL/gl.h>

#include "polygon.hxx"

FG_USING_STD(string);
FG_USING_STD(vector);


class FGSuperPoly {

private:

    string material;		// material/texture name
    FGPolygon poly;		// master polygon
    FGPolygon normals;		// corresponding normals
    FGPolygon texcoords;	// corresponding texture coordinates
    FGPolygon tris;		// triangulation
    GLenum tri_mode;		// GL_TRIANGLE, GL_FAN, GL_TRISTRIP, etc.

public:

    // Constructor and destructor
    FGSuperPoly( void );
    ~FGSuperPoly( void );

    inline string get_material() const { return material; }
    inline void set_material( const string &m ) { material = m; }

    inline FGPolygon get_poly() const { return poly; }
    inline void set_poly( const FGPolygon &p ) { poly = p; }

    inline FGPolygon get_normals() const { return normals; }
    inline void set_normals( const FGPolygon &p ) { normals = p; }

    inline FGPolygon get_texcoords() const { return texcoords; }
    inline void set_texcoords( const FGPolygon &p ) { texcoords = p; }

    inline FGPolygon get_tris() const { return tris; }
    inline void set_tris( const FGPolygon &p ) { tris = p; }

    inline GLenum get_tri_mode() const { return tri_mode; }
    inline void set_tri_mode( const GLenum &m ) { tri_mode = m; }

    // erase the polygon
    void erase();

};


typedef vector < FGSuperPoly > superpoly_list;
typedef superpoly_list::iterator superpoly_list_iterator;
typedef superpoly_list::const_iterator const_superpoly_list_iterator;


#endif // _SUPERPOLY_HXX
