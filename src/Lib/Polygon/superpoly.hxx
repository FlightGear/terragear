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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
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

#ifdef _MSC_VER
#  include <windows.h>
#endif
#include <GL/gl.h>

#include "polygon.hxx"

SG_USING_STD(string);
SG_USING_STD(vector);


class TGSuperPoly {

private:

    string material;		// material/texture name
    TGPolygon poly;		// master polygon
    TGPolygon normals;		// corresponding normals
    TGPolygon texcoords;	// corresponding texture coordinates
    TGPolygon tris;		// triangulation
    GLenum tri_mode;		// GL_TRIANGLE, GL_FAN, GL_TRISTRIP, etc.
    string flag;         // For various potential record keeping needs

public:

    // Constructor and destructor
    TGSuperPoly( void );
    ~TGSuperPoly( void );

    inline string get_material() const { return material; }
    inline void set_material( const string &m ) { material = m; }

    inline TGPolygon get_poly() const { return poly; }
    inline void set_poly( const TGPolygon &p ) { poly = p; }

    inline TGPolygon get_normals() const { return normals; }
    inline void set_normals( const TGPolygon &p ) { normals = p; }

    inline TGPolygon get_texcoords() const { return texcoords; }
    inline void set_texcoords( const TGPolygon &p ) { texcoords = p; }

    inline TGPolygon get_tris() const { return tris; }
    inline void set_tris( const TGPolygon &p ) { tris = p; }

    inline GLenum get_tri_mode() const { return tri_mode; }
    inline void set_tri_mode( const GLenum &m ) { tri_mode = m; }

    inline string get_flag() const { return flag; }
    inline void set_flag( const string f ) { flag = f; }

    // erase the polygon
    void erase();

};


typedef vector < TGSuperPoly > superpoly_list;
typedef superpoly_list::iterator superpoly_list_iterator;
typedef superpoly_list::const_iterator const_superpoly_list_iterator;


#endif // _SUPERPOLY_HXX
