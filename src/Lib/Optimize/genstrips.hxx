// genstrips.hxx -- Optimize a collection of individual triangles into
//                  strips.
//
// Written by Curtis Olson, started May 2000.
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


#ifndef _GENSTRIPS_HXX
#define _GENSTRIPS_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>

#include <vector>

#include <simgear/math/fg_types.hxx>

#include <Triangulate/trieles.hxx>

FG_USING_STD(vector);


typedef vector < int_list > opt_list;
typedef opt_list::iterator opt_list_iterator;
typedef opt_list::const_iterator const_opt_list_iterator;


// generate a set of tri-strips from a random collection of triangles.
// Note, the triangle verticies are integer indices into a vertex list
// and not actual vertex data.
opt_list tgGenStrips( const triele_list tris );


#endif // _GENSTRIPS_HXX


