// genstrips.hxx -- Optimize a collection of individual triangles into
//                  strips.
//
// Written by Curtis Olson, started May 2000.
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
// $Id: genstrips.hxx,v 1.4 2004-11-19 22:25:50 curt Exp $


#ifndef _GENSTRIPS_HXX
#define _GENSTRIPS_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>

#include <vector>

#include <simgear/math/sg_types.hxx>

#include <Geometry/trieles.hxx>

typedef std::vector < int_list > opt_list;
typedef opt_list::iterator opt_list_iterator;
typedef opt_list::const_iterator const_opt_list_iterator;


// generate a set of tri-strips from a random collection of triangles.
// Note, the triangle verticies are integer indices into a vertex list
// and not actual vertex data.
opt_list tgGenStrips( const triele_list tris );


#endif // _GENSTRIPS_HXX


