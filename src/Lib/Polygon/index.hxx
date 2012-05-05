// index.cxx -- routines to handle a unique/persistant integer polygon index
//
// Written by Curtis Olson, started February 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: index.hxx,v 1.4 2004-11-19 22:25:50 curt Exp $


#ifndef _INDEX_HXX
#define _INDEX_HXX


#include <simgear/compiler.h>

#include <string>

// initialize the unique polygon index counter stored in path
bool poly_index_init( std::string path );

// increment the persistant counter and return the next poly_index
long int poly_index_next();



#endif // _INDEX_HXX


