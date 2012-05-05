// names.hxx -- process shapefiles names
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
// $Id: names.hxx,v 1.16 2007-10-31 15:05:13 curt Exp $


#ifndef _NAMES_HXX
#define _NAMES_HXX


#include <simgear/compiler.h>

#include <string>

inline static bool is_ocean_area( const std::string &area )
{
    return area == "Ocean" || area == "Bay  Estuary or Ocean";
}

inline static bool is_void_area( const std::string &area )
{
    return area == "Void Area";
}

inline static bool is_null_area( const std::string& area )
{
    return area == "Null";
}

#endif // _NAMES_HXX

