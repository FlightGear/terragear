// priorities.hxx -- manage material priorities
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: names.hxx,v 1.14 2005-09-28 16:43:18 curt Exp $
 

#ifndef _PRIORITIES_HXX
#define _PRIORITIES_HXX


#include <simgear/compiler.h>

#include <string>

typedef unsigned int AreaType;
int load_area_types( const std::string& filename );
bool is_hole_area(AreaType areaType);
bool is_landmass_area(AreaType areaType);
bool is_island_area(AreaType areaType);
bool is_water_area(AreaType areaType);
bool is_lake_area(AreaType areaType);
bool is_stream_area(AreaType areaType);
bool is_road_area(AreaType areaType);
bool is_ocean_area(AreaType areaType);
AreaType get_sliver_target_area_type();
AreaType get_default_area_type();

// return area type from text name
AreaType get_area_type( const std::string &area );

// return text form of area name
std::string get_area_name( AreaType area );

#endif // _PRIORITIES_HXX

