// gshhs_split.hxx -- split a gshhs polygon
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
// $Id: gshhs_split.hxx,v 1.3 2004-11-19 22:25:51 curt Exp $


#ifndef _GSHHS_SPLIT_HXX
#define _GSHHS_SPLIT_HXX


#include <Polygon/names.hxx>
#include <Polygon/polygon.hxx>


// process shape front end ... split shape into lon = -180 ... 180,
// -360 ... -180, and 180 ... 360 ... shift the offset sections and
// process each separately
void split_and_shift_chunk( const string& path, AreaType area, 
			    const TGPolygon& shape );


// process a large shape through my crude polygon splitter to reduce
// the polygon sizes before handing off to gpc
void gshhs_split_polygon( const string& path, AreaType area, TGPolygon& shape,
			  const double min, const double max );


#endif // _GSHHS_SPLIT_HXX


