// simple_clip.hxx -- simplistic polygon clipping routine.  Returns
//                    the portion of a polygon that is above or below
//                    a horizontal line of y = a.  Only works with
//                    single contour polygons (no holes.)
//
// Written by Curtis Olson, started August 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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
// $Id$


#ifndef _SIMPLE_CLIP_HXX
#define _SIMPLE_CLIP_HXX


#include "polygon.hxx"


// side type
enum fgSideType {
    Above = 0,
    Below = 1
};


// simple polygon clipping routine.  Returns the portion of a polygon
// that is above and below a horizontal line of y = a.  Clips
// multi-contour polygons individually and then reassembles the
// results.  Doesn't work with holes.  Returns true if routine thinks
// it was successful.

TGPolygon horizontal_clip( const TGPolygon& in, const double y, 
		      const fgSideType side );


#endif // _SIMPLE_CLIP_HXX


