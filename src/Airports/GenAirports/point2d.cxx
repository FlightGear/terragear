// point2d.cxx -- 2d coordinate routines
//
// Written by Curtis Olson, started September 1998.
//
// Copyright (C) 1998  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: point2d.cxx,v 1.2 2004-11-19 22:25:49 curt Exp $
//


#include <math.h>

#include "point2d.hxx"


// convert a point from cartesian to polar coordinates
Point3D cart_to_polar_2d(const Point3D& in) {
    Point3D result( sqrt(in.x() * in.x() + in.y() * in.y()),
		    atan2(in.y(), in.x()),
		    0 );
    return result;
}


