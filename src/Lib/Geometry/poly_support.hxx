// poly_support.hxx -- additional supporting routines for the FGPolygon class
//                     specific to the object building process.
//
// Written by Curtis Olson, started October 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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


#ifndef _POLY_SUPPORT_HXX
#define _POLY_SUPPORT_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/fg_types.hxx>

#include <Polygon/polygon.hxx>

#include "trinodes.hxx"


// calculate some "arbitrary" point inside the specified contour for
// assigning attribute areas.  This requires data structures outside
// of "FGPolygon" which is why it is living over here in "Lib/Build"

Point3D calc_point_inside( const FGPolygon& p, const int contour, 
			   const FGTriNodes& trinodes );

void calc_points_inside( const FGPolygon& p );


#endif // _POLY_SUPPORT_HXX


