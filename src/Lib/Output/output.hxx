// output.hxx -- routines to output a polygon model of an airport
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999 - 2000  Curtis L. Olson  - curt@flightgear.org
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
//


#ifndef _TG_OUTPUT_HXX
#define _TG_OUTPUT_HXX

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <plib/sg.h>

#include <simgear/compiler.h>

#include <stdio.h>
#include <time.h>

#include <list>
#include STL_STRING

#include <simgear/math/sg_types.hxx>

#include <Polygon/polygon.hxx>


void write_polygon( const FGPolygon& poly, const string& base );

// update index
void write_index(const string& base, const SGBucket& b, const string& name);

void write_boundary( const string& base, const SGBucket& b, 
		     const FGPolygon& bounds, long int p_index );

#endif // _TG_OUTPUT_HXX
