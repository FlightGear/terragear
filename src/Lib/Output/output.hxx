// output.hxx -- routines to output a polygon model of an airport
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999 - 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: output.hxx,v 1.5 2004-11-19 22:25:50 curt Exp $
//


#ifndef _TG_OUTPUT_HXX
#define _TG_OUTPUT_HXX

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <simgear/compiler.h>

#include <stdio.h>
#include <time.h>

#include <list>
#include <string>

#include <simgear/math/sg_types.hxx>

#include <Polygon/polygon.hxx>


void write_polygon( const TGPolygon& poly, const std::string& base );

// update index file (list of objects to be included in final scenery build)
void write_index( const std::string& base, const SGBucket& b, const std::string& name );

// update index file (list of shared objects to be included in final
// scenery build)
void write_index_shared( const std::string &base, const SGBucket &b,
                         const Point3D &p, const std::string& name,
                         const double &heading );

// update index file (list of shared objects to be included in final
// scenery build)
void write_object_sign( const std::string &base, const SGBucket &b,
                         const Point3D &p, const std::string& sign,
                         const double &heading, const int &size );

void write_boundary( const std::string& base, const SGBucket& b, 
		     const TGPolygon& bounds, long int p_index );

#endif // _TG_OUTPUT_HXX
