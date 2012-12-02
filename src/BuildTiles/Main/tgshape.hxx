// TGShape.hxx -- Class to handle polygons shapes generated in ogr-decode
//                A shape may consist of many polygons when it is generated
//                from a polyline.  They are kept together to speed up clipping
//                but also must be represented as seperate polygons in order to
//                keep track of textur parameters.
//
// Written by Curtis Olson, started May 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: construct.hxx,v 1.13 2004-11-19 22:25:49 curt Exp $

#ifndef _TGSHAPE_HXX
#define _TGSHAPE_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#define TG_MAX_AREA_TYPES       128

#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/io/lowlevel.hxx>

#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>

#include "priorities.hxx"

class TGShape
{
public:
    tgpolygon_list  polys;
    tgPolygon       mask;
    bool            textured;
    AreaType        area;
    unsigned int    id;

    void GetName( char* name ) const;
    void SetMask( const tgPolygon& mask );
    void BuildMask( void );
    void IntersectPolys( void );
    void SaveToGzFile( gzFile& fp );
    void LoadFromGzFile( gzFile& fp );

    // Friend for output to stream
    friend std::ostream& operator<< ( std::ostream&, const TGShape& );
};

typedef std::vector < TGShape > shape_list;
typedef shape_list::iterator shape_list_iterator;
typedef shape_list::const_iterator const_shape_list_iterator;

#endif // _TGSHAPE_HXX