/*
-------------------------------------------------------------------------
 Library for the clipper of Flight Gear base scenery data
 Losely based on the library libClipper by Curtis Olson

 Written by Alexei Novikov, Oct. 1999.

 Copyright (C) 1999 Alexei Novikov, anovikov@heron.itep.ru

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
--------------------------------------------------------------------------
*/
//#ifndef _CLIPPER_HXX
//#define _CLIPPER_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/fg_types.hxx>

#include <Polygon/polygon.hxx>
#include <Polygon/names.hxx>

#include STL_STRING
#include <vector>

FG_USING_STD(string);
FG_USING_STD(vector);

#define FG_MAX_AREA_TYPES 20

class FGPolyList {
public:
    poly_list polys[FG_MAX_AREA_TYPES];
    FGPolygon safety_base;
};

class FGMerger {

private:

  // gpc_vertex_list v_list;
  // static gpc_polygon poly;
  FGPolyList polys_subject, polys_clipp, polys_out;
  
public:
  
    // Constructor
  FGMerger( void );

    // Destructor
  ~FGMerger( void );

  // Initialize Clipper (allocate and/or connect structures)
  bool init();  
  
  bool load_polys(const string& path, FGPolyList& clipped);
  // Merge all polygons of the given area
  void merge(FGPolyList& clipped);

  // Clip all merged polygons with the land mass one
  void clip(FGPolyList& subject, FGPolyList& clip);
  
  void write(FGPolyList& subject, string& file);

  inline FGPolyList get_polys_clipped() const { return polys_subject; }
  
};
