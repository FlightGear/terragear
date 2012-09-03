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
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
--------------------------------------------------------------------------
*/
//#ifndef _CLIPPER_HXX
//#define _CLIPPER_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <Polygon/polygon.hxx>
#include <Polygon/names.hxx>

#include <string>

#define FG_MAX_AREA_TYPES 128	// FIXME: also defined in clipper.hxx

class FGPolyList {
public:
    poly_list polys[FG_MAX_AREA_TYPES];
    TGPolygon safety_base;
};

class FGMerger {

private:

  FGPolyList polys_subject, polys_clipp, polys_out;
  
public:
  
    // Constructor
  FGMerger( void );

    // Destructor
  ~FGMerger( void );

  // Initialize Clipper (allocate and/or connect structures)
  bool init();  
  
  bool load_polys(const std::string& path, FGPolyList& clipped);
  // Merge all polygons of the given area
  void merge(FGPolyList& clipped);

  // Clip all merged polygons with the land mass one
  void clip(FGPolyList& subject, FGPolyList& clip);
  
  void write(FGPolyList& subject, std::string& file);

  inline FGPolyList get_polys_clipped() const { return polys_subject; }
  
};
