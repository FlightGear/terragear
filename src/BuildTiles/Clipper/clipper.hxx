// clipper.hxx -- top level routines to take a series of arbitrary areas and
//                produce a tight fitting puzzle pieces that combine to make a
//                tile
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
// $Id: clipper.hxx,v 1.11 2007-08-15 14:36:52 curt Exp $
 


#ifndef _CLIPPER_HXX
#define _CLIPPER_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>

#include <Geometry/trinodes.hxx>
#include <Polygon/polygon.hxx>
#include <Polygon/point2d.hxx>

#include STL_STRING
#include <vector>

SG_USING_STD(string);
SG_USING_STD(vector);


#define TG_MAX_AREA_TYPES 128	// FIXME also defined in
                                // MergerClipper/clipper.hxx
#define EXTRA_SAFETY_CLIP


class TGPolyList
{
public:
    poly_list polys[TG_MAX_AREA_TYPES];
    TGPolygon safety_base;
};


class TGClipper 
{

private:

    TGPolyList polys_in, polys_clipped;
    TGTriNodes fixed_elevations;

public:

    // Constructor.
    TGClipper (void);

    // Destructor.
    ~TGClipper (void);

    // Initialize Clipper (allocate and/or connect structures.)
    bool init();

    // Load a polygon definition file
    bool load_polys(const string& path);

    // Load an Osgb36 polygon definition file
    bool load_osgb36_polys(const string& path);

    // Add a polygon.
    void add_poly(int area, const TGPolygon &poly);

    // Remove any slivers from in polygon and move them to out
    // polygon.
    void move_slivers( TGPolygon& in, TGPolygon& out );

    // For each sliver contour, see if a union with another polygon
    // yields a polygon with no increased contours (i.e. the sliver is
    // adjacent and can be merged.)  If so, replace the clipped
    // polygon with the new polygon that has the sliver merged in.
    void merge_slivers( TGPolyList& clipped, TGPolygon& slivers );
    
    // Do actual clipping work.
    bool clip_all(const point2d& min, const point2d& max);

    // Return output poly list
    inline TGPolyList get_polys_clipped() const { return polys_clipped; }

    // Return the fixed elevation points list
    inline TGTriNodes get_fixed_elevations() const { return fixed_elevations; }
};


#endif // _CLIPPER_HXX


