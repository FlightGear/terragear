// triangle.hxx -- "Triangle" interface class
//
// Written by Curtis Olson, started March 1999.
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: triangle.hxx,v 1.7 2004-11-19 22:25:50 curt Exp $


#ifndef _TRIANGLE_HXX
#define _TRIANGLE_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/point3d.hxx>

#include <Array/array.hxx>
#include <Main/construct.hxx>

#include <Geometry/trinodes.hxx>
#include <Geometry/trisegs.hxx>
#include <Polygon/names.hxx>
#include <Polygon/polygon.hxx>

#define REAL double
extern "C" {
#include <TriangleJRS/triangle.h>
}

#include "trieles.hxx"


class TGTriangle {

private:

    // list of nodes
    TGTriNodes in_nodes;
    TGTriNodes out_nodes;

    // list of segments
    TGTriSegments in_segs;
    TGTriSegments out_segs;

    // polygon list
    poly_list polylist[TG_MAX_AREA_TYPES];
    
    // triangle list
    triele_list elelist;

public:

    // Constructor and destructor
    TGTriangle( void );
    ~TGTriangle( void );

    // add nodes from the dem fit
    int add_nodes();

    // populate this class based on the specified gpc_polys list
    int build( const point_list& corner_list, 
	       const point_list& fit_list,
	       const TGPolyList& gpc_polys );

    // populate this class based on the specified gpc_polys list
    int rebuild( TGConstruct& c );

    // Front end triangulator for polygon list.  Allocates and builds
    // up all the needed structures for the triangulator, runs it,
    // copies the results, and frees all the data structures used by
    // the triangulator.  "pass" can be 1 or 2.  1 = first pass which
    // generates extra nodes for a better triangulation.  2 = second
    // pass after split/reassem where we don't want any extra nodes
    // generated.
    int run_triangulate( double angle, const int pass );

    inline TGTriNodes get_out_nodes() const { return out_nodes; }
    inline size_t get_out_nodes_size() const { return out_nodes.size(); }
    inline triele_list get_elelist() const { return elelist; }
    inline TGTriSegments get_out_segs() const { return out_segs; }
};


#endif // _TRIANGLE_HXX


