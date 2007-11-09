// construct.hxx -- Class to manage the primary data used in the
//                  construction process
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: construct.hxx,v 1.13 2004-11-19 22:25:49 curt Exp $


#ifndef _CONSTRUCT_HXX
#define _CONSTRUCT_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


// Maximum nodes per tile
#define TG_MAX_NODES 4000


#include <simgear/compiler.h>

#include STL_STRING

#include <simgear/bucket/newbucket.hxx>

#include <Geometry/trinodes.hxx>
#include <Geometry/trisegs.hxx>
#include <Clipper/priorities.hxx>

#include <Clipper/clipper.hxx>
#include <Triangulate/trieles.hxx>

SG_USING_STD(string);


typedef vector < int_list > belongs_to_list;
typedef belongs_to_list::iterator belongs_to_list_iterator;
typedef belongs_to_list::const_iterator belongs_to_list_tripoly_iterator;


class TGConstruct {

private:

    // path to land-cover file (if any)
    string cover;

    // paths
    string work_base;
    string output_base;

    // flag indicating whether to align texture coords within the UK
    // with the UK grid
    bool useUKGrid;

    // detail level constraints
    int min_nodes;
    int max_nodes;

    // this bucket
    SGBucket bucket;

    // clipped polygons (gpc format)
    TGPolyList clipped_polys;

    // Fixed elevations (from polygon input data with z values
    // pre-specified)
    TGTriNodes fixed_elevations;

    // raw node list (after triangulation)
    TGTriNodes tri_nodes;

    // node list in geodetic coords (with fixed elevation)
    point_list geod_nodes;

    // node list in cartesian coords (wgs84 model)
    point_list wgs84_nodes;

    // triangle elements (after triangulation)
    triele_list tri_elements;

    // edge segments (after triangulation)
    TGTriSegments tri_segs;

    // for each node, a list of triangle indices that contain this node
    belongs_to_list reverse_ele_lookup;

    // face normal list (for flat shading)
    point_list face_normals;

    // normal list (for each point) in cart coords (for smooth
    // shading)
    point_list point_normals;

public:

    // Constructor
    TGConstruct();

    // Destructor
    ~TGConstruct();
    
    // land cover file
    inline string get_cover () const { return cover; }
    inline void set_cover (const string &s) { cover = s; }

    // paths
    inline string get_work_base() const { return work_base; }
    inline void set_work_base( const string s ) { work_base = s; }
    inline string get_output_base() const { return output_base; }
    inline void set_output_base( const string s ) { output_base = s; }

    // UK grid flag
    inline bool get_useUKGrid() const { return useUKGrid; }
    inline void set_useUKGrid( const bool b ) { useUKGrid = b; }

    // detail level constraints
    inline int get_min_nodes() const { return min_nodes; }
    inline void set_min_nodes( const int n ) { min_nodes = n; }
    inline int get_max_nodes() const { return max_nodes; }
    inline void set_max_nodes( const int n ) { max_nodes = n; }

    // this bucket
    inline SGBucket get_bucket() const { return bucket; } 
    inline void set_bucket( const SGBucket b ) { bucket = b; } 

    // clipped polygons
    inline TGPolyList get_clipped_polys() const { return clipped_polys; }
    inline void set_clipped_polys( TGPolyList p ) { clipped_polys = p; }

    // Fixed elevations (from polygon input data with z values
    // pre-specified)
    inline TGTriNodes get_fixed_elevations() const { return fixed_elevations; }
    inline void set_fixed_elevations( TGTriNodes n ) { fixed_elevations = n; }

    // node list (after triangulation)
    inline TGTriNodes get_tri_nodes() const { return tri_nodes; }
    inline void set_tri_nodes( TGTriNodes n ) { tri_nodes = n; }

    // triangle elements (after triangulation)
    inline triele_list get_tri_elements() const { return tri_elements; }
    inline void set_tri_elements( triele_list e ) { tri_elements = e; }
    inline void set_tri_attribute( int num, AreaType a ) {
        tri_elements[num].set_attribute( a );
    }

    // edge segments (after triangulation)
    inline TGTriSegments get_tri_segs() const { return tri_segs; }
    inline void set_tri_segs( TGTriSegments s ) { tri_segs = s; }

    // node list in geodetic coords (with fixed elevation)
    inline point_list get_geod_nodes() const { return geod_nodes; }
    inline void set_geod_nodes( point_list n ) { geod_nodes = n; }

    // node list in cartesian coords (wgs84 model)
    inline point_list get_wgs84_nodes() const { return wgs84_nodes; }
    inline void set_wgs84_nodes( point_list n ) { wgs84_nodes = n; }

    // for each node, a list of triangle indices that contain this node
    inline belongs_to_list get_reverse_ele_lookup() const { 
	return reverse_ele_lookup;
    }
    inline void set_reverse_ele_lookup( belongs_to_list r ) {
	reverse_ele_lookup = r;
    }

    // face normal list (for flat shading)
    inline point_list get_face_normals() const { return face_normals; }
    inline void set_face_normals( point_list n ) { face_normals = n; }

    // normal list (for each point) in cart coords (for smooth
    // shading)
    inline point_list get_point_normals() const { return point_normals; }
    inline void set_point_normals( point_list n ) { point_normals = n; }
};


#endif // _CONSTRUCT_HXX
