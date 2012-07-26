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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: construct.hxx,v 1.13 2004-11-19 22:25:49 curt Exp $


#ifndef _CONSTRUCT_HXX
#define _CONSTRUCT_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#define TG_MAX_AREA_TYPES       128

#include <string>
#include <vector>

#include <simgear/compiler.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

#include <Array/array.hxx>

#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>
#include <Geometry/tg_nodes.hxx>

#include <landcover/landcover.hxx>

// TO REMOVE
#include <Geometry/trieles.hxx>
#include <Geometry/trinodes.hxx>
#include <Geometry/trisegs.hxx>
// TO REMOVE

#include "priorities.hxx"

typedef std::vector < int_list > belongs_to_list;
typedef belongs_to_list::iterator belongs_to_list_iterator;
typedef belongs_to_list::const_iterator belongs_to_list_tripoly_iterator;

class TGPolyList
{
public:
    superpoly_list superpolys[TG_MAX_AREA_TYPES];
    texparams_list texparams[TG_MAX_AREA_TYPES];
    TGPolygon safety_base;

    void clear(void)
    {
        int i;

        for (i=0; i<TG_MAX_AREA_TYPES; i++) {
            superpolys[i].clear();
        }

        for (i=0; i<TG_MAX_AREA_TYPES; i++) {
            texparams[i].clear();
        }

        safety_base.erase();
    }
};

class TGConstruct {

private:

    // path to land-cover file (if any)
    std::string cover;

    // paths
    std::string work_base;
    std::string output_base;

    std::vector<std::string> load_dirs;

    // flag indicating whether to align texture coords within the UK
    // with the UK grid
    bool useUKGrid;

    double nudge;

    // flag indicating whether this is a rebuild and Shared edge
    // data should only be used for fitting, but not rewritten
    bool writeSharedEdges;
    
    // flag indicating whether the shared edge data of the
    // tile to be built should be used in addition to neighbour data
    bool useOwnSharedEdges;
    
    // flag indicating whether to ignore the landmass
    bool ignoreLandmass;

    // this bucket
    SGBucket bucket;

    // Elevation data
    TGArray array;

    // land class polygons
    TGPolyList polys_in;
    TGPolyList polys_clipped;

    // All Nodes
    TGNodes nodes;

    // TODO : Add to superpoly
    // face normal list (for flat shading)
    point_list face_normals;

private:
    void LookupNodesPerVertex( void );
    void LookupFacesPerNode( void );
    void CalcFaceNormals( void );
    void CalcPointNormals( void );

    // Should be in superpoly?
    void calc_normals( point_list& wgs84_nodes, TGSuperPoly& sp );

    // Where should this be?  Geometry library, I think...
    double calc_tri_area( int_list& triangle_nodes );

public:

    // Constructor
    TGConstruct();

    // Destructor
    ~TGConstruct();
    
    void construct_bucket( SGBucket b );
    bool load_array();
    int  load_polys();
    bool load_poly(const std::string& path);
    bool load_osgb36_poly(const std::string& path);
    void add_poly(int area, const TGPolygon &poly, std::string material);

    void move_slivers( TGPolygon& in, TGPolygon& out );
    void merge_slivers( TGPolyList& clipped, poly_list& slivers_list );

    bool clip_all(const point2d& min, const point2d& max);

    void add_intermediate_nodes(void);
    void clean_clipped_polys(void);

    void calc_gc_course_dist( const Point3D& start, const Point3D& dest, 
                              double *course, double *dist );
    double distanceSphere( const Point3D p1, const Point3D p2 );
    void   fix_point_heights();

    TGPolygon linear_tex_coords( const TGPolygon& tri, const TGTexParams& tp );
    TGPolygon area_tex_coords( const TGPolygon& tri );

    int      load_landcover ();
    void     add_to_polys( TGPolygon &accum, const TGPolygon &poly);
    double   measure_roughness( TGPolygon &poly );
    AreaType get_landcover_type (const LandCover &cover, double xpos, double ypos, double dx, double dy);
    void make_area( const LandCover &cover, TGPolygon *polys,
                    double x1, double y1, double x2, double y2,
                    double half_dx, double half_dy );

    void do_custom_objects(void);

    // land cover file
    inline std::string get_cover () const { return cover; }
    inline void set_cover (const std::string &s) { cover = s; }

    // paths
    inline std::string get_work_base() const { return work_base; }
    inline void set_work_base( const std::string s ) { work_base = s; }
    inline std::string get_output_base() const { return output_base; }
    inline void set_output_base( const std::string s ) { output_base = s; }
    inline void set_load_dirs( const std::vector<std::string> ld ) { load_dirs = ld; }

    // UK grid flag
    inline bool get_useUKGrid() const { return useUKGrid; }
    inline void set_useUKGrid( const bool b ) { useUKGrid = b; }
    
    // Nudge
    inline void set_nudge( double n ) { nudge = n; }

    // shared edge write flag
    inline void set_write_shared_edges( const bool b ) { writeSharedEdges = b; }
    
    // own shared edge use flag
    inline void set_use_own_shared_edges( const bool b ) { useOwnSharedEdges = b; }
    
    // ignore landmass flag
    inline void set_ignore_landmass( const bool b) { ignoreLandmass = b; }

    // TODO : REMOVE
    inline TGNodes* get_nodes() { return &nodes; }

    // node list in geodetic coords (with fixed elevation)
    inline point_list get_geod_nodes() const { return nodes.get_geod_nodes(); }

    // face normal list (for flat shading)
    inline point_list get_face_normals() const { return face_normals; }
    inline void set_face_normals( point_list n ) { face_normals = n; }

    // normal list (for each point) in cart coords (for smooth
    // shading)
    inline point_list get_point_normals() const { return nodes.get_normals(); }
};


#endif // _CONSTRUCT_HXX
