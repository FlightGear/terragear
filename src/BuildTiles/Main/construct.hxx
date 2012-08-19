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
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>
#include <Geometry/tg_nodes.hxx>

#include <landcover/landcover.hxx>

#include "priorities.hxx"

#define USE_CLIPPER     (0)
#define FIND_SLIVERS    (1)
#define USE_ACCUMULATOR (1)

class TGShape
{
public:
    TGPolygon       clip_mask;
    bool            textured;
    superpoly_list  sps;
    texparams_list  tps;
    AreaType        area;
    unsigned int    id;

    void    GetName( char* name ) const
    {
        sprintf( name, "%s_%d", get_area_name( (AreaType)area ).c_str(), id );
    }
    
    void    SetMask( TGPolygon mask )
    {
        clip_mask = mask;
    }

    void    BuildMask( void )
    {
        TGPolygon poly;
        clip_mask.erase();

        for (unsigned int i=0; i<sps.size(); i++)
        {
            poly = sps[i].get_poly();
#ifdef USE_CLIPPER
            clip_mask = tgPolygonUnionClipper( clip_mask, poly );
#else
            clip_mask = tgPolygonUnion( clip_mask, poly );
#endif
        }
    }

    void    IntersectPolys( void )
    {
        if ( sps.size() > 1 ) {
            TGPolygon original, intersect;

            for (unsigned int i=0; i<sps.size(); i++)
            {
                original  = sps[i].get_poly();

#ifdef USE_CLIPPER
                intersect = tgPolygonIntClipper( clip_mask, original );
#else
                intersect = tgPolygonInt( clip_mask, original );
#endif

                sps[i].set_poly( intersect );
            }
        } else {
            sps[0].set_poly( clip_mask );
        }
    }
};

typedef std::vector < TGShape > shape_list;
typedef shape_list::iterator shape_list_iterator;
typedef shape_list::const_iterator const_shape_list_iterator;

class TGLandclass
{
public:
    void clear(void)
    {
        int i;

        for (i=0; i<TG_MAX_AREA_TYPES; i++) {
            shapes[i].clear();
        }
    }

    inline unsigned int area_size( unsigned int area )
    {
        return shapes[area].size();
    }
    inline unsigned int shape_size( unsigned int area, unsigned int shape )
    {
        return shapes[area][shape].sps.size();
    }

    inline void add_shape( unsigned int area, TGShape shape )
    {
        shapes[area].push_back( shape );
    }
    inline TGShape& get_shape( unsigned int area, unsigned int shape )
    {
        return shapes[area][shape];
    }

    inline TGPolygon get_mask( unsigned int area, unsigned int shape )
    {
        return shapes[area][shape].clip_mask;
    }
    inline void set_mask( unsigned int area, unsigned int shape, TGPolygon mask )
    {
        shapes[area][shape].clip_mask = mask;
    }

    inline bool get_textured( unsigned int area, unsigned int shape )
    {
        return shapes[area][shape].textured;
    }
    inline void set_textured( unsigned int area, unsigned int shape, bool t )
    {
        shapes[area][shape].textured = t;
    }


    inline TGSuperPoly& get_superpoly( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].sps[segment];
    }
    inline void set_superpoly( unsigned int area, unsigned int shape, unsigned int segment, TGSuperPoly sp )
    {
        shapes[area][shape].sps[segment] = sp;
    }

    inline TGPolygon get_poly( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].sps[segment].get_poly();
    }
    inline void set_poly( unsigned int area, unsigned int shape, unsigned int segment, TGPolygon poly )
    {
        return shapes[area][shape].sps[segment].set_poly( poly );
    }

    inline TGPolygon get_tris( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].sps[segment].get_tris();
    }
    inline void set_tris( unsigned int area, unsigned int shape, unsigned int segment, TGPolygon tris )
    {
        shapes[area][shape].sps[segment].set_tris( tris );
    }

    inline Point3D get_face_normal( unsigned int area, unsigned int shape, unsigned int segment, unsigned int tri )
    {
        return shapes[area][shape].sps[segment].get_face_normal( tri );
    }

    inline double get_face_area( unsigned int area, unsigned int shape, unsigned int segment, unsigned int tri )
    {
        return shapes[area][shape].sps[segment].get_face_area( tri );
    }

    inline std::string get_flag( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].sps[segment].get_flag();
    }

    inline std::string get_material( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].sps[segment].get_material();
    }
    inline TGTexParams& get_texparams( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].tps[segment];
    }


    inline TGPolygon get_texcoords( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].sps[segment].get_texcoords();
    }
    inline void set_texcoords( unsigned int area, unsigned int shape, unsigned int segment, TGPolygon tcs )
    {
        return shapes[area][shape].sps[segment].set_texcoords( tcs );
    }

    inline TGPolyNodes get_tri_idxs( unsigned int area, unsigned int shape, unsigned int segment )
    {
        return shapes[area][shape].sps[segment].get_tri_idxs();
    }
    inline void set_tri_idxs( unsigned int area, unsigned int shape, unsigned int segment, TGPolyNodes tis )
    {
        return shapes[area][shape].sps[segment].set_tri_idxs( tis );
    }

private:
    shape_list     shapes[TG_MAX_AREA_TYPES];
};

// forward declaration
class TGMatch;

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

    // path to the debug shapes
    std::string debug_path;

    bool debug_all;

    // list of shapes to dump during debug
    std::vector<unsigned int> debug_shapes;

    // OGR encode variables
    // For debug:
    void*     ds_id;            // If we are going to build shapefiles
    void*     l_id;             // datasource and layer IDs
    char      ds_name[128];
    char      layer_name[128];
    char      feature_name[128];

    // this bucket
    SGBucket bucket;

    // Elevation data
    TGArray array;

    // land class polygons
    TGLandclass polys_in;
    TGLandclass polys_clipped;

    // All Nodes
    TGNodes nodes;

    // SHared Edges match data
    TGMatch match;

private:
    // Load Data
    void LoadElevationArray( void );
    int  LoadLandclassPolys( void );
    // Load Data Helpers
    bool load_poly(const std::string& path);
    bool load_osgb36_poly(const std::string& path);
    void add_poly(int area, const TGPolygon &poly, std::string material);

    // Clip Data
    bool ClipLandclassPolys( void );
    // Clip Helpers
    void move_slivers( TGPolygon& in, TGPolygon& out );
    void merge_slivers( TGLandclass& clipped, poly_list& slivers_list );

    // Shared edge Matching
    void LoadSharedEdgeData( void );
    void SaveSharedEdgeData( void );    

    // Polygon Cleaning
    void CleanClippedPolys( void );
    void FixTJunctions( void );

    // Tesselation
    void TesselatePolys( void );

    // Elevation and Flattening
    void CalcElevations( void );

    // Normals and texture coords
    void LookupNodesPerVertex( void );
    void LookupFacesPerNode( void );
    void CalcFaceNormals( void );
    void CalcPointNormals( void );
    void CalcTextureCoordinates( void );
    // Helpers
    TGPolygon linear_tex_coords( const TGPolygon& tri, const TGTexParams& tp );
    TGPolygon area_tex_coords( const TGPolygon& tri );

    // Output
    void WriteBtgFile( void );
    void AddCustomObjects( void );

    // Misc
    void calc_normals( point_list& wgs84_nodes, TGSuperPoly& sp );
    double calc_tri_area( int_list& triangle_nodes );

    // debug
    bool IsDebugShape( unsigned int id );
    void WriteDebugShape( const char* layer_name, const TGShape& shape );
    void WriteDebugPoly( const char* layer_name, const char* name, const TGPolygon& poly );
    void WriteDebugPolys( const char* layer_name, const poly_list& polys );
    
public:
    // Constructor
    TGConstruct();

    // Destructor
    ~TGConstruct();

    void set_bucket( SGBucket b ) { bucket = b; }
    
    void ConstructBucket();


    void calc_gc_course_dist( const Point3D& start, const Point3D& dest, 
                              double *course, double *dist );
    double distanceSphere( const Point3D p1, const Point3D p2 );

    int      load_landcover ();
    void     add_to_polys( TGPolygon &accum, const TGPolygon &poly);
    double   measure_roughness( TGPolygon &poly );
    AreaType get_landcover_type (const LandCover &cover, double xpos, double ypos, double dx, double dy);
    void make_area( const LandCover &cover, TGPolygon *polys,
                    double x1, double y1, double x2, double y2,
                    double half_dx, double half_dy );

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

    // normal list (for each point) in cart coords (for smooth
    // shading)
    inline point_list get_point_normals() const { return nodes.get_normals(); }

    // Debug
    void set_debug( std::string path, std::vector<std::string> defs );
};


#endif // _CONSTRUCT_HXX
