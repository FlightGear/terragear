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

#include <Array/array.hxx>
#include <Geometry/tg_nodes.hxx>
#include <landcover/landcover.hxx>

#include "tglandclass.hxx"

#define FIND_SLIVERS    (1)
#define USE_ACCUMULATOR (1)


// Stage2 shared edge data
struct TGNeighborFaces {
public:
    Point3D node;

    double_list elevations;     // we'll take the average
    double_list face_areas;
    point_list  face_normals;
};

typedef std::vector < TGNeighborFaces > neighbor_face_list;
typedef neighbor_face_list::iterator neighbor_face_list_iterator;
typedef neighbor_face_list::const_iterator const_neighbor_face_list_iterator;


class TGConstruct {

private:

    // path to land-cover file (if any)
    std::string cover;

    // paths
    std::string work_base;
    std::string output_base;
    std::string share_base;

    std::vector<std::string> load_dirs;

    const static double gSnap = 0.00000001;      // approx 1 mm

    // flag indicating whether to ignore the landmass
    bool ignoreLandmass;

    // I think we should remove this
    double nudge;

    // path to the debug shapes
    std::string debug_path;

    bool debug_all;

    // list of shapes to dump during debug
    std::vector<unsigned int> debug_areas;
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

    // ocean tile?
    bool isOcean;

    // Neighbor Faces
    neighbor_face_list  neighbor_faces;

private:
    // Ocean tile or not
    void SetOceanTile() { isOcean = true; }
    bool IsOceanTile()  { return isOcean; }
    
    // Load Data
    void LoadElevationArray( void );
    int  LoadLandclassPolys( void );
    // Load Data Helpers
    bool load_poly(const std::string& path);
    void add_poly(int area, const TGPolygon &poly, std::string material);

    // Clip Data
    bool ClipLandclassPolys( void );
    // Clip Helpers
    void move_slivers( TGPolygon& in, TGPolygon& out );
    void merge_slivers( TGLandclass& clipped, poly_list& slivers_list );

    // Shared edge Matching
    void SaveSharedEdgeDataStage2( void );
    void LoadSharedEdgeDataStage2( void );
    void WriteSharedEdgeNeighboorFaces( std::ofstream& ofs_e, Point3D pt );
    void LoadSharedEdgeData( int stage );
    void LoadNeighboorEdgeDataStage1( SGBucket& b, point_list& north, point_list& south, point_list& east, point_list& west );

    void SaveSharedEdgeData( int stage );

    void ReadNeighborFaces( std::ifstream& ifs_e );
    void WriteNeighborFaces( std::ofstream& ofs_e, Point3D pt );
    TGNeighborFaces* AddNeighborFaces( Point3D node );
    TGNeighborFaces* FindNeighborFaces( Point3D node );

    // Polygon Cleaning
    void CleanClippedPolys( void );
    void FixTJunctions( void );

    // Tesselation
    void TesselatePolys( void );

    // Elevation and Flattening
    void CalcElevations( void );
    void AverageEdgeElevations( void );

    // Normals and texture coords
    void LookupNodesPerVertex( void );
    void LookupFacesPerNode( void );
    void CalcFaceNormals( void );
    void CalcPointNormals( void );
    void CalcTextureCoordinates( void );
    // Helpers
    SGVec3d   calc_normal( double area, Point3D p1, Point3D p2, Point3D p3 );
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
    bool IsDebugArea( unsigned int area );

    void WriteDebugShape( const char* layer_name, const TGShape& shape );
    void WriteDebugPoly( const char* layer_name, const char* name, const TGPolygon& poly );
    void WriteDebugPolys( const char* layer_name, const poly_list& polys );

public:
    // Constructor
    TGConstruct();

    // Destructor
    ~TGConstruct();

    void set_bucket( SGBucket b ) { bucket = b; }

    // New shared edge matching
    void SaveToIntermediateFiles( int stage );
    void LoadFromIntermediateFiles( int stage );

    // Three stage construct
    void ConstructBucketStage1();
    void ConstructBucketStage2();
    void ConstructBucketStage3();

    int      load_landcover ();
    double   measure_roughness( TGPolygon &poly );
    AreaType get_landcover_type (const LandCover &cover, double xpos, double ypos, double dx, double dy);
    void make_area( const LandCover &cover, TGPolygon *polys,
                    double x1, double y1, double x2, double y2,
                    double half_dx, double half_dy );

    // land cover file
    inline std::string get_cover () const { return cover; }
    inline void set_cover (const std::string &s) { cover = s; }

    // paths
    void set_paths( const std::string work, const std::string share, const std::string output, const std::vector<std::string> load_dirs );

#if 0
    inline std::string get_work_base() const { return work_base; }
    inline void set_work_base( const std::string s ) { work_base = s; }
    inline std::string get_output_base() const { return output_base; }
    inline void set_output_base( const std::string s ) { output_base = s; }
    inline std::string get_share_base() const { return share_base; }
    inline void set_share_base( const std::string s ) { share_base = s; }
    inline void set_load_dirs( const std::vector<std::string> ld ) { load_dirs = ld; }
#endif

    void set_options( bool ignore_lm, double n );

#if 0
    // UK grid flag
    inline bool get_useUKGrid() const { return useUKGrid; }
    inline void set_useUKGrid( const bool b ) { useUKGrid = b; }

    // Nudge
    inline void set_nudge( double n ) { nudge = n; }

    // ignore landmass flag
    inline void set_ignore_landmass( const bool b) { ignoreLandmass = b; }
#endif

    // TODO : REMOVE
    inline TGNodes* get_nodes() { return &nodes; }

    // node list in geodetic coords (with fixed elevation)
    inline point_list get_geod_nodes() const { return nodes.get_geod_nodes(); }

    // normal list (for each point) in cart coords (for smooth
    // shading)
    inline point_list get_point_normals() const { return nodes.get_normals(); }

    // Debug
    void set_debug( std::string path, std::vector<std::string> area_defs, std::vector<std::string> shape_defs );
};


#endif // _CONSTRUCT_HXX
