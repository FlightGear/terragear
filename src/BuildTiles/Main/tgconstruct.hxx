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

#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGQueue.hxx>

#include <Array/array.hxx>
#include <terragear/tg_nodes.hxx>
#include <landcover/landcover.hxx>

#include "tglandclass.hxx"
#include "priorities.hxx"

#define FIND_SLIVERS    (0)

// Stage2 shared edge data
struct TGNeighborFaces {
public:
    SGGeod      node;

    double_list elevations;     // we'll take the average
    double_list face_areas;
    std::vector<SGVec3f> face_normals;
};

typedef std::vector < TGNeighborFaces > neighbor_face_list;
typedef neighbor_face_list::iterator neighbor_face_list_iterator;
typedef neighbor_face_list::const_iterator const_neighbor_face_list_iterator;

class TGConstruct : public SGThread
{
public:
    // Constructor
    TGConstruct( const TGAreaDefinitions& areas, unsigned int s, SGLockedQueue<SGBucket>& q, SGMutex* l );

    // Destructor
    ~TGConstruct();

    // New shared edge matching
    void SaveToIntermediateFiles( int stage );
    void LoadFromIntermediateFiles( int stage );

#if 0
    int      load_landcover ();
    double   measure_roughness( tgContour &contour );
    AreaType get_landcover_type (const LandCover &cover, double xpos, double ypos, double dx, double dy);
    void make_area( const LandCover &cover, tgpolygon_list& polys,
                    double x1, double y1, double x2, double y2,
                    double half_dx, double half_dy );

    // land cover file
    inline std::string get_cover () const { return cover; }
    inline void set_cover (const std::string &s) { cover = s; }
#endif

    // paths
    void set_paths( const std::string& work, const std::string& share, const std::string& match, 
                    const std::string& output, const std::vector<std::string>& load_dirs );
    void set_options( bool ignore_lm, double n );

    // TODO : REMOVE
    inline TGNodes* get_nodes() { return &nodes; }

    // node list in geodetic coords (with fixed elevation)
    inline void get_geod_nodes( std::vector<SGGeod>& points ) const { nodes.get_geod_nodes( points ); }

    // normal list (for each point) in cart coords (for smooth shading)
    inline void get_point_normals( std::vector<SGVec3f>& normals ) const { nodes.get_normals( normals ); }

    // Debug
    void set_debug( std::string path, std::vector<std::string> area_defs, std::vector<std::string> shape_defs );

    void CreateMatchedEdgeFiles( std::vector<SGBucket>& bucketList );
    
private:
    virtual void run();

    // Ocean tile or not
    bool IsOceanTile()  { return isOcean; }

    // Load Data
    void LoadElevationArray( bool add_nodes );
    int  LoadLandclassPolys( void );
    bool CheckMatchingNode( SGGeod& node, bool road, bool fixed );
    
    SGGeod GetNearestNodeLatitude( const SGGeod& node, const std::vector<SGGeod>& selection );
    SGGeod GetNearestNodeLongitude( const SGGeod& node, const std::vector<SGGeod>& selection );

    // Clip Data
    bool ClipLandclassPolys( void );

    // Clip Helpers
//    void move_slivers( TGPolygon& in, TGPolygon& out );
//    void merge_slivers( TGLandclass& clipped, tgcontour_list& sliver_list );

    // Shared edge Matching
    void SaveSharedEdgeData( int stage );
    void LoadSharedEdgeData( int stage );
    void LoadMatchedEdgeFiles();

    void LoadNeighboorEdgeDataStage1( SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west );
    void LoadNeighboorMatchDataStage1( SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west );

    void ReadNeighborFaces( gzFile& fp );
    void WriteNeighborFaces( gzFile& fp, const SGGeod& pt ) const;
    TGNeighborFaces* AddNeighborFaces( const SGGeod& node );
    TGNeighborFaces* FindNeighborFaces( const SGGeod& node );

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
    SGVec3f calc_normal( double area, const SGVec3d& p1, const SGVec3d& p2, const SGVec3d& p3 ) const;

    // Output
    void WriteBtgFile( void );
    void AddCustomObjects( void );

    // Misc
    void calc_normals( std::vector<SGGeod>& geod_nodes, std::vector<SGVec3d>& wgs84_nodes, tgPolygon& sp );

    // debug
    void get_debug( void );
    bool IsDebugShape( unsigned int id );
    bool IsDebugArea( unsigned int area );

private:
    TGAreaDefinitions const& area_defs;
    
    // construct stage to perform
    SGLockedQueue<SGBucket>& workQueue;
    unsigned int total_tiles;
    unsigned int stage;
    std::vector<SGGeod> nm_north, nm_south, nm_east, nm_west;

    // path to land-cover file (if any)
    std::string cover;

    // paths
    std::string work_base;
    std::string share_base;
    std::string match_base;
    std::string output_base;

    std::vector<std::string> load_dirs;

    const static double gSnap;      // approx 1 mm

    // flag indicating whether to ignore the landmass
    bool ignoreLandmass;

    // I think we should remove this
    double nudge;

    // path to the debug shapes
    std::string debug_path;

    bool debug_all;

    // list of debug definitions (for a whole tgconstruct run
    std::vector<std::string> debug_area_defs;
    std::vector<std::string> debug_shape_defs;

    // list of shapes to dump during debug (for a single tile)
    std::vector<unsigned int> debug_areas;
    std::vector<unsigned int> debug_shapes;

    // OGR encode variables for debug:
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

    unsigned int num_areas;

    // Neighbor Faces
    neighbor_face_list  neighbor_faces;
    
    // file lock
    SGMutex*    lock;
};

#endif // _CONSTRUCT_HXX
