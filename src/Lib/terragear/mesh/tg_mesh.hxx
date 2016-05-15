#ifndef __TG_MESH_HXX__
#define __TG_MESH_HXX__

#include <ogrsf_frmts.h>

#include <terragear/polygon_set/tg_polygon_def.hxx>
#include <terragear/polygon_set/tg_polygon_set.hxx>
#include <terragear/tg_array.hxx>
#include <terragear/tg_mutex.hxx>

#include "tg_mesh_def.hxx"

#include "tg_mesh_arrangement.hxx"
#include "tg_mesh_triangulation.hxx"
#include "tg_mesh_polyhedral_surface.hxx"

// next steps :
//
// 1) tg-construct stage3
// 2) load stage 2 & arrangement stage1
// 3) face normals
// 4) point normals
// 5) texture
// 6) output
// 7) yay

class tgMesh
{
public:
    tgMesh() : meshArrangement(this), meshTriangulation(this), meshSurface(this) {};

    void initDebug( const std::string& dbgRoot );
    void initPriorities( const std::vector<std::string>& priorityNames );
    void setLock( tgMutex* l ) { lock = l; }
    void clipAgainstBucket( const SGBucket& bucket );

    void clear( void );
    bool empty( void );

    void addPoly( unsigned int priority, const tgPolygonSet& poly );
    void addPolys( unsigned int priority, const tgPolygonSetList& polys );
    void addPoints( const std::vector<cgalPoly_Point>& points );

    tgPolygonSet join( unsigned int priority, const tgPolygonSetMeta& meta );

    void generate( void );

    bool loadStage1( const std::string& path, const SGBucket& b );
    void calcElevation( const std::string& basePath );

    void loadStage2( const std::string& path, const SGBucket& b );
    void calcFaceNormals( void );

    void toShapefiles( const char* dataset ) const;

    void save( const std::string& path ) const;
    void save2( const std::string& path ) const;

    std::string getDebugPath( void ) { return debugPath; }
    SGBucket    getBucket( void )    { return b; }

    friend class tgMeshArrangement;
    friend class tgMeshTriangulation;

private:
    tgArray* loadElevationArray( const std::string& demBase, const SGBucket& bucket );

    void saveIncidentFaces( const std::string& path, const char* layer, const std::vector<meshTriVertexHandle>& vertexes ) const;

    typedef enum {
        LAYER_FIELDS_NONE,
        LAYER_FIELDS_ARR,
        LAYER_FIELDS_TDS_VERTEX,
        LAYER_FIELDS_TDS_FACE
    } MeshLayerFields;

    GDALDataset* openDatasource( const std::string& datasource_name ) const;
    OGRLayer*    openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, MeshLayerFields lf, const char* layer_name ) const;

    tgMeshArrangement               meshArrangement;
    tgMeshTriangulation             meshTriangulation;
    tgMeshPolyhedralSurface         meshSurface;
    SGBucket                        b;
    bool                            clipBucket;
    tgMutex*                        lock;
    std::string                     debugPath;
};

#endif /* __TG_MESH_HXX__ */