#ifndef __TG_MESH_HXX__
#define __TG_MESH_HXX__

#include <ogrsf_frmts.h>

#include <terragear/polygon_set/tg_polygon_def.hxx>
#include <terragear/polygon_set/tg_polygon_set.hxx>

#include "tg_mesh_def.hxx"

struct tgMeshFaceMeta
{
public:
    tgMeshFaceMeta( meshArrFaceConstHandle h, const tgPolygonSetMeta& m ) : face(h), meta(m) {}
    
    meshArrFaceConstHandle  face;
    tgPolygonSetMeta        meta;
};

class tgMesh
{
public:
    void initDebug( const std::string& dbgRoot );
    void initPriorities( const std::vector<std::string>& priorityNames );
    void setLock( SGMutex* l ) { lock = l; }
    void clipAgainstBucket( const SGBucket& bucket );

    void clear( void );
    bool empty( void );
    
    void addPoly( unsigned int priority, const tgPolygonSet& poly );
    void addPolys( unsigned int priority, const tgPolygonSetList& polys );
    void addPoints( const std::vector<cgalPoly_Point>& points );
    
    tgPolygonSet join( unsigned int priority, const tgPolygonSetMeta& meta );
    
    void generate( void );
    
    void toShapefiles( const char* dataset ) const;
    
    void save( const std::string& path ) const;

private:
    typedef enum {
        SRC_POINT_OK        = 0,
        SRC_POINT_PROJECTED = 1,
        SRC_POINT_DELETED   = 2
    } SrcPointOp_e;
    
    void clipPolys( void );
    void cleanArrangement( void );
    void arrangePolys( void );
    void arrangementInsert( std::vector<tgPolygonSet>::iterator pit );
    
    void doClusterEdges( const tgCluster& cluster );    
    SrcPointOp_e checkPointNearEdge( const cgalPoly_Point& pt, meshArrFaceConstHandle fh, cgalPoly_Point& projPt );
    void doProjectPointsToEdges( const tgCluster& cluster );
    void doRemoveAntenna( void );
    void doSnapRound( void );
    
    meshArrFaceConstHandle findPolyFace( meshArrFaceConstHandle f ) const;
    meshArrFaceConstHandle findMeshFace( const meshArrPoint& pt) const;
    meshArrFaceConstHandle findMeshFace( const meshTriPoint& pt) const;
    
    void constrainedTriangulate(void);
    void clearDomains(void);
    void markDomains(void);
    void markDomains(meshTriFaceHandle start, meshArrFaceConstHandle face, std::list<meshTriCDTPlus::Edge>& border );
    
    void getEdgeNodes( std::vector<meshTriPoint>& north, std::vector<meshTriPoint>& south, std::vector<meshTriPoint>& east, std::vector<meshTriPoint>& west ) const;
        
    GDALDataset* openDatasource( const std::string& datasource_name ) const;
    OGRLayer*    openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, const char* layer_name ) const;
    
    meshTriPoint toMeshTriPoint( const meshArrPoint& aPoint ) const {
        return meshTriPoint ( CGAL::to_double( aPoint.x() ), CGAL::to_double(aPoint.y()) );
    }
    meshArrPoint toMeshArrPoint( const meshTriPoint& tPoint ) const {
        return meshArrPoint( tPoint.x(), tPoint.y() );
    }
    
    void toShapefile( const std::string& datasource, const char* layer, const meshArrangement& arr ) const;
    void toShapefile( const std::string& datasource, const char* layer, const std::vector<meshTriPoint>& points ) const;
    void toShapefile( const std::string& datasource, const char* layer, const meshTriCDTPlus& triangulation, bool marked ) const;

    void toShapefile( OGRLayer* poLayer, const meshArrSegment& seg, const char* desc ) const;
    void toShapefile( OGRLayer* poLayer, const meshArrPoint& pt, const char* desc ) const;
    void toShapefile( OGRLayer* poLayer, const meshTriPoint& pt, const char* desc ) const;
    void toShapefile( OGRLayer* poLayer, const meshTriangle& tri ) const;
    
    void writeCdtFile(  const char* filename, std::vector<meshTriPoint>& points,  std::vector<meshTriSegment>& constraints ) const;
    void writeCdtFile2( const char* filename, const meshTriCDTPlus& cdt) const;
    
    unsigned int                    numPriorities;
    std::vector<std::string>        priorityNames;
    std::vector<tgPolygonSetList>   sourcePolys;
    std::vector<cgalPoly_Point>     sourcePoints;

    meshArrangement                 meshArr;
    meshArrLandmarks_pl             meshPointLocation;
    meshTriCDTPlus                  meshTriangulation;
    std::vector<tgMeshFaceMeta>     metaLookup;
    SGBucket                        b;
    bool                            clipBucket;
    SGMutex*                        lock;
    std::string                     datasource;
};

#endif /* __TG_MESH_HXX__ */
