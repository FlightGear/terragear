#ifndef __TG_MESH_ARRANGEMENT_HXX__
#define __TG_MESH_ARRANGEMENT_HXX__

#include "tg_mesh.hxx"

// forward declarations
class tgMesh;

struct tgMeshFaceMeta
{
public:
    tgMeshFaceMeta( meshArrFaceConstHandle h, const cgalPoly_Point& qp, const tgPolygonSetMeta& m ) : face(h), point(qp), meta(m) {}
    
    meshArrFaceConstHandle  face;
    cgalPoly_Point          point;
    tgPolygonSetMeta        meta;
};

class tgMeshArrangement
{
public:
    tgMeshArrangement( tgMesh* m ) { mesh = m; }

    typedef enum {
        SRC_POINT_OK        = 0,
        SRC_POINT_PROJECTED = 1,
        SRC_POINT_DELETED   = 2
    } SrcPointOp_e;
    
    void clear( void ) {
        meshPointLocation.detach();
        meshArr.clear();
        metaLookup.clear();   
        
        // clear source polys
        for ( unsigned int i=0; i<numPriorities; i++ ) {
            sourcePolys[i].clear();
        }
        sourcePoints.clear();        
    }
    
    bool empty( void );
    
    void initPriorities( const std::vector<std::string>& names );
    
    void addPoly( unsigned int priority, const tgPolygonSet& poly );
    void addPolys( unsigned int priority, const tgPolygonSetList& polys );
    void addPoints( const std::vector<cgalPoly_Point>& points );    
    tgPolygonSet join( unsigned int priority, const tgPolygonSetMeta& meta );
    
    void clipPolys( const SGBucket& b, bool clipBucket );
    void cleanArrangement( SGMutex* lock );
    void arrangePolys( void );
    void arrangementInsert( std::vector<tgPolygonSet>::iterator pit );
    
    void doClusterEdges( const tgCluster& cluster );    
    SrcPointOp_e checkPointNearEdge( const cgalPoly_Point& pt, meshArrFaceConstHandle fh, cgalPoly_Point& projPt );
    void doProjectPointsToEdges( const tgCluster& cluster );
    void doRemoveAntenna( void );
    void doSnapRound( SGMutex* lock );
    
    meshArrFaceConstHandle findPolyFace( meshArrFaceConstHandle f ) const;
    meshArrFaceConstHandle findMeshFace( const meshArrPoint& pt) const;
    meshArrFaceConstHandle findMeshFace( const meshTriPoint& pt) const;

    meshArrPoint toMeshArrPoint( const meshTriPoint& tPoint ) const {
        return meshArrPoint( tPoint.x(), tPoint.y() );
    }
    meshTriPoint toMeshTriPoint( const meshArrPoint& aPoint ) const {
        return meshTriPoint ( CGAL::to_double( aPoint.x() ), CGAL::to_double(aPoint.y()) );
    }

    void getPoints( std::vector<meshTriPoint>& points ) const;
    void getSegments( std::vector<meshTriSegment>& constraints ) const;
    
    
    void loadArrangement( const std::string& path );
    
    // ********** Arrangement I/O **********
    //
    // Main APIs
    void toShapefile( const std::string& datasource, const char* layer ) const;
    void fromShapefile( const std::string& filename, std::vector<meshArrSegment>& segments ) const;
    
    // helper - save a segment
    void toShapefile( OGRLayer* poLayer, const meshArrSegment& seg, const char* desc ) const;
    
    // helper - save a point
    void toShapefile( OGRLayer* poLayer, const meshArrPoint& pt, const char* desc ) const;
    
    // helper - save a face
    void toShapefile( OGRLayer* poLayer, const meshArrFaceConstHandle f, const cgalPoly_Point& qp, const char* desc ) const;
    
    // helper read a face
    void fromShapefile( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, std::vector<meshArrSegment>& segments ) const;
    
private:
    tgMesh*                         mesh;
    
    unsigned int                    numPriorities;
    std::vector<std::string>        priorityNames;
    std::vector<tgPolygonSetList>   sourcePolys;
    std::vector<cgalPoly_Point>     sourcePoints;
    
    meshArrangement                 meshArr;
    meshArrLandmarks_pl             meshPointLocation;
    std::vector<tgMeshFaceMeta>     metaLookup;    
};

#endif /* __TG_MESH_ARRANGEMENT_HXX__ */
