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
    void init( unsigned int numPriorities, const std::vector<std::string>& priorityNames, const std::string& dbgRoot );
    void addPoly( unsigned int priority, const tgPolygonSet& poly );
    void addPolys( unsigned int priority, const tgPolygonSetList& polys );
    
    tgPolygonSet join( unsigned int priority, const tgPolygonSetMeta& meta );
    
    void generate( void );
    
    void toShapefiles( const char* dataset );

private:
    void clipPolys( void );
    void cleanPolys( void );
    void arrangePolys( void );
    void arrangementInsert( std::vector<tgPolygonSet>::iterator pit );
    
    meshArrFaceConstHandle findPolyFace( meshArrFaceConstHandle f );
    meshArrFaceConstHandle findMeshFace( const meshArrPoint& pt);
    meshArrFaceConstHandle findMeshFace( const meshTriPoint& pt);
    
    void constrainedTriangulate( const char* layer );
    void clearDomains(void);
    void markDomains(void);
    void markDomains(meshTriFaceHandle start, meshArrFaceConstHandle face, std::list<meshCDTPlus::Edge>& border );
    
    GDALDataset* openDatasource( const char* datasource_name );
    OGRLayer*    openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, const char* layer_name );
    
    meshTriPoint toMeshTriPoint( const meshArrPoint& aPoint ) {
        return meshTriPoint ( CGAL::to_double( aPoint.x() ), CGAL::to_double(aPoint.y()) );
    }
    meshArrPoint toMeshArrPoint( const meshTriPoint& tPoint ) {
        return meshArrPoint( tPoint.x(), tPoint.y() );
    }
    
    void toShapefile( const char* datasource, const char* layer, const meshArrangement& arr );
    void toShapefile( OGRLayer* poLayer, const meshArrSegment& seg, char* desc );
    
    void toShapefile( const char* datasource, const char* layer, const meshCDTPlus& triangulation, bool marked );
    void toShapefile( OGRLayer* poLayer, const meshTriangle& tri );
    
    unsigned int                    numPriorities;
    std::vector<std::string>        priorityNames;
    std::vector<tgPolygonSetList>   sourcePolys;
    meshArrangement                 meshArr;
    meshArrLandmarks_pl             meshPointLocation;
    meshCDTPlus                     meshTriangulation;
    std::vector<tgMeshFaceMeta>     metaLookup;
    char datasource[128];
};

#endif /* __TG_MESH_HXX__ */
