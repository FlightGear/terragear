#ifndef __TG_MESH_TRIANGULATION_HXX__
#define __TG_MESH_TRIANGULATION_HXX__

#include "tg_mesh.hxx"

// forward declarations
class tgMesh;
class tgMeshArrangement;


// Shared edge definitions and search trees
typedef enum {
    NORTH_EDGE,
    SOUTH_EDGE,
    EAST_EDGE,
    WEST_EDGE
} edgeType;

// search tree for nodes one the that are moved to match neighbor tiles
typedef CGAL::Search_traits_2<meshTriKernel>                                                                                            moveNodeBaseTraits;
typedef boost::tuple<meshTriPoint,meshTriPoint>                                                                                         moveNodeData;
typedef CGAL::Search_traits_adapter<moveNodeData,CGAL::Nth_of_tuple_property_map<0, moveNodeData>,moveNodeBaseTraits>                   moveNodeTraits;
typedef CGAL::Fuzzy_sphere<moveNodeTraits>                                                                                              moveNodeFuzzyCir;
typedef CGAL::Kd_tree<moveNodeTraits>                                                                                                   moveNodeTree;

// search tree for shared nodes between two tiles ( along with the membership of which tile node came from )
typedef enum {
    NODE_CURRENT,
    NODE_NEIGHBOR,
    NODE_BOTH
} nodeMembership;

typedef CGAL::Search_traits_2<meshTriKernel>                                                                                            nodeMembershipTraitsBase;
typedef boost::tuple<meshTriPoint,nodeMembership,int>                                                                                   nodeMembershipData;
typedef CGAL::Search_traits_adapter<nodeMembershipData,CGAL::Nth_of_tuple_property_map<0, nodeMembershipData>,nodeMembershipTraitsBase> nodeMembershipTraits;
typedef CGAL::Orthogonal_k_neighbor_search<nodeMembershipTraits>                                                                        nodeMembershipSearch;
typedef nodeMembershipSearch::Tree                                                                                                      nodeMembershipTree;

// search tree for nodes finding closest node to a position
typedef CGAL::Search_traits_2<meshTriKernel>                                                                                            edgeNodeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<edgeNodeTraits>                                                                              edgeNodeSearch;
typedef edgeNodeSearch::Tree                                                                                                            edgeNodeTree;

// search tree for finding nodes on a shared edge ( just position )
//typedef CGAL::Search_traits_2<meshTriKernel>                                                                                            findNodeTraits;
//typedef CGAL::Fuzzy_iso_box<findNodeTraits>                                                                                             findNodeFuzzyBox;
//typedef CGAL::Kd_tree<findNodeTraits>                                                                                                   findNodeTree;

typedef CGAL::Search_traits_2<meshTriKernel>                                                                                            findVertexInfoTraitsBase;
typedef boost::tuple<meshTriPoint,meshVertexInfo>                                                                                       findVertexInfoData;
typedef CGAL::Search_traits_adapter<findVertexInfoData,CGAL::Nth_of_tuple_property_map<0, findVertexInfoData>,findVertexInfoTraitsBase> findVertexInfoTraits;
typedef CGAL::Fuzzy_iso_box<findVertexInfoTraits>                                                                                       findVertexInfoFuzzyBox;
typedef CGAL::Kd_tree<findVertexInfoTraits>                                                                                             findVertexInfoTree;


// search tree for finding nodes on a shared edge ( position and vertex handle in triangulation ) 
typedef CGAL::Search_traits_2<meshTriKernel>                                                                                            findVertexTraitsBase;
typedef boost::tuple<meshTriPoint,meshTriVertexHandle>                                                                                  findVertexData;
typedef CGAL::Search_traits_adapter<findVertexData,CGAL::Nth_of_tuple_property_map<0, findVertexData>,findVertexTraitsBase>             findVertexTraits;
typedef CGAL::Fuzzy_iso_box<findVertexTraits>                                                                                           findVertexFuzzyBox;
typedef CGAL::Kd_tree<findVertexTraits>                                                                                                 findVertexTree;

class tgMeshTriangulation
{
public:
    tgMeshTriangulation( tgMesh* m ) { mesh = m; }
    
    void constrainedTriangulateWithEdgeModification( const tgMeshArrangement& arr );
    void constrainedTriangulateWithoutEdgeModification( const std::vector<meshTriPoint>& points, const std::vector<meshTriSegment>& constraints );
    void clearDomains(void);
    void markDomains( const tgMeshArrangement& arr );
    void markDomains(meshTriFaceHandle start, meshArrFaceConstHandle face, std::list<meshTriCDTPlus::Edge>& border );

    void clear( void ) {
        meshTriangulation.clear();
        vertexInfo.clear();
        faceInfo.clear();
    }

    void matchNodes( edgeType edge, std::vector<meshVertexInfo>& current, std::vector<meshVertexInfo>& neighbor, std::vector<meshTriPoint>& addedNodes, moveNodeTree& movedNodes );    
    void loadTriangulation( const std::string& path, const SGBucket& bucket );
    
    void saveSharedEdgeNodes( const std::string& path ) const;
    void saveSharedEdgeFaces( const std::string& path ) const;
    
    void saveIncidentFaces( const std::string& path, const char* layer, const std::vector<meshTriVertexHandle>& vertexes ) const;
    
    void calcTileElevations( const tgArray* array );
    
    // ********** Triangulation I/O **********
    // 
    // Main APIs
    void toShapefile( const std::string& datasource, const char* layer, bool marked ) const;
    // void toShapefile( const std::string& datasource, const char* layer, const std::vector<meshTriPoint>& points ) const;
    // void fromShapefile( const std::string& filename, std::vector<meshTriSegment>& segments ) const;
    
    // helper - save single point
    void toShapefile( OGRLayer* poLayer, const meshTriPoint& pt, const char* desc ) const;
    
    void toShapefile( const std::string& datasource, const char* layer, std::vector<meshVertexInfo>& points ) const;
    
    // helper - save single point with calcTileElevation
    // void toShapefile( OGRLayer* poLayer, const meshTriPoint& pt, double elv, unsigned int idx ) const;
    
    // helper - save single constraint
    // void toShapefile( OGRLayer* poLayer, const meshTriCDTPlus::Subconstraint_iterator cit ) const;
    
    // helper - save single triangle ( from triangle - no vertex info - just points )
    // void toShapefile( OGRLayer* poLayer, const meshTriangle& tri ) const;
    
    // helper - save single vertex with info
    //void toShapefile( OGRLayer* poLayer, const meshTriVertexHandle vit, 
    //                  CGAL::Unique_hash_map<meshTriVertexHandle, unsigned int>& V) const;
    
    // helper - save single triangle ( from face iterator - can get vertex info )
    //void toShapefile( OGRLayer* poLayer, const meshTriFaceHandle tri ) const;
    //void toShapefile( OGRLayer* poLayer, const meshTriFaceHandle tri, 
    //                  CGAL::Unique_hash_map<meshTriVertexHandle, unsigned int>& V,
    //                  CGAL::Unique_hash_map<meshTriFaceHandle, unsigned int>& F ) const;
    
    
    void toShapefile( OGRLayer* poLayer, const meshVertexInfo& vi) const;
    void toShapefile( OGRLayer* poLayer, const meshFaceInfo& fi) const;
    
    // helper - read vertex info from feature
    void fromShapefile( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, std::vector<meshVertexInfo>& points ) const;
    
    // helper - read face info from feature
    void fromShapefile( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, std::vector<meshFaceInfo>& faces ) const;
    
        
    // debug i/o
    void toShapefile( const std::string& datasource, const char* layer, const moveNodeTree& tree );
    void toShapefile( const std::string& datasource, const char* layer, const nodeMembershipTree& tree );
    
    // loading stage 1 shared edge data
    void fromShapefile( const std::string& filename, std::vector<meshVertexInfo>& points ) const;
        
    // loading stage 1 triangulation 
    void fromShapefile( const std::string& filename, std::vector<meshFaceInfo>& faces ) const;
                        
    // CGAL debugging - save triangulation that can be read byt cgal_tri_test app - fro reporting issues upstream
    void writeCdtFile(  const char* filename, std::vector<meshTriPoint>& points,  std::vector<meshTriSegment>& constraints ) const;
    void writeCdtFile2( const char* filename, const meshTriCDTPlus& cdt) const;
    
    void saveAscii( const std::string& datasource, const char* filename ) const;
    void loadTds(std::vector<meshVertexInfo>& points, std::vector<meshFaceInfo>& faces);
    
    void prepareTds( void );
    void saveTds( const std::string& datasource, const char* layer ) const;
    
private:
    void loadStage1SharedEdge( const std::string& p, const SGBucket& b, edgeType edge, std::vector<meshVertexInfo>& points );
    void sortByLat( std::vector<meshVertexInfo>& points ) const;
    void sortByLon( std::vector<meshVertexInfo>& points ) const;
    
    void getEdgeNodes( std::vector<meshVertexInfo>& north, std::vector<meshVertexInfo>& south, std::vector<meshVertexInfo>& east, std::vector<meshVertexInfo>& west ) const;
    void getEdgeVertices( std::vector<meshTriVertexHandle>& north, std::vector<meshTriVertexHandle>& south, std::vector<meshTriVertexHandle>& east, std::vector<meshTriVertexHandle>& west ) const;
    
    const meshVertexInfo* findVertex( int idx ) const;
    
    tgMesh*                                         mesh;
    meshTriCDTPlus                                  meshTriangulation;    
    
    std::vector<meshVertexInfo>                     vertexInfo;
    std::vector<meshFaceInfo>                       faceInfo;
};

#endif /* __TG_MESH_TRIANGULATION_HXX__ */