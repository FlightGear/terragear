#ifndef __TG_CLUSTER_HXX__
#define __TG_CLUSTER_HXX__

#include <ogrsf_frmts.h>


#include "tg_cgal_epec.hxx"
#include "tg_cgal.hxx"

// includes for defining the Voronoi diagram adaptor
#include <CGAL/basic.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/centroid.h>
#include <CGAL/Dimension.h>

#include <CGAL/Kd_tree.h>
#include <CGAL/algorithm.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Search_traits_2.h>

// typedefs for defining the adaptor
typedef CGAL::Delaunay_triangulation_2<EPECKernel>                           DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT,AT,AP>                                    VD;
typedef CGAL::Dimension_tag<0>                                               tag;

//typedef K::Point_2                            Point;
typedef EPECKernel::Iso_rectangle_2             EPECIsoRectangle;

// typedef for the result type of the point location
typedef AT::Site_2                              EPECSite;

typedef VD::Locate_result                       VDLocateResult;
typedef AT::Nearest_site_2                      ATLocateResult;
typedef VD::Vertex_handle                       VDVertexHandle;
typedef VD::Face_handle                         VDFaceHandle;
typedef VD::Halfedge_handle                     VDHalfedgeHandle;
typedef VD::Ccb_halfedge_circulator             VDCcbHalfedgeCirculator;
typedef VD::Halfedge_around_vertex_circulator   VDHalfedge_around_vertex_circulator;

struct tgClusterNode
{
public:
    tgClusterNode( const EPECPoint_2& p, bool f ) : point(p), fixed(f), added(false) {};
    tgClusterNode() {};

    bool operator==(const tgClusterNode& other ) {
        return (point == other.point);
    }

    EPECPoint_2 point;
    bool        fixed;
    bool        added;
};

typedef boost::tuple<EPECPoint_2, std::vector<tgClusterNode>::iterator> nodesData;
typedef CGAL::Search_traits_2<EPECKernel>       nodesTraitsBase;
typedef CGAL::Search_traits_adapter<nodesData, CGAL::Nth_of_tuple_property_map<0, nodesData>,nodesTraitsBase>  VDnodesTraits;
//typedef CGAL::Search_traits_2<EPECKernel>                         VDnodesTraits;

typedef CGAL::Fuzzy_sphere<VDnodesTraits>       VDnodesFuzzyCir;
typedef CGAL::Kd_tree<VDnodesTraits>            VDnodesTree;

typedef std::vector<EPECPoint_2>                Points;
typedef std::vector<Points>                     PointsArray;

struct tgVoronoiCell 
{
public:

    EPECPoint_2                centroid;
    bool                       fixed;
    VDFaceHandle               face;
    std::vector<tgClusterNode> nodes;

    bool operator==(const tgVoronoiCell& other ) {
        return (centroid == other.centroid);
    }
};

class tgCluster 
{
public:
    tgCluster( std::list<tgClusterNode>& points, double err, const std::string& d );
    EPECPoint_2 Locate( const EPECPoint_2& point ) const;

    void toShapefile( const char* datasource, const char* layer );

private:
    void computenewcentroids(void);

    // debug
    GDALDataset* openDatasource( const std::string& debug ) const;
    OGRLayer*    openLayer( GDALDataset* poDs, OGRwkbGeometryType type, const char* layer ) const;

    void toShapefile( const std::string& ds, const char* layer, const std::vector<tgClusterNode>& nodes );
    void toShapefile( OGRLayer* poLayer, const tgClusterNode& node );
    void toShapefile( OGRLayer* poLayer, const std::vector<EPECPoint_2>& points );
    void toShapefile( OGRLayer* poLayer, const EPECPoint_2& point );

    std::list<tgClusterNode>  nodes;
    double                    squaredError;
    int                       numcentroids;

    std::vector<tgsegment_list> v_faces;       // segments making up the face
    PointsArray                 vpoints;

    std::vector<tgClusterNode>  oldcentroids, newcentroids;
    std::vector<tgVoronoiCell>  oldcells,     newcells;

    VDnodesTree tree;
    VD          vd;
    std::string debug;
};

#endif /* __TG_CLUSTER_HXX__ */