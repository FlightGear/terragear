#ifndef __TG_CLUSTER_HXX__
#define __TG_CLUSTER_HXX__

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

//typedef CGAL::Search_traits_2<EPECKernel>                           nodesTraitsBase;
//typedef CGAL::Search_traits_adapter<nodesPointHandle, CGAL::Nth_of_tuple_property_map<0, nodesPointHandle>,nodesTraitsBase>  nodesTraits;
typedef CGAL::Search_traits_2<EPECKernel>                           VDnodesTraits;

typedef CGAL::Fuzzy_sphere<VDnodesTraits>                           VDnodesFuzzyCir;
typedef CGAL::Kd_tree<VDnodesTraits>                                VDnodesTree;

typedef std::vector<EPECPoint_2>                Points;
typedef std::vector<Points>                     PointsArray;

struct tgVoronoiCell 
{
public:
    
    EPECPoint_2              centroid;
    VDFaceHandle             face;
    std::vector<EPECPoint_2> nodes;
    
    bool operator==(const tgVoronoiCell& other ) {
        return (centroid == other.centroid);
    }
};

class tgCluster 
{
public:
    tgCluster( std::list<EPECPoint_2>& points, double err );
    // tgCluster( std::list<EPECPoint_2>&, meshArr.vertices_end(), 0.0000025 );

    EPECPoint_2 Locate( const EPECPoint_2& point ) const;
    
    void toShapefile( const char* datasource, const char* layer );
    
private:
    void computenewcentroids(void);
    
    std::list<EPECPoint_2> nodes;
    double squaredError;
    int numcentroids;
    
    std::vector<tgsegment_list>  v_faces;       // segments making up the face
    
    PointsArray vpoints;

    std::vector<EPECPoint_2>   oldcentroids, newcentroids;
    std::vector<tgVoronoiCell> oldcells, newcells;
    
    VDnodesTree tree;
    VD          vd;    
};

#endif /* __TG_CLUSTER_HXX__ */