#ifndef _TGSEGMENTNETWORK_HXX
#define _TGSEGMENTNETWORK_HXX

#include <set>

// arrangement
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_consolidated_curve_data_traits_2.h>
#include <CGAL/Arrangement_2/Arr_traits_adaptor_2.h>
//#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_point_location_result.h>
//#include <CGAL/Arr_observer.h>
#include <CGAL/Aff_transformation_2.h>

// kd-tree for verticies ( with data for the vertex handle in arrangement
// #include <CGAL/Simple_cartesian.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/algorithm.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <boost/iterator/zip_iterator.hpp>

// snap_rounding for near colinear segment removal
#include <CGAL/Snap_rounding_traits_2.h>
#include <CGAL/Snap_rounding_2.h>

#include "tg_cgal.hxx"

// the network arrangement
class segnetEdge 
{
public:
    segnetEdge( const SGGeod& s, const SGGeod& e, double w, unsigned int t ) {
        start = s;
        end   = e;
        width = w;
        type  = t;
    }

    SGGeod start;
    SGGeod end;
    double width;
    unsigned int type;
};
typedef std::vector<segnetEdge>   segnetedge_list;
typedef segnetedge_list::iterator segnetedge_it;

class CurveData {
public:
    CurveData() {}
    CurveData( double w, int t ) : width(w), type(t) {}
    
    bool operator==(const CurveData& rhs) const {
        return width == rhs.width && type == rhs.type;
    }
    
    double          width;
    unsigned int    type;
};

typedef CGAL::Exact_predicates_exact_constructions_kernel                           segnetKernel;
typedef CGAL::Arr_segment_traits_2<segnetKernel>                                    segnetSegmentTraits;
typedef segnetSegmentTraits::Curve_2                                                segnetCurve;
typedef CGAL::Arr_consolidated_curve_data_traits_2<segnetSegmentTraits, CurveData>  segnetTraits;
typedef segnetTraits::Point_2                                                       segnetPoint;
typedef segnetTraits::Segment_2                                                     segnetSegment;
typedef segnetTraits::Line_2                                                        segnetLine;
typedef segnetTraits::Curve_2                                                       segnetCurveWithData;
//typedef CGAL::Arrangement_with_history_2<segnetTraits>                              segnetArrangement;
typedef CGAL::Arrangement_2<segnetTraits>                                           segnetArrangement;

typedef CGAL::Aff_transformation_2<segnetKernel>                                    segnetTransformation;

typedef CGAL::Snap_rounding_traits_2<segnetKernel>                                  segnetSRTraits;
typedef std::list<segnetSegment>                                                    segnetSegmentList;
typedef std::list<segnetPoint>                                                      segnetPolyline;
typedef std::list<segnetPolyline>                                                   segnetPolylineList;
#if 1
// need a traitsAdaptor for arbitrary ray shooting
typedef segnetArrangement::Geometry_traits_2                                        segnetGeometryTraits;
typedef CGAL::Arr_traits_basic_adaptor_2<segnetGeometryTraits>                      segnetTraitsAdaptor;

typedef segnetArrangement::X_monotone_curve_2                                       segnetXMonotoneCurve;
typedef segnetArrangement::Vertex_const_handle                                      segnetVertexHandle;
typedef segnetArrangement::Halfedge_const_handle                                    segnetHalfedgeHandle;
typedef segnetArrangement::Halfedge_handle                                          segnetNonConstHalfedgeHandle;
typedef segnetArrangement::Face_const_handle                                        segnetFaceHandle;

typedef segnetArrangement::Halfedge_around_vertex_const_circulator                  Halfedge_around_vertex_const_circulator;
typedef segnetArrangement::Halfedge_around_vertex_circulator                        Halfedge_around_vertex_circulator;
typedef segnetArrangement::Inner_ccb_const_iterator                                 Inner_ccb_const_iterator;
typedef segnetArrangement::Outer_ccb_const_iterator                                 Outer_ccb_const_iterator;
typedef segnetArrangement::Ccb_halfedge_const_circulator                            Ccb_halfedge_const_circulator;

struct Less_halfedge_handle {
    bool operator()(segnetHalfedgeHandle h1, segnetHalfedgeHandle h2) const
    { return (&(*h1) < &(*h2)); }
};

typedef std::set<segnetHalfedgeHandle, Less_halfedge_handle>        Halfedge_set;

typedef CGAL::Arr_point_location_result<segnetArrangement>          Result;
typedef typename Result::Type                                       Result_type;
#endif

// the network nodes for fast lookup
// the Tuple is the location (for lookup), the handle to the vertex in the arrangement, and the cluster center for this node
typedef boost::tuple<segnetPoint,segnetVertexHandle>                segnetPointHandle;
typedef std::list<segnetPointHandle>                                segnetPointHandleList;


typedef boost::tuple<segnetPoint,segnetVertexHandle,segnetPoint>    nodesPointHandle;

typedef CGAL::Search_traits_2<segnetKernel>                         nodesTraitsBase;
typedef CGAL::Search_traits_adapter<nodesPointHandle, CGAL::Nth_of_tuple_property_map<0, nodesPointHandle>,nodesTraitsBase>  nodesTraits;

typedef CGAL::Fuzzy_sphere<nodesTraits>                             nodesFuzzyCir;
typedef CGAL::Kd_tree<nodesTraits>                                  nodesTree;


#define DEBUG_STAGES            (0)
#define DEBUG_CLUSTERS          (0)
#define DEBUG_FINGER_REMOVAL    (0)
#define DEBUG_FINGER_EXTENSION  (0)
#define DEBUG_SHORT_EDGES       (0)


class tgSegmentNetwork
{
public:
    tgSegmentNetwork( const std::string debugRoot );
    
    void      Add( const SGGeod& source, const SGGeod& target, double width, unsigned int type );
    void      Clean( bool clean );
    
    void      DumpPolys( void );
    void      ToShapefiles( const char* prefix );
    
    segnetedge_it output_begin( void ) { return output.begin(); }
    segnetedge_it output_end( void )   { return output.end();   }
    
    bool      empty( void ) const { return (arr.number_of_edges() == 0); }
    
private:
    void      BuildTree( void );
    void      Cluster( void );
    void      RemoveFingers( void );
    void      ExtendFingers( void );
    void      FixShortSegments( void );
    void      RemoveColinearSegments( void );
    void      GenerateOutput( void );
    
    bool      ArbitraryRayShoot( const segnetVertexHandle trg, double course, double dist, segnetPoint& minPoint, unsigned int finger_id, const char* dirname) const;
    
    bool      IsVertexHandleInList( segnetVertexHandle h, std::list<segnetVertexHandle>& vertexList );
    void      ClusterVertex( segnetPoint newTargPoint, std::list<segnetVertexHandle>& vertexList, 
                                                       std::vector<segnetNonConstHalfedgeHandle>& delEdges, 
                                                       std::vector<segnetCurveWithData>& newEdges );
    
    void      ModifyEdges( segnetVertexHandle oldTarget, segnetPoint newTargPoint, segnetVertexHandle ignoreSrc, 
                           std::list<nodesPointHandle>& delNodes, 
                           std::vector<segnetNonConstHalfedgeHandle>& delEdges, 
                           std::vector<segnetCurveWithData>& newEdges );

#if 1    
    // TODO: move arbitraru ray shooting to it's own class - it's very involved....
    Result_type _walk_from_vertex(segnetVertexHandle nearest_vertex, const segnetPoint& p, bool add_ce, Halfedge_set& crossed_edges) const;
    //Result_type _walk_from_edge(segnetHalfedgeHandle eh, const segnetPoint& np, const segnetPoint& p, Halfedge_set& crossed_edges) const;
    Result_type _walk_from_face(segnetFaceHandle fh, const segnetPoint& np, const segnetPoint& p, Halfedge_set& crossed_edges) const;
    Result_type _find_face_around_vertex(segnetVertexHandle vh, const segnetPoint& p, bool& new_vertex) const;
    Result_type _deal_with_curve_contained_in_segment(segnetHalfedgeHandle he, bool p_is_left, const segnetPoint& p, Halfedge_set& crossed_edges) const;
    segnetHalfedgeHandle _intersection_with_ccb(Ccb_halfedge_const_circulator circ, const segnetXMonotoneCurve& seg, const segnetPoint& p, bool p_is_left, 
                                                Halfedge_set& crossed_edges, bool& is_on_edge, bool& is_target, bool& cv_is_contained_in_seg, segnetVertexHandle& new_vertex) const;
    segnetHalfedgeHandle _in_case_p_is_on_edge(segnetHalfedgeHandle he, Halfedge_set& crossed_edges, const segnetPoint& p, bool& is_target) const;
    bool _have_odd_intersections(const segnetXMonotoneCurve& cv, const segnetXMonotoneCurve& seg, bool p_is_left, bool& p_on_curve, bool& cv_and_seg_overlap, bool& cv_is_contained_in_seg) const;
#endif
    
    segnetArrangement  arr;
    nodesTree          tree;
    segnetedge_list    output;
    segnetVertexHandle invalid_vh;
    
    
    char               datasource[128];
};

#if 0
// An arrangement observer, used to receive notifications of face splits and
// face mergers.
class tgSegNetObserver : public CGAL::Arr_observer<segnetArrangement>
{
public:
    tgSegNetObserver (segnetArrangement& arr) : CGAL::Arr_observer<segnetArrangement>(arr) {}

    virtual void before_remove_vertex(segnetVertexHandle v) {    
        std::cout << "-> REMOVE VERTEX " << std::endl;
    }
};
#endif

#endif // _TGSEGMENTNETWORK_HXX__