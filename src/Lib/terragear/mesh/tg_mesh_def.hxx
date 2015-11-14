#ifndef __TG_MESH_DEF_HXX__
#define __TG_MESH_DEF_HXX__

// mesh class contains two man cgal features
// an arrangement ( with exact constructions ) 
// to define face - polygon meta info, and provides
// cleanup capabilities
// the other is the constrained triangulation
// with required inexact constructions ( for refining )
// it MAY be possible to use exact constructions ( with square root ),
// but it is really slow...

// arrangement
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_landmarks_point_location.h>

// triangulation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

// mesh refinement
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

// the arrangement uses EPECK, the triangulation is EPICK
typedef CGAL::Exact_predicates_exact_constructions_kernel   meshArrKernel;
typedef CGAL::Arr_segment_traits_2<meshArrKernel>           meshArrTraits;

typedef meshArrTraits::Point_2                              meshArrPoint;
typedef meshArrTraits::Curve_2                              meshArrSegment;
typedef CGAL::Arrangement_2<meshArrTraits>                  meshArrangement;
typedef meshArrangement::Face_handle                        meshArrFaceHandle;
typedef meshArrangement::Face_const_handle                  meshArrFaceConstHandle;
typedef meshArrangement::Halfedge_const_handle              meshArrHalfedgeConstHandle;
typedef meshArrangement::Edge_const_iterator                meshArrEdgeConstIterator;
typedef meshArrangement::Vertex_const_handle                meshArrVertexConstHandle;
typedef meshArrangement::Vertex_const_iterator              meshArrVertexConstIterator;
typedef meshArrangement::Edge_const_iterator                meshArrEdgeConstIterator;
typedef CGAL::Arr_landmarks_point_location<meshArrangement> meshArrLandmarks_pl;

// face info for providing a link back to an arrangement face per triangle
struct tgMeshArrFaceInfo
{
    tgMeshArrFaceInfo() : arrMeshFace(NULL), visited(false) {}
    
    void clear( void ) {
        visited     = false;
        arrMeshFace = (meshArrFaceConstHandle)NULL;
    }
    
    void setFace( meshArrFaceConstHandle f ) {
        arrMeshFace = f;
        visited = true;
    }
    
    bool isVisited( void ) const { 
        return visited;
    }
    
    bool hasFace( void ) const {
        return arrMeshFace != (meshArrFaceConstHandle)NULL;
    }
    
private:
    meshArrFaceConstHandle arrMeshFace;
    bool                   visited;
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel                         meshTriKernel;
typedef meshTriKernel::Point_2                                                      meshTriPoint;
typedef meshTriKernel::Segment_2                                                    meshTriSegment;

typedef CGAL::Triangulation_vertex_base_2<meshTriKernel>                            Vb;
typedef CGAL::Constrained_triangulation_face_base_2<meshTriKernel>                  Fbbb;
typedef CGAL::Triangulation_face_base_with_info_2<tgMeshArrFaceInfo, meshTriKernel, Fbbb> Fbb;
typedef CGAL::Delaunay_mesh_face_base_2<meshTriKernel,Fbb>                          Fb;

typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                                 TDS;
typedef CGAL::Exact_intersections_tag                                               Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<meshTriKernel, TDS, Itag>        CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT>                                 meshCDTPlus;

typedef meshCDTPlus::Face_handle                                                    meshTriFaceHandle;
typedef CGAL::Triangle_2<meshTriKernel>                                             meshTriangle;

typedef CGAL::Delaunay_mesh_size_criteria_2<meshCDTPlus>                            meshCriteria;
typedef CGAL::Delaunay_mesher_2<meshCDTPlus, meshCriteria>                          meshRefiner;

#endif