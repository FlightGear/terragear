#ifndef __TG_POLYGON_DEF_HXX__
#define __TG_POLYGON_DEF_HXX__

//#include <CGAL/leda_rational.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Filtered_kernel.h>

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

#include <CGAL/Aff_transformation_2.h>

#include <CGAL/Segment_2.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Line_2.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel   cgalPoly_Kernel;
typedef cgalPoly_Kernel::FT                                 cgalPoly_FT;
typedef cgalPoly_Kernel::Direction_2                        cgalPoly_Direction;

typedef CGAL::Arr_segment_traits_2<cgalPoly_Kernel>         cgalPoly_Traits;
typedef cgalPoly_Traits::Point_2                            cgalPoly_Point;
typedef cgalPoly_Traits::Ray_2                              cgalPoly_Ray;
typedef cgalPoly_Traits::Line_2                             cgalPoly_Line;
typedef cgalPoly_Traits::Segment_2                          cgalPoly_Segment;
typedef cgalPoly_Traits::Curve_2                            cgalPoly_Curve;
typedef CGAL::Arrangement_2<cgalPoly_Traits>                cgalPoly_Arrangement;

typedef CGAL::Aff_transformation_2<cgalPoly_Kernel>         cgalPoly_Transformation; 

typedef cgalPoly_Arrangement::Vertex_const_handle           cgalPoly_VertexConstHandle;
typedef cgalPoly_Arrangement::Face_const_handle             cgalPoly_FaceConstHandle;
typedef cgalPoly_Arrangement::Face_const_iterator           cgalPoly_FaceConstIterator;
typedef cgalPoly_Arrangement::Hole_const_iterator           cgalPoly_HoleConstIterator;
typedef cgalPoly_Arrangement::Edge_const_iterator           cgalPoly_EdgeConstIterator;
typedef cgalPoly_Arrangement::Ccb_halfedge_circulator       cgalPoly_CcbHeCirculator;
typedef cgalPoly_Arrangement::Ccb_halfedge_const_circulator cgalPoly_CcbHeConstCirculator;
typedef cgalPoly_Arrangement::Halfedge                      cgalPoly_He;
typedef cgalPoly_Arrangement::Halfedge_handle               cgalPoly_HeHandle;
typedef cgalPoly_Arrangement::Halfedge_const_handle         cgalPoly_HeConstHandle;

typedef CGAL::Polygon_2<cgalPoly_Kernel>                    cgalPoly_Polygon;
typedef CGAL::Polygon_with_holes_2<cgalPoly_Kernel>         cgalPoly_PolygonWithHoles;
typedef CGAL::Polygon_set_2<cgalPoly_Kernel>                cgalPoly_PolygonSet;

#endif /* __TG_POLYGON_DEF_HXX__ */
