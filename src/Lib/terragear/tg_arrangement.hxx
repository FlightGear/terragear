#ifndef _TGARRANGEMENT_HXX
#define _TGARRANGEMENT_HXX

#include <CGAL/Line_2.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_polyline_traits_2.h>
#include <CGAL/Arrangement_2.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "tg_cgal_epec.hxx"
#include "tg_polygon.hxx"

typedef EPECKernel                                          arrKernel;
//typedef EPICKernel                                          arrKernel;
//typedef INEXACTKernel                                       arrKernel;
typedef CGAL::Arr_segment_traits_2<arrKernel>               arrTraits;
//typedef CGAL::Arr_polyline_traits_2<arrSegment_traits>      arrTraits;
typedef arrTraits::Point_2                                  arrPoint;
typedef arrTraits::Curve_2                                  arrSegment;
//typedef CGAL::Arr_segment_2<arrKernel>                      arrSegment;
typedef CGAL::Arrangement_2<arrTraits>                      arrArrangement;

typedef arrArrangement::Ccb_halfedge_const_circulator       arrCcbHEConstCirc;
typedef arrArrangement::Halfedge_const_handle               arrHEConstHand;


typedef CGAL::Polygon_2<arrKernel>                          Polygon;
typedef CGAL::Polygon_with_holes_2<arrKernel>               Polygon_with_holes;
typedef CGAL::Polygon_set_2<arrKernel>                      Polygon_set;

class tgArrangement
{
public:
    void      Clear( void );
    void      Add( const tgPolygon& subject );
    void      Add( const tgContour& contour, const char* layer );
    void      Add( const tgSegment& subject );
    void      DumpPolys( void );
    void      ToShapefiles( const std::string& path, const std::string& layer );
    void      ToShapefiles( const std::string& path, const std::string& layer_prefix, Polygon_set& polys );

    Polygon_set ToPolygonSet( int contour );
    
private:    
    void GetPolygons( const arrArrangement::Face_const_handle& fh, Polygon_set& polygons, int contour );
    void GetHoles( const arrArrangement::Face_const_handle& fh, std::list<Polygon>& holes, Polygon_set& polygons );
    
    void SaveCCB( const arrArrangement::Ccb_halfedge_const_circulator& ccb, const char* path, const char* layer );
    void SaveFace( const arrArrangement::Face_const_handle& fh, const char* path, const char* layer );
    
    arrArrangement  arr;
};

#endif // _TGARANGEMENT_HXX