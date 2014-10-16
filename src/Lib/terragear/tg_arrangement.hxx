#ifndef _TGARRANGEMENT_HXX
#define _TGARRANGEMENT_HXX

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Line_2.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_polyline_traits_2.h>
#include <CGAL/Arrangement_2.h>

#include "tg_polygon.hxx"

typedef CGAL::Exact_predicates_exact_constructions_kernel   arrKernel;
typedef CGAL::Line_2<arrKernel>                             arrLine;
typedef CGAL::Ray_2<arrKernel>                              arrRay;
typedef arrKernel::FT                                       arrNumber_type;
typedef CGAL::Arr_segment_traits_2<arrKernel>               arrSegment_traits;
typedef CGAL::Arr_polyline_traits_2<arrSegment_traits>      arrTraits;
typedef arrTraits::Point_2                                  arrPoint;
typedef arrTraits::Curve_2                                  arrPolyline;
typedef CGAL::Arr_segment_2<arrKernel>                      arrSegment;
typedef CGAL::Arrangement_2<arrTraits>                      arrArrangement;

#if 0
static arrPoint SGGeod_toArrPoint( const SGGeod& g )
{
    return arrPoint( g.getLongitudeDeg(), g.getLatitudeDeg() );
}
#endif

class tgArrangement
{
public:
    void      Add( const tgPolygon& subject );
    void      Add( const tgSegment& subject );
    void      ToShapefiles( const std::string& path, const std::string& layer );
    
private:
    arrArrangement  arr;
};

#endif // _TGARANGEMENT_HXX