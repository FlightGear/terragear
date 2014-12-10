#ifndef _TGARRANGEMENT_HXX
#define _TGARRANGEMENT_HXX

#include <CGAL/Line_2.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_polyline_traits_2.h>
#include <CGAL/Arrangement_2.h>

#include "tg_cgal_epec.hxx"
#include "tg_polygon.hxx"

typedef EPECKernel                                          arrKernel;
typedef CGAL::Arr_segment_traits_2<arrKernel>               arrSegment_traits;
typedef CGAL::Arr_polyline_traits_2<arrSegment_traits>      arrTraits;
typedef arrTraits::Point_2                                  arrPoint;
typedef arrTraits::Curve_2                                  arrPolyline;
typedef CGAL::Arr_segment_2<arrKernel>                      arrSegment;
typedef CGAL::Arrangement_2<arrTraits>                      arrArrangement;

class tgArrangement
{
public:
    void      Add( const tgPolygon& subject );
    void      Add( const tgSegment& subject );
    void      DumpPolys( void );
    void      ToShapefiles( const std::string& path, const std::string& layer );
    
private:
    arrArrangement  arr;
};

#endif // _TGARANGEMENT_HXX