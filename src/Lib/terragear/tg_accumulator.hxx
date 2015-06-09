#ifndef _TGACCUMULATOR_HXX
#define _TGACCUMULATOR_HXX

#include "tg_polygon.hxx"
#include "tg_contour.hxx"

#include "tg_arrangement.hxx"
#include "clipper.hpp"

struct tgAccumEntry
{
public:
    Polygon_with_holes    pwh;
    CGAL::Bbox_2          bbox;
};

class tgAccumulator
{
public:
    tgAccumulator() { accumEmpty = true; }
    
    tgPolygon Diff( const tgContour& subject );
    tgPolygon Diff( const tgPolygon& subject );
    
    void      Diff_cgal( tgPolygon& subject );

    void      Add( const tgContour& subject );
    void      Add( const tgPolygon& subject );
    void      Add_cgal( const tgPolygon& subject );

    void      Diff_and_Add_cgal( tgPolygon& subject );
    
    void      ToShapefiles( const std::string& path, const std::string& layer, bool individual );
    void      ToClipperfiles( const std::string& path, const std::string& layer_prefix, bool individual );

    tgPolygon Union( void );
    
private:
    Polygon_set                 GetAccumPolygonSet( const CGAL::Bbox_2& bb );
    void                        AddAccumPolygonSet( const Polygon_set& ps );

    typedef std::vector < ClipperLib::Paths > clipper_polygons_list;

    clipper_polygons_list       accum;
    bool                        accumEmpty;
    Polygon_set                 accum_cgal;

    std::list<tgAccumEntry>     accum_cgal_list;

    UniqueSGGeodSet             nodes;
};

#endif // _TGACCUMULATOR_HXX