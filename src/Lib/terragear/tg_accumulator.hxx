#ifndef _TGACCUMULATOR_HXX
#define _TGACCUMULATOR_HXX

#include "tg_polygon.hxx"
#include "tg_contour.hxx"
#include "clipper.hpp"

class tgAccumulator
{
public:
    tgPolygon Diff( const tgContour& subject );
    tgPolygon Diff( const tgPolygon& subject );

    void      Add( const tgContour& subject );
    void      Add( const tgPolygon& subject );

    void      ToShapefiles( const std::string& path, const std::string& layer, bool individual );

private:
    typedef std::vector < ClipperLib::Polygons > clipper_polygons_list;

    clipper_polygons_list   accum;
    UniqueSGGeodSet         nodes;
};

#endif // _TGACCUMULATOR_HXX