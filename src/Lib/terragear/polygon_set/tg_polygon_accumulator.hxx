#ifndef _TGACCUMULATOR_HXX
#define _TGACCUMULATOR_HXX

#include "tg_polygon_set.hxx"

struct tgAccumEntry
{
public:
    cgalPoly_PolygonWithHoles   pwh;
    CGAL::Bbox_2                bbox;
};

class tgAccumulator
{
public:
    tgAccumulator() { accumEmpty = true; }

    void      Diff_and_Add_cgal( tgPolygonSet& subject );
    
private:
    cgalPoly_PolygonSet         GetAccumPolygonSet( const CGAL::Bbox_2& bb );
    void                        AddAccumPolygonSet( const cgalPoly_PolygonSet& ps );

    bool                        accumEmpty;
    cgalPoly_PolygonSet         accum_cgal;

    std::list<tgAccumEntry>     accum_cgal_list;
};

#endif // _TGACCUMULATOR_HXX