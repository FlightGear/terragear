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

    void      add(const tgPolygonSet& subject);
    void      Diff_and_Add_cgal( tgPolygonSet& subject );
    
    void      toShapefile( const char* datasource, const char* layer );
    
    
private:
    void                    GetAccumPolygonSet( const CGAL::Bbox_2& bb, cgalPoly_PolygonSet& accumPs );
    void                    AddAccumPolygonSet( const cgalPoly_PolygonSet& ps );

    bool                        accumEmpty;
    cgalPoly_PolygonSet         accum_cgal;

    std::list<tgAccumEntry>     accum_cgal_list;
};

#endif // _TGACCUMULATOR_HXX