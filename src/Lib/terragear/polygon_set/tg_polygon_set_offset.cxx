
#include <CGAL/Origin.h>
#include<CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

#include<boost/shared_ptr.hpp>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon_set.hxx"

typedef boost::shared_ptr<cgalPoly_Polygon>             PolygonPtr ;
typedef std::vector<PolygonPtr>                         PolygonPtrVector;

typedef boost::shared_ptr<cgalPoly_PolygonWithHoles>    PolygonWithHolesPtr ;
typedef std::vector<PolygonWithHolesPtr>                PolygonWithHolesPtrVector;

void tgPolygonSet::contractPolygon( double oset, const cgalPoly_Polygon& poly, std::vector<cgalPoly_Polygon>& offsetPWHs ) const
{
    cgalPoly_PolygonWithHoles           pwh( poly );
    PolygonPtrVector                    polysWithHoles;
    PolygonPtrVector::const_iterator    polyIt;
    cgalPoly_Kernel::FT                 lOffset(oset);

    polysWithHoles = CGAL::create_interior_skeleton_and_offset_polygons_2( lOffset, pwh );
    for ( polyIt = polysWithHoles.begin(); polyIt != polysWithHoles.end(); polyIt++ ) {
        offsetPWHs.push_back( **polyIt );
    }
}

void tgPolygonSet::expandPolygon( double oset, const cgalPoly_Polygon& poly, std::vector<cgalPoly_Polygon>& offsetPWHs ) const
{
    PolygonPtrVector                    polys;
    PolygonPtrVector::const_iterator    polyIt;
    cgalPoly_Kernel::FT                 lOffset(oset);

    polys = CGAL::create_exterior_skeleton_and_offset_polygons_2( lOffset, poly );
    for ( polyIt = polys.begin(); polyIt != polys.end(); polyIt++ ) {
        offsetPWHs.push_back( **polyIt );
    }
}

tgPolygonSet tgPolygonSet::offset( double oset ) const
{
    std::vector<cgalPoly_Polygon> offsetBoundaries;
    std::vector<cgalPoly_Polygon> offsetHoles;
    
    // check for 0 offset
    if ( oset == 0.0l ) {
        return tgPolygonSet( ps, ti, 0 );
    }
    
    // get a list of polygons with holes from the set
    std::list<cgalPoly_PolygonWithHoles>                 pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator it;

    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::offset got " << pwh_list.size() << " polys with holes ");
    
    // we will generate a new polygon set from the union of the offset polyWithHoles
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        cgalPoly_PolygonWithHoles pwh = (*it);

        if ( oset < 0 ) {
            // contract border, expand holes
            contractPolygon( oset, pwh.outer_boundary(), offsetBoundaries );
            
            cgalPoly_PolygonWithHoles::Hole_const_iterator holeIt;
            for ( holeIt = pwh.holes_begin(); holeIt != pwh.holes_end(); holeIt++ ) {
                expandPolygon( oset, *holeIt, offsetHoles );
            }
        } else if ( oset > 0 ) {
            // expand border, contract holes
            expandPolygon( oset, pwh.outer_boundary(), offsetBoundaries );
            
            cgalPoly_PolygonWithHoles::Hole_const_iterator holeIt;
            for ( holeIt = pwh.holes_begin(); holeIt != pwh.holes_end(); holeIt++ ) {
                contractPolygon( oset, *holeIt, offsetHoles );
            }
        }
    }
    
    // how, union all of the boundaries, and holes
    cgalPoly_PolygonSet boundaries;    
    boundaries.join( offsetBoundaries.begin(), offsetBoundaries.end() );
    
    cgalPoly_PolygonSet holes;
    holes.join( offsetHoles.begin(), offsetHoles.end() );

    boundaries.difference( holes );
    
    return tgPolygonSet( boundaries, ti, 0 );
}
