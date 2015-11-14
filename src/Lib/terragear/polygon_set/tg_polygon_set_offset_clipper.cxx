#include <simgear/debug/logstream.hxx>
#include "tg_polygon_set.hxx"

#define CLIPPER_FIXEDPT           (1000000000000)
#define CLIPPER_METERS_PER_DEGREE (111000)

tgPolygonSet tgPolygonSet::offset( double oset ) const
{
    ClipperLib::Paths clipper_src, clipper_dst;
    clipper_src = toClipper( getPs() );

    ClipperLib::ClipperOffset co(2.0, 0.25);
    //co.AddPaths(clipper_src, ClipperLib::jtSquare, ClipperLib::etClosedPolygon); 
    co.AddPaths(clipper_src, ClipperLib::jtMiter, ClipperLib::etClosedPolygon); 
    co.Execute(clipper_dst, toClipper( oset ) );

    cgalPoly_PolygonSet result = fromClipper( clipper_dst );
    
    return tgPolygonSet( result, getMeta() );    
}

double tgPolygonSet::toClipper( double dist ) const
{
    return ( (dist / CLIPPER_METERS_PER_DEGREE) * CLIPPER_FIXEDPT );
}

ClipperLib::IntPoint tgPolygonSet::toClipper( const cgalPoly_Point& p ) const
{
    ClipperLib::cUInt x, y;
    
    if ( p.x() > 0 ) {
        x = (ClipperLib::cUInt)( ( CGAL::to_double(p.x()) * CLIPPER_FIXEDPT ) + 0.5  );
    } else {
        x = (ClipperLib::cUInt)( ( CGAL::to_double(p.x()) * CLIPPER_FIXEDPT ) - 0.5  );
    }
    
    if ( p.y() > 0 ) {
        y = (ClipperLib::cUInt)( ( CGAL::to_double(p.y()) * CLIPPER_FIXEDPT ) + 0.5  );
    } else {
        y = (ClipperLib::cUInt)( ( CGAL::to_double(p.y()) * CLIPPER_FIXEDPT ) - 0.5  );
    }
    
    return ClipperLib::IntPoint( x, y );
}

cgalPoly_Point tgPolygonSet::fromClipper( const ClipperLib::IntPoint& p ) const
{
    double lon, lat;
    
    lon = (double)( ((double)p.X) / (double)CLIPPER_FIXEDPT );
    lat = (double)( ((double)p.Y) / (double)CLIPPER_FIXEDPT );
    
    return cgalPoly_Point( lon, lat );
}

ClipperLib::Path tgPolygonSet::toClipper( const cgalPoly_Polygon& subject, bool isHole ) const
{
    ClipperLib::Path  contour;
    
    for ( unsigned int i=0; i<subject.size(); i++)
    {
        contour.push_back( toClipper(subject[i]) );
    }

    if ( isHole ) {
        if ( Orientation( contour ) ) {
            ReversePath( contour );
        }
    } else {
        if ( !Orientation( contour ) ) {
            ReversePath( contour );
        }        
    }
    
    return contour;
}

void tgPolygonSet::toClipper( const cgalPoly_PolygonWithHoles& pwh, ClipperLib::Paths& paths ) const
{
    // first, add outer boundary
    paths.push_back( toClipper( pwh.outer_boundary(), false ) );
    
    cgalPoly_PolygonWithHoles::Hole_const_iterator hit;
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        paths.push_back( toClipper( *hit, true ) );
    }
}

ClipperLib::Paths tgPolygonSet::toClipper( const cgalPoly_PolygonSet& polySet ) const
{
    ClipperLib::Paths result;
    
    std::list<cgalPoly_PolygonWithHoles>                 pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator it;
    
    polySet.polygons_with_holes( std::back_inserter(pwh_list) );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::toClipper: got " << pwh_list.size() << " polys with holes ");
    
    // save each poly with holes to the layer
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        cgalPoly_PolygonWithHoles pwh = (*it);
        
        toClipper( pwh, result );
    }
    
    return result;
}

cgalPoly_Polygon tgPolygonSet::fromClipper( const ClipperLib::Path& subject ) const
{
    cgalPoly_Polygon poly;
    
    for (unsigned int i = 0; i < subject.size(); i++)
    {
        ClipperLib::IntPoint ip = ClipperLib::IntPoint( subject[i].X, subject[i].Y );
        poly.push_back( fromClipper( ip ) );
    }
    
    return poly;
}

cgalPoly_PolygonSet tgPolygonSet::fromClipper( const ClipperLib::Paths& subject ) const
{
    std::vector<cgalPoly_Polygon> boundaries;
    std::vector<cgalPoly_Polygon> holes;
    
    for (unsigned int i = 0; i < subject.size(); i++)
    {
        cgalPoly_Polygon poly = fromClipper( subject[i] );
        
        if ( poly.is_clockwise_oriented() ) {
            holes.push_back( poly );
        } else {
            boundaries.push_back( poly );
        }
    }
    
    cgalPoly_PolygonSet psBoundaries;
    cgalPoly_PolygonSet psHoles;
    
    psBoundaries.join( boundaries.begin(), boundaries.end() );
    psHoles.join( holes.begin(), holes.end() );
    
    psBoundaries.difference( psHoles );
    
    return psBoundaries;
}
