#include <simgear/debug/logstream.hxx>

#include "tg_polygon_set.hxx"

#include <simgear/misc/sg_path.hxx> // for file i/o


// every polygon set (metadata) gets its own unique identifier
unsigned long tgPolygonSetMeta::cur_id = 1;

void tgPolygonSet::polygonToSegmentList( const cgalPoly_Polygon& p, std::vector<cgalPoly_Segment>& segs ) const
{
    cgalPoly_Polygon::Vertex_const_iterator   src, trg;

    src = p.vertices_begin();
    trg = src; trg++;
    while( trg != p.vertices_end() ) {
        segs.push_back( cgalPoly_Segment(*src++, *trg++) );
    }
    trg = p.vertices_begin();
    segs.push_back( cgalPoly_Segment(*src, *trg) );    
}

void tgPolygonSet::findIntersections( const cgalPoly_PolygonWithHoles& pwh, const cgalPoly_Line& line, std::vector<cgalPoly_Point>& intersections ) const
{
    // find the intersection of all segments and sorth them from bottom to top.
    cgalPoly_Polygon                                  p  = pwh.outer_boundary();
    cgalPoly_PolygonWithHoles::Hole_const_iterator    hit;
    std::vector<cgalPoly_Segment>                     segs;

    polygonToSegmentList( p, segs );
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        polygonToSegmentList( *hit, segs );
    }
    
    for ( unsigned int i=0; i<segs.size(); i++ ) {
        CGAL::Object result = CGAL::intersection(line, segs[i]);
        if (const cgalPoly_Point *ipoint = CGAL::object_cast<cgalPoly_Point>(&result)) {
            intersections.push_back( *ipoint );
        }
    }
    std::sort( intersections.begin(), intersections.end() );    
}

static bool sortDeltaAndPosDescending(boost::tuple<cgalPoly_Kernel::RT, cgalPoly_Kernel::RT> i, boost::tuple<cgalPoly_Kernel::RT, cgalPoly_Kernel::RT> j ) 
{ 
    // sort from largest to smallest
    return (  boost::get<0>(i) > boost::get<0>(j) ); 
}

cgalPoly_Point tgPolygonSet::getInteriorPoint( const cgalPoly_PolygonWithHoles& pwh ) const
{
    std::vector<cgalPoly_Kernel::RT>                                      xcoords;
    std::vector< boost::tuple<cgalPoly_Kernel::RT, cgalPoly_Kernel::RT> >   xbest;
    cgalPoly_Point  max_pos;
    
    // find the largest delta in x
    cgalPoly_Polygon  p  = pwh.outer_boundary();
    CGAL::Bbox_2    bb = p.bbox();

    cgalPoly_PolygonWithHoles::Hole_const_iterator    hit;
    cgalPoly_Polygon::Vertex_const_iterator           vit;
    for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
        xcoords.push_back( vit->x() );
    }
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        for (vit = hit->vertices_begin(); vit != hit->vertices_end(); ++vit) {
            xcoords.push_back( vit->x() );
        }
    }
    std::sort( xcoords.begin(), xcoords.end() );

    for (unsigned int i=0; i<xcoords.size()-1; i++) {
        cgalPoly_Kernel::RT delta = xcoords[i+1]-xcoords[i];
        xbest.push_back( boost::make_tuple( delta, xcoords[i]+delta/2 ) );
    }
    std::sort( xbest.begin(), xbest.end(), sortDeltaAndPosDescending );
    
    // create a vertical line at the midpoint of the largest delta
    for ( unsigned int i=0; i<xbest.size(); i++ ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "GetInteriorPoint: i " << i << " width " << xbest[i].get<0>() << " location " << xbest[i].get<1>() );       
    }
    
    for ( unsigned int i=0; i<xbest.size(); i++ ) {
        cgalPoly_Line line( cgalPoly_Point( xbest[i].get<1>(), bb.ymin() ), cgalPoly_Point(xbest[i].get<1>(), bb.ymax()) );
        // get and sort the intersections with all segments of the pwh and this line
        
        std::vector<cgalPoly_Point> intersections;
        findIntersections( pwh, line, intersections );
        // from 0-1 IN face, 1-2 OUT of face, 2-3 IN face, etccc.
        // we want the biggest delta between 0,1 2,3 4,5, etc, and the midpoint of the biggest.
        
        cgalPoly_Kernel::RT max_delta = 0.0;
        for ( unsigned int i=0; i<intersections.size(); i+=2 ) {
            if ( intersections[i+1].y() - intersections[i].y() > max_delta ) {
                max_delta = intersections[i+1].y()-intersections[i].y();
                max_pos   = cgalPoly_Point( intersections[i].x(), intersections[i].y()+max_delta/2 );
            }
        }
        
        if ( max_delta > 0.000001 ) {
            break;
        }
    }

    return max_pos;
}

// intersect and modify current tgPolygonSet
void tgPolygonSet::intersection2( const cgalPoly_Polygon& other )
{    
    ps.intersection( other );    
}

// intersect and return a new tgPolygonSet
tgPolygonSet tgPolygonSet::intersection( const cgalPoly_Polygon& other ) const
{
    // copy the geometry;
    cgalPoly_PolygonSet result = getPs();

    result.intersection( other );
    
    // create a new polygonSet
    return tgPolygonSet( result, getMeta() );
}

void tgPolygonSet::difference( const cgalPoly_Polygon& other )
{    
    ps.intersection( other );    
}

void tgPolygonSet::join( const cgalPoly_Polygon& other )
{    
    ps.join( other );    
}

tgPolygonSet tgPolygonSet::join( const tgPolygonSetList& sets, const tgPolygonSetMeta& m )
{
    tgPolygonSetList::const_iterator it;
    cgalPoly_PolygonSet              result;
    
    for ( it = sets.begin(); it != sets.end(); it++ ) {
        result.join( it->getPs() );
    }
    
    return tgPolygonSet( result, m );
}

CGAL::Bbox_2 tgPolygonSet::getBoundingBox( void ) const
{
    std::list<cgalPoly_PolygonWithHoles> pwh_list;    
    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    
    return CGAL::bbox_2( pwh_list.begin(), pwh_list.end() );
}

// where to put these...
double DirectionToHeading( cgalPoly_Direction dir )
{
    double angle = SGMiscd::rad2deg( atan2( CGAL::to_double(dir.dy()), CGAL::to_double(dir.dx()) ) );

    return SGMiscd::normalizePeriodic( 0, 360, -(angle-90) );   
}

cgalPoly_Transformation CreateGeodesyTranslation( const cgalPoly_Point& src, const CGAL::Vector_2<cgalPoly_Kernel>& dir, double offset )
{
    // get the direction of this vector in degrees
    double h = DirectionToHeading( dir.direction() );

    // use simgear Geodesy to get the second point
    SGGeod gsrc = SGGeod::fromDeg( CGAL::to_double( src.x() ), CGAL::to_double( src.y() ) );
    SGGeod gdst = SGGeodesy::direct( gsrc, h, offset );
    
    cgalPoly_Point dst = cgalPoly_Point( gdst.getLongitudeDeg(), gdst.getLatitudeDeg() );
    CGAL::Vector_2<cgalPoly_Kernel> direct = CGAL::Vector_2<cgalPoly_Kernel>(src, dst);
    
    // create a transformation to translate middle point 
    return cgalPoly_Transformation(CGAL::TRANSLATION, direct);
}

SGGeod OffsetPointMiddle( const cgalPoly_Point& pPrev, const cgalPoly_Point& pCur, const cgalPoly_Point& pNext, double offset_by )
{
    // Generate two unit vectors from middle to prev, and middle to next
    CGAL::Vector_2<cgalPoly_Kernel> vecPrev;
    vecPrev = CGAL::Vector_2<cgalPoly_Kernel>( pCur, pPrev );
    vecPrev = vecPrev / sqrt( CGAL::to_double( vecPrev.squared_length() ) );
    
    CGAL::Vector_2<cgalPoly_Kernel> vecNext;
    vecNext = CGAL::Vector_2<cgalPoly_Kernel>( pCur, pNext );
    vecNext = vecNext / sqrt( CGAL::to_double( vecNext.squared_length() ) );
    
    CGAL::Vector_2<cgalPoly_Kernel> vecAvg;
    if ( CGAL::right_turn( pPrev, pCur, pNext ) ) {
        vecAvg = vecPrev + vecNext;
    } else {
        vecAvg = -(vecPrev + vecNext);
    }
    
    // create a translation along vecAvg for offset_by meters
    cgalPoly_Transformation translate = CreateGeodesyTranslation( pCur, vecAvg, offset_by );
    cgalPoly_Point p = translate( pCur );
    
    return SGGeod::fromDeg( CGAL::to_double( p.x() ), CGAL::to_double( p.y() ) );
}

SGGeod OffsetPointFirst( const cgalPoly_Point& pCur, const cgalPoly_Point& pNext, double offset_by )
{
    // Generate vector from cur to next
    CGAL::Vector_2<cgalPoly_Kernel> vecNext;
    vecNext = CGAL::Vector_2<cgalPoly_Kernel>( pCur, pNext );
    
    // create perp to the right
    CGAL::Vector_2<cgalPoly_Kernel> vecRight = vecNext.perpendicular( CGAL::CLOCKWISE );
    
    // create a translation along vecRight for offset_by meters
    cgalPoly_Transformation translate = CreateGeodesyTranslation( pCur, vecRight, offset_by );
    cgalPoly_Point p = translate( pCur );
    
    return SGGeod::fromDeg( CGAL::to_double( p.x() ), CGAL::to_double( p.y() ) );    
}

SGGeod OffsetPointLast( const cgalPoly_Point& pPrev, const cgalPoly_Point& pCur, double offset_by )
{
    // Generate vector from prev to cur
    CGAL::Vector_2<cgalPoly_Kernel> vecCur;
    vecCur = CGAL::Vector_2<cgalPoly_Kernel>( pPrev, pCur );
    
    // create perp to the right
    CGAL::Vector_2<cgalPoly_Kernel> vecRight = vecCur.perpendicular( CGAL::CLOCKWISE );
    
    // create a translation along vecRight for offset_by meters
    cgalPoly_Transformation translate = CreateGeodesyTranslation( pCur, vecRight, offset_by );
    cgalPoly_Point p = translate( pCur );
    
    return SGGeod::fromDeg( CGAL::to_double( p.x() ), CGAL::to_double( p.y() ) );        
}
