#include <simgear/debug/logstream.hxx>

#include <simgear/math/SGMath.hxx>
#include <simgear/math/SGGeod.hxx>

#include "tg_polygon_set.hxx"


// SPLIT LONG EDGES
cgalPoly_Polygon tgPolygonSet::splitLongEdges( cgalPoly_Polygon& p, int maxSegmentSize )
{
    cgalPoly_Polygon::Vertex_iterator src_it, trg_it;
    cgalPoly_Point src, trg;

    std::vector<cgalPoly_Point>       nodes;
    
    // traverse the source poly, and add new nodes conforming to maxSegmentLength
    for ( src_it = p.vertices_begin(), trg_it = src_it+1; trg_it != p.vertices_end(); src_it++, trg_it++ ) {
        src = *src_it;
        trg = *trg_it;
        
        SG_LOG(SG_GENERAL, SG_DEBUG, "src point = " << src << " trg point = " << trg );

        if ( CGAL::abs(src.y()) < 90.0 || CGAL::abs(trg.y()) < 90.0 )
        {
            // convert to Geodesy for distance calcs
            SGGeod gSrc = SGGeod::fromDeg( CGAL::to_double(src.x()), CGAL::to_double(src.y()) );
            SGGeod gTrg = SGGeod::fromDeg( CGAL::to_double(trg.x()), CGAL::to_double(trg.y()) );
            cgalPoly_Kernel::RT dist    = SGGeodesy::distanceM( gSrc, gTrg );
            cgalPoly_Kernel::RT maxDist = maxSegmentSize;
            
            SG_LOG(SG_GENERAL, SG_DEBUG, "distance = " << dist);

            if ( dist > maxDist ) {
                unsigned int segments = (int)CGAL::to_double( (dist / maxDist) + 1 );
                
                SG_LOG(SG_GENERAL, SG_DEBUG, "brek into " << segments << " segments" );

                cgalPoly_Kernel::RT segs = segments;
                cgalPoly_Kernel::RT dx = (trg.x() - src.x()) / segs;
                cgalPoly_Kernel::RT dy = (trg.y() - src.y()) / segs;

                for ( unsigned int j = 0; j < segments; j++ ) {
                    cgalPoly_Kernel::RT idx = j;
                    cgalPoly_Point      tmp = cgalPoly_Point( src.x() + dx*idx, src.y() + dy*idx );
                    SG_LOG(SG_GENERAL, SG_DEBUG, tmp);
                    nodes.push_back(tmp);
                }
            } else {
                SG_LOG(SG_GENERAL, SG_DEBUG, src);
                nodes.push_back(src);
            }
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, src);
            nodes.push_back(src);
        }

        // end of segment is beginning of next segment
    }

    src = trg;
    trg = *(p.vertices_begin());
    
    // convert to Geodesy for distance calcs
    SGGeod gSrc = SGGeod::fromDeg( CGAL::to_double(src.x()), CGAL::to_double(src.y()) );
    SGGeod gTrg = SGGeod::fromDeg( CGAL::to_double(trg.x()), CGAL::to_double(trg.y()) );
    cgalPoly_Kernel::RT dist    = SGGeodesy::distanceM( gSrc, gTrg );
    cgalPoly_Kernel::RT maxDist = maxSegmentSize;
            
    SG_LOG(SG_GENERAL, SG_DEBUG, "distance = " << dist);

    if ( dist > maxDist ) {
        unsigned int segments = (int)CGAL::to_double( (dist / maxDist) + 1 );
        SG_LOG(SG_GENERAL, SG_DEBUG, "brek into " << segments << " segments" );

        cgalPoly_Kernel::RT segs = segments;
        cgalPoly_Kernel::RT dx = (trg.x() - src.x()) / segs;
        cgalPoly_Kernel::RT dy = (trg.y() - src.y()) / segs;

        for ( unsigned int j = 0; j < segments; j++ ) {
            cgalPoly_Kernel::RT idx = j;
            cgalPoly_Point      tmp = cgalPoly_Point( src.x() + dx*idx, src.y() + dy*idx );
            SG_LOG(SG_GENERAL, SG_DEBUG, tmp);
            nodes.push_back(tmp);
        }
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, src);
        nodes.push_back(src);
    }
    
    return ( cgalPoly_Polygon(nodes.begin(), nodes.end()) );
}

cgalPoly_PolygonWithHoles tgPolygonSet::splitLongEdges( cgalPoly_PolygonWithHoles& pwh, int maxSegmentSize )
{
    // need new outer boundary, and a vector of holes
    cgalPoly_Polygon                outer;
    std::vector<cgalPoly_Polygon>   holes;
    
    outer = splitLongEdges( pwh.outer_boundary(), maxSegmentSize );
    
    cgalPoly_PolygonWithHoles::Hole_iterator    hit;
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        holes.push_back( splitLongEdges( *hit, maxSegmentSize ) );
    }
    
    if ( !holes.empty() ) {
        return cgalPoly_PolygonWithHoles( outer );
    } else {
        return cgalPoly_PolygonWithHoles( outer, holes.begin(), holes.end() );
    }
}

void tgPolygonSet::splitLongEdges( int maxSegmentLength )
{
    cgalPoly_PolygonSet                                  result;
    
    std::list<cgalPoly_PolygonWithHoles>                 pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator it;

    // first, extract all of the polys with holes
    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    SG_LOG(SG_GENERAL, SG_ALERT, "tgPolygonSet::splitLongEdges: got " << pwh_list.size() << " polys with holes ");
    
    // for each poly with holes, create a new one with each segment conforming to 
    // maxSegmentLength
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        cgalPoly_PolygonWithHoles src_pwh = (*it);

        result.insert( splitLongEdges(src_pwh, maxSegmentLength) );
    }
    
    // replace with the new polygonSet
    ps = result;
}
// SPLIT LONG EDGES
