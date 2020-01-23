#include <CGAL/Snap_rounding_traits_2.h>
#include <CGAL/Snap_rounding_2.h>

#include <mutex>

#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

// snap rounding looks like it causes more issues than it solves.
// currently unused.
// a few issues:
// 1 - cgal translates points to the center of the pixel.  
//     I wish we could snap to the sw corner of the pixel.
//     We need to translate all points sw by 1/2 pixel size after snap rounding
// 2 - it looks like if we are on the pixel border, we sometimes translate
//     'the wrong way' making tile edge in the incorrect position.
//     I found this when tile matching broke for just a few tiles 
//     ( I beleive this is due to fp roundoff errors )
// 3 - CGAL snaprounding is apparently not threadsafe - I had to protect the whole 
//     procedure with a mutex.

// instead of snaprounding, we remove small areas, then small angles ( spikes )
// not as simple to code, but hopefully doesn't cause as many issues.

typedef CGAL::Snap_rounding_traits_2<meshArrKernel> srTraits;
typedef std::list<meshArrSegment>                   srSegmentList;
typedef std::vector<meshArrPoint>                   srPointList;
typedef std::list<meshArrPoint>                     srPolyline;
typedef std::list<srPolyline>                       srPolylineList;

void tgMeshArrangement::doSnapRound( std::mutex* lock )
{
    srSegmentList  srInputSegs;
    srPolylineList srOutputSegs;
    srPointList    srInputPoints;

    meshArrEdgeIterator eit;
    for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {
        srInputSegs.push_back( eit->curve() );
    }

    meshArrVertexIterator vit;
    for ( vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); ++vit ) {
        if ( vit->is_isolated() ) {
            srInputPoints.push_back( vit->point() );
        }
    }

    // snap rounding notes:
    // 1) doesn't appear to be threadsafe
    // 2) no way to define the origin of snapping.  so if a point is at 0,0, and pixel size is 1, new point will be at 0.5, 0.5.
    //    We don't want this, so we translate the entire dataset back by 1/2 pixel size so 0,0 is still 0,0

#define SR_OFFSET  (0.0000001)
//#define SR_OFFSET  (0)

    lock->lock();
    CGAL::snap_rounding_2<srTraits, srSegmentList::const_iterator, srPolylineList>
    (srInputSegs.begin(), srInputSegs.end(), srOutputSegs, 0.0000002, true, false, 5);
    lock->unlock();

    std::vector<meshArrSegment> segs;

    srPolylineList::const_iterator iter1;
    for (iter1 = srOutputSegs.begin(); iter1 != srOutputSegs.end(); ++iter1) {
        srPolyline::const_iterator itSrc = iter1->begin();
        srPolyline::const_iterator itTrg = itSrc; itTrg++;
        while (itTrg != iter1->end()) {
            meshArrPoint src( itSrc->x() - SR_OFFSET, itSrc->y() - SR_OFFSET );
            meshArrPoint trg( itTrg->x() - SR_OFFSET, itTrg->y() - SR_OFFSET );

            segs.push_back( meshArrSegment(src, trg) );
            itSrc++; itTrg++;
        }
    }

    meshArr.clear();
    CGAL::insert( meshArr, segs.begin(), segs.end() );

    // snap round the isolated vertices, too
    srTraits srT;
    for ( unsigned int i=0; i<srInputPoints.size(); i++ ) {
        meshArr_FT x, y;
        srT.snap_2_object()(srInputPoints[i], 0.0000002, x, y);

        CGAL::insert_point( meshArr, meshArrPoint( x - SR_OFFSET, y - SR_OFFSET ) );
    }
}
