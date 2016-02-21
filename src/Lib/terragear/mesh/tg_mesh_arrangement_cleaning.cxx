#include <CGAL/Snap_rounding_traits_2.h>
#include <CGAL/Snap_rounding_2.h>

#include <simgear/debug/logstream.hxx>

#include <terragear/polygon_set/tg_polygon_accumulator.hxx>

// TODO - cluster used by vector intersection code, and mesh - let's clean it up
// to show how generic it is.
#include <terragear/tg_cluster.hxx>

#include "tg_mesh.hxx"
#include "../polygon_set/tg_polygon_set.hxx"

void tgMesh::doClusterEdges( const tgCluster& cluster )
{
    meshArrEdgeConstIterator eit;
    std::vector<cgalPoly_Segment> segs;
    for (eit = meshArr.edges_begin(); eit != meshArr.edges_end(); eit++) {
        cgalPoly_Point source, target;
        
        source = cluster.Locate( eit->curve().source() );
        target = cluster.Locate( eit->curve().target() );
        
        if ( source != target ) {
            segs.push_back( cgalPoly_Segment( source, target ) );
        }
    }
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment clear old arr, and recreate" );
    // wipe the original arrangement clean and regenerate with the new segment 
    // list
    meshArr.clear();
    CGAL::insert( meshArr, segs.begin(), segs.end() );    
}

typedef CGAL::Snap_rounding_traits_2<meshArrKernel> srTraits;
typedef std::list<meshArrSegment>                   srSegmentList;
typedef std::list<meshArrPoint>                     srPolyline;
typedef std::list<srPolyline>                       srPolylineList;

tgMesh::SrcPointOp_e tgMesh::checkPointNearEdge( const cgalPoly_Point& pt, meshArrFaceConstHandle fh, cgalPoly_Point& projPt )
{
    //    const meshArr_FT distThreshSq(0.0000000005);
    const meshArr_FT distThreshSq(0.00000002);
    static unsigned int ptKeep = 0;
    static unsigned int ptDelete = 0;
    static unsigned int ptProject = 0;
    SrcPointOp_e retVal;
    
    if ( fh->has_outer_ccb() ) {
        // traverse the face, and find the closest edge
        meshArrHalfedgeConstCirculator ccb = fh->outer_ccb();
        meshArrHalfedgeConstCirculator cur = ccb;
        meshArrHalfedgeConstHandle     curHe;
        meshArrHalfedgeConstHandle     closestHe;
        meshArr_FT                     closestDistSq(100);
        
        do {
            curHe = cur;
            
            // calc distance between he and point
            cgalPoly_Segment seg( curHe->source()->point(), curHe->target()->point() );
            meshArr_FT distSq = CGAL::squared_distance( pt, seg );
            
            if ( distSq < closestDistSq ) {
                closestDistSq = distSq;
                closestHe     = curHe;
            }
            
            cur++;
        } while ( cur != ccb );
        
        if ( closestDistSq < distThreshSq ) {
            // project the point onto the line - if it is outside the bb, remove it
            cgalPoly_Line    edgeLine( closestHe->source()->point(), closestHe->target()->point() );
            cgalPoly_Segment edgeSegment( closestHe->source()->point(), closestHe->target()->point() );
            cgalPoly_Point   ptProj = edgeLine.projection( pt );
            
            if ( CGAL::do_overlap( edgeSegment.bbox(), ptProj.bbox() ) ) {
                // we have a winner
                GDALDataset*  poDS = NULL;
                OGRLayer*     poLineLayer = NULL;
                OGRLayer*     poPointLayer = NULL;
                
                poDS = openDatasource( datasource );
                if ( poDS ) {
                    poLineLayer = openLayer( poDS, wkbLineString25D, "closest Egde" );
                    
                    if ( poLineLayer ) {
                        toShapefile( poLineLayer, closestHe->curve(), "closest" );
                    }
                    
                    poPointLayer = openLayer( poDS, wkbPoint25D, "point" );
                    if ( poPointLayer ) {
                        toShapefile( poPointLayer, pt, "point" );
                    }
                    
                    GDALClose( poDS );    
                }
                
                ptProject++;
                projPt = ptProj;
                retVal = SRC_POINT_PROJECTED;
            } else {
                ptDelete++;
                retVal = SRC_POINT_DELETED;
            }
        } else {
            ptKeep++;
            retVal = SRC_POINT_OK;
        }
    } else {
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - Closest Edge no outer ccb " );        
        retVal = SRC_POINT_OK;
    }
    
//  SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh ptKeep " << ptKeep << " ptDelete " << ptDelete << " ptProject " << ptProject );
    
    return retVal;
}

void tgMesh::doRemoveAntenna( void )
{
    // remove all edges that have the same face on both sides
    bool edgeRemoved;
    do {
        meshArrEdgeIterator eit;
        edgeRemoved = false;
        for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {
            if ( eit->face() == eit->twin()->face() ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "tgMesh::cleanArrangmentFound antenna in cleaned arrangement" );
                CGAL::remove_edge( meshArr, eit );
                edgeRemoved = true;
                break;
            }
        }
    } while (edgeRemoved);
}

void tgMesh::doSnapRound( void )
{
    srSegmentList  srInput;
    srPolylineList srOutput;
    
    meshArrEdgeIterator eit;
    for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {
        srInput.push_back( eit->curve() );
    }
    
    // snap rounding notes:
    // 1) doesn't appear to be threadsafe
    // 2) no way to define the origin of snapping.  so if a point is at 0,0, and pixel size is 1, new point will be at 0.5, 0.5.
    //    We don't want this, so we translate the entire dataset back by 1/2 pixel size so 0,0 is still 0,0
    
    lock->lock();
    CGAL::snap_rounding_2<srTraits, srSegmentList::const_iterator, srPolylineList>
    (srInput.begin(), srInput.end(), srOutput, 0.0000002, true, false, 5);
    lock->unlock();
    
    std::vector<cgalPoly_Segment> segs;
    
    srPolylineList::const_iterator iter1;
    for (iter1 = srOutput.begin(); iter1 != srOutput.end(); ++iter1) {
        srPolyline::const_iterator itSrc = iter1->begin();
        srPolyline::const_iterator itTrg = itSrc; itTrg++;
        while (itTrg != iter1->end()) {
            cgalPoly_Point src( itSrc->x() - 0.0000001, itSrc->y() - 0.0000001 );
            cgalPoly_Point trg( itTrg->x() - 0.0000001, itTrg->y() - 0.0000001 );
            
            segs.push_back( cgalPoly_Segment(src, trg) );
            itSrc++; itTrg++;
        }
    }
    
    meshArr.clear();
    CGAL::insert( meshArr, segs.begin(), segs.end() );
    
    // snap round the elevation point, too
    srTraits srT;
    
    for ( unsigned int i=0; i<sourcePoints.size(); i++ ) {
        meshArr_FT x, y;
        
        srT.snap_2_object()(sourcePoints[i], 0.0000002, x, y);
        
        sourcePoints[i] = meshArrPoint( x - 0.0000001, y - 0.0000001 );
    }
}

void tgMesh::doProjectPointsToEdges( const tgCluster& cluster )
{
    // This function projects elevation points onto face edges when 
    // the distance to the face edge is below a threshold.
    // if elevation point is really close to a constraint, the mesh
    // blows up with tiny triangles...
    std::vector<cgalPoly_Point>::iterator pit = sourcePoints.begin();
    while( pit != sourcePoints.end() ) {        
        *pit = cluster.Locate( *pit );
        
        CGAL::Object obj = meshPointLocation.locate(*pit);
        
        meshArrFaceConstHandle      f;
        meshArrHalfedgeConstHandle  e;
        meshArrVertexConstHandle    v;
        
        if (CGAL::assign(e, obj)) {
            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT found on edge!" );
            pit = sourcePoints.erase(pit);
        } else if (CGAL::assign(v, obj)) {
            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT found on vertex!" );
            pit = sourcePoints.erase(pit);
        } else if (CGAL::assign(f, obj)) {
            if ( !f->is_unbounded() ) {
                cgalPoly_Point newPt;
                
                // check if the point is near a face edge
                switch( checkPointNearEdge( *pit, f, newPt ) ) {
                    case SRC_POINT_OK:
                        pit++;
                        break;
                        
                    case SRC_POINT_DELETED:
                        pit = sourcePoints.erase(pit);
                        break;
                        
                    case SRC_POINT_PROJECTED:
                        *pit = newPt;
                        pit++;
                        break;
                }
            } else {
                SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - Elevation POINT found on unbounded FACE! - erasing" );
                pit++;
            }
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT not found!" );
            pit = sourcePoints.erase(pit);            
        }        
    }
}