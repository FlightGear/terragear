#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

tgMeshArrangement::SrcPointOp_e tgMeshArrangement::checkPointNearEdge( const meshArrPoint& pt, meshArrFaceConstHandle fh, meshArrPoint& projPt )
{
    const meshArr_FT    distThreshSq(0.0000000005);
    static unsigned int ptKeep = 0;
    static unsigned int ptDelete = 0;
    static unsigned int ptProject = 0;
    SrcPointOp_e        retVal;

    if ( fh->has_outer_ccb() ) {
        meshArrKernel::Compute_squared_distance_2 squared_distance;

        // traverse the face, and find the closest edge
        meshArrHalfedgeConstCirculator ccb = fh->outer_ccb();
        meshArrHalfedgeConstCirculator cur = ccb;
        meshArrHalfedgeConstHandle     curHe;
        meshArrHalfedgeConstHandle     closestHe;
        meshArr_FT                     closestDistSq(100);

        do {
            curHe = cur;

            // calc distance between he and point
            meshArrSegment  seg( curHe->source()->point(), curHe->target()->point() );
            meshArr_FT      distSq = squared_distance( pt, seg );

            if ( distSq < closestDistSq ) {
                closestDistSq = distSq;
                closestHe     = curHe;
            }

            cur++;
        } while ( cur != ccb );

        if ( closestDistSq < distThreshSq ) {
            // project the point onto the line - if it is outside the bb, remove it
            // ( it's projection is really the endpoint of the segment )
            meshArrLine    edgeLine( closestHe->source()->point(), closestHe->target()->point() );
            meshArrSegment edgeSegment( closestHe->source()->point(), closestHe->target()->point() );
            meshArrPoint   ptProj = edgeLine.projection( pt );

            if ( CGAL::do_overlap( edgeSegment.bbox(), ptProj.bbox() ) ) {
                // we have a winner
                GDALDataset*  poDS = NULL;
                OGRLayer*     poLineLayer = NULL;
                OGRLayer*     poPointLayer = NULL;

                poDS = mesh->openDatasource( mesh->getDebugPath() );
                if ( poDS ) {
                    poLineLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, "closest Egde" );

                    if ( poLineLayer ) {
                        toShapefile( poLineLayer, closestHe->curve(), "closest" );
                    }

                    poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, "point" );
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

void tgMeshArrangement::doProjectPointsToEdges( const tgCluster& cluster )
{
    // This function projects elevation points onto face edges when 
    // the distance to the face edge is below a threshold.
    // if elevation point is really close to a constraint, the mesh
    // blows up with tiny triangles...
    std::vector<meshArrPoint>        addList;
    std::vector<meshArrVertexHandle> removeList;
    std::vector<meshArrVertexHandle>::iterator rlit;

    for( meshArrVertexIterator vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); vit++ ) {
        // isolated points are elevation points.
        // but elevations points need not be isolated
        if ( vit->is_isolated() ) {
            CGAL::Object obj = meshPointLocation.locate(vit->point());
            meshArrFaceConstHandle f;

            if ( CGAL::assign(f, obj) ) {
                if ( !f->is_unbounded() ) {
                    meshArrPoint newPt;

                    // check if the point is near a face edge
                    switch( checkPointNearEdge( vit->point(), f, newPt ) ) {
                        case SRC_POINT_OK:
                            break;

                        case SRC_POINT_DELETED:
                            // add this vertex to the remove list
                            removeList.push_back( vit );
                            break;

                        case SRC_POINT_PROJECTED:
                            removeList.push_back( vit );
                            addList.push_back( newPt );
                            break;
                    }
                } else {
                    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - Elevation POINT found on unbounded FACE! - erasing" );
                    removeList.push_back( vit );
                }
            } else {
                SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT not found!" );
                removeList.push_back( vit );
            }
        } else {
            // non-isolated 
        }
    }

    // remove 
    for( rlit = removeList.begin(); rlit != removeList.end(); rlit++ ) {
        CGAL::remove_vertex( meshArr, *rlit );
    }

    // add new
    for( unsigned int i=0; i<addList.size(); i++ ) {
        CGAL::insert_point( meshArr, addList[i] );
    }
}