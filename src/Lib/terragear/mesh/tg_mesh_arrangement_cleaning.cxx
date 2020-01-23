#include <simgear/debug/logstream.hxx>

#include <mutex>

// TODO - cluster used by vector intersection code, and mesh - let's clean it up
// to show how generic it is.
#include <terragear/tg_cluster.hxx>

#include "tg_mesh.hxx"
#include "../polygon_set/tg_polygon_set.hxx"

#define DEBUG_MESH_CLEANING (0)

void tgMeshArrangement::doClusterEdges( const tgCluster& cluster )
{
    meshArrEdgeConstIterator eit;
    std::vector<meshArrSegment> segs;
    for (eit = meshArr.edges_begin(); eit != meshArr.edges_end(); eit++) {
        cgalPoly_Point source, target;

        source = cluster.Locate( toCpPoint( eit->curve().source()) );
        target = cluster.Locate( toCpPoint( eit->curve().target()) );

        if ( source != target ) {
            segs.push_back( meshArrSegment( toMeshArrPoint(source), toMeshArrPoint(target) ) );
        }
    }
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment clear old arr, and recreate" );
    // wipe the original arrangement clean and regenerate with the new segment 
    // list
    meshArr.clear();
    CGAL::insert( meshArr, segs.begin(), segs.end() );    
}

bool tgMeshArrangement::isEdgeVertex( meshArrVertexConstHandle v )
{
    bool isEdge = false;

    if ( !v->is_isolated() ) {
        // a vertex is on the edge if anny incident edges ( or their twins )
        // are adjacent to the unbounded face
        // note isolated vertices cannot be on the edge :)
        meshArrIncidentHalfedgeConstCirculator firstCirc = v->incident_halfedges();
        meshArrIncidentHalfedgeConstCirculator   curCirc = firstCirc;

        do {
            meshArrHalfedgeConstHandle curHe = curCirc;

            if ( curHe->face()->is_unbounded() || 
                curHe->twin()->face()->is_unbounded() ) {
                //SG_LOG(SG_GENERAL, SG_INFO, "Edge vertex " );
                isEdge = true;
                break;
            }

            curCirc++;
        } while ( curCirc != firstCirc );
    }

    return isEdge;
}

// Use Lloyd Voronoi relaxation to cluster and 
// remove nodes too close to one another.
void tgMeshArrangement::cleanArrangement( std::mutex* lock )
{
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMeshArrangement::cleanArrangment : start" );

#if DEBUG_MESH_CLEANING
    toShapefile( mesh->getDebugPath().c_str(), "arr_original" );
#endif

    // create the point list from the arrangement
    // TODO: cluster needs to know if a point can mode or not.
    // We don't want to average points on the tile edges...
    meshArrVertexConstIterator vit;
    std::list<tgClusterNode>   nodes;
    for ( vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); vit++ ) {
        nodes.push_back( tgClusterNode( toCpPoint(vit->point()), isEdgeVertex(vit) ) );
    }

    // create the cluster : not thread safe
    lock->lock();
    tgCluster cluster( nodes, 0.0000025, mesh->debugPath );
    lock->unlock();

#if DEBUG_MESH_CLEANING
    cluster.toShapefile( mesh->getDebugPath().c_str(), "cluster" );
#endif

    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment create new segments" );
    // CLEAN 1
    // collect the original segment list, and generate a new list
    // with clustered source / target points.
    // just add the segments that still exist
    doClusterEdges( cluster );

    // clean 2
    doRemoveAntenna();

#if DEBUG_MESH_CLEANING
    toShapefile( mesh->getDebugPath().c_str(), "arr_clustered" );
#endif

    // clean 3 - remove skinny faces
    // doRemoveSmallAreas();
    // doRemoveSpikes( lock );

    // clean 3
    // clustering may have moved an edge too close to a vertex - 
    // snap rounding creates as many issues as it solves.  
    // moving all points to a 'pixel' center moves edge nodes off of the edges.
    // getting them back is tricky.
    // maybe mark edges as 'special?
    // let's try without, first.
    doSnapRound( lock );

    // clean 4
    doRemoveAntenna();

#if DEBUG_MESH_CLEANING
    toShapefile( mesh->getDebugPath(), "arr_snapround" );
#endif

    // now attach the point locater to quickly find faces from points
    // need this for projecting
    meshPointLocation.attach( meshArr );

    // clean 5
    doProjectPointsToEdges( cluster );

    // cleaning done
#if DEBUG_MESH_CLEANING
    toShapefile( mesh->getDebugPath(), "arr_projected" );
#endif

    // traverse the original polys, and add the metadata / arrangement face lookups
    // TODO error if a face is added twice
    // this can happen if the topology is altered too much 
    // ( an interior point is no longer interior to the original poly )
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment create face lookup" );

#if DEBUG_MESH_CLEANING
    GDALDataset* poDs    = mesh->openDatasource( mesh->getDebugPath() );
    OGRLayer*    poLayer = mesh->openLayer( poDs, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, "unbounded_qps" );
#endif

    // face lookup function...
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator pit;
        for ( pit = sourcePolys[i].begin(); pit != sourcePolys[i].end(); pit++ ) {
            if ( !pit->isEmpty() ) {
                const std::vector<cgalPoly_Point>& queryPoints = pit->getInteriorPoints();
                for ( unsigned int i=0; i<queryPoints.size(); i++ ) {
                    CGAL::Object obj = meshPointLocation.locate( toMeshArrPoint(queryPoints[i]) );

                    meshArrFaceConstHandle      f;
                    meshArrHalfedgeConstHandle  e;
                    meshArrVertexConstHandle    v;

                    if (CGAL::assign(f, obj)) {
                        // point is in face - set the material, and the query point, so we can save it
                        if ( !f->is_unbounded() ) {
                            metaLookup.push_back( tgMeshFaceMeta(f, queryPoints[i], pit->getMeta() ) );
                        } else {
                            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT " << i << " queryPoint found on unbounded FACE!" );
#if DEBUG_MESH_CLEANING
                            // add to debug layer
                            if ( poLayer ) {
                                toShapefile( poLayer, queryPoints[i], "qp" );
                            }
#endif
                        }
                    } else if (CGAL::assign(e, obj)) {
                        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT " << i << " found on edge!" );                    
                    } else if (CGAL::assign(v, obj)) {
                        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT " << i << " found on vertex!" );                    
                    } else {
                        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT " << i << " not found!" );                    
                    }
                }
            }
        }
    }

#if DEBUG_MESH_CLEANING
    GDALClose( poDs );    
#endif

    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment Complete" );

#if DEBUG_MESH_CLEANING
    // debug function...
    toShapefile( mesh->getDebugPath(), "arr_clean" );

    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer = NULL;

    poDS = mesh->openDatasource( mesh->getDebugPath() );
    if ( poDS ) {
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, "arr_queryPoints" );

        for ( unsigned int i=0; i<numPriorities; i++ ) {
            std::vector<tgPolygonSet>::iterator pit;
            for ( pit = sourcePolys[i].begin(); pit != sourcePolys[i].end(); pit++ ) {
                if ( !pit->isEmpty() ) {
                    const std::vector<cgalPoly_Point>& queryPoints = pit->getInteriorPoints();
                    for ( unsigned int i=0; i<queryPoints.size(); i++ ) {
                        toShapefile( poPointLayer, queryPoints[i], "qp" );
                    }
                }
            }
        }

        // close datasource
        GDALClose( poDS );    
    }
#endif
}
