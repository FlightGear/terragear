#include <simgear/debug/logstream.hxx>

#include <terragear/polygon_set/tg_polygon_accumulator.hxx>

// TODO - cluster used by vector intersection code, and mesh - let's clean it up
// to show how generic it is.
#include <terragear/tg_cluster.hxx>

#include "tg_mesh.hxx"
#include "../polygon_set/tg_polygon_set.hxx"

#define DEBUG_MESH_CLIPPING (0)
#define DEBUG_MESH_CLEANING (1)

// perform polygon clipping ( the soup )
void tgMesh::clipPolys( void )
{
    tgAccumulator    accum;
    cgalPoly_Polygon bucketPoly;
    
#if DEBUG_MESH_CLIPPING            
    static int polyNum = 1;
#endif
    
    if ( clipBucket ) {
        // create exact bucket
        bucketPoly.push_back( cgalPoly_Point( b.get_corner( SG_BUCKET_SW ).getLongitudeDeg(), b.get_corner( SG_BUCKET_SW ).getLatitudeDeg() ) );
        bucketPoly.push_back( cgalPoly_Point( b.get_corner( SG_BUCKET_SE ).getLongitudeDeg(), b.get_corner( SG_BUCKET_SE ).getLatitudeDeg() ) );
        bucketPoly.push_back( cgalPoly_Point( b.get_corner( SG_BUCKET_NE ).getLongitudeDeg(), b.get_corner( SG_BUCKET_NE ).getLatitudeDeg() ) );
        bucketPoly.push_back( cgalPoly_Point( b.get_corner( SG_BUCKET_NW ).getLongitudeDeg(), b.get_corner( SG_BUCKET_NW ).getLatitudeDeg() ) );
    }
    
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator poly_it;
        for ( poly_it = sourcePolys[i].begin(); poly_it != sourcePolys[i].end(); poly_it++ ) {
            tgPolygonSet current = (*poly_it);
    
#if DEBUG_MESH_CLIPPING            
            char layerName[64];
            GDALDataset* poDs = tgPolygonSet::openDatasource( datasource.c_str() );

            sprintf( layerName, "%s_%s_%d_orig", b.gen_index_str().c_str(), current.getMeta().material.c_str(), polyNum );
            OGRLayer*    poLayerOrig = tgPolygonSet::openLayer( poDs, wkbLineString25D, tgPolygonSet::LF_DEBUG, layerName );            
            tgPolygonSet::toDebugShapefile( poLayerOrig, current.getPs(), "orig" );
#endif
            
            accum.Diff_and_Add_cgal( current );                        

#if DEBUG_MESH_CLIPPING            
            sprintf( layerName, "%s_%s_%d_clip", b.gen_index_str().c_str(), current.getMeta().material.c_str(), polyNum );
            OGRLayer*    poLayerClip = tgPolygonSet::openLayer( poDs, wkbLineString25D, tgPolygonSet::LF_DEBUG, layerName );            
            tgPolygonSet::toDebugShapefile( poLayerClip, current.getPs(), "clip" );
#endif
            
            if ( clipBucket ) { 
                
#if DEBUG_MESH_CLIPPING                            
                sprintf( layerName, "%s_%s_%d_bucket", b.gen_index_str().c_str(), current.getMeta().material.c_str(), polyNum );
                OGRLayer*    poLayerBucket = tgPolygonSet::openLayer( poDs, wkbLineString25D, tgPolygonSet::LF_DEBUG, layerName );            
                tgPolygonSet::toDebugShapefile( poLayerBucket, bucketPoly, "bucket" );
#endif
                
                // then clip against bucket
                current.intersection2( bucketPoly );
                
#if DEBUG_MESH_CLIPPING            
                sprintf( layerName, "%s_%s_%d_clip_bucket", b.gen_index_str().c_str(), current.getMeta().material.c_str(), polyNum );
                OGRLayer*    poLayerClipBucket = tgPolygonSet::openLayer( poDs, wkbLineString25D, tgPolygonSet::LF_DEBUG, layerName );            
                tgPolygonSet::toDebugShapefile( poLayerClipBucket, current.getPs(), "clipBucket" );                
#endif

            }

#if DEBUG_MESH_CLIPPING            
            GDALClose( poDs );
#endif
            
            poly_it->setPs( current.getPs() );
        }
    }
}


// Use Lloyd Voronoi relaxation to cluster and 
// remove nodes too close to one another.
void tgMesh::cleanArrangement( void )
{        
    // create the point list from the arrangement
    meshArrVertexConstIterator vit;
    std::list<cgalPoly_Point>  nodes;
    for ( vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); vit++ ) {
        nodes.push_back( vit->point() );
    }

    // create the cluster
    tgCluster cluster( nodes, 0.0000025 );
    //cluster.toShapefile( datasource, "cluster" );

    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment create new segments" );
    
    
    // CLEAN 1
    // collect the original segment list, and generate a new list
    // with clustered source / target points.
    // just add the segments that still exist
    doClusterEdges( cluster );
#if DEBUG_MESH_CLEANING
    toShapefile( datasource, "arr_clustered", meshArr );
#endif
    
    // clean 2
    doRemoveAntenna();
#if DEBUG_MESH_CLEANING
    toShapefile( datasource, "arr_noantenna", meshArr );
#endif
    
    // clean 3
    // clustering may have moved an edge too close to a vertex - 
    doSnapRound();
#if DEBUG_MESH_CLEANING
    toShapefile( datasource, "arr_snapround", meshArr );    
#endif
    
    // clean 4
    doRemoveAntenna();

    // now attach the point locater to quickly find faces from points
    meshPointLocation.attach( meshArr );

    // clean 5
    doProjectPointsToEdges( cluster );
    
    // cleaning done
    
    // traverse the original polys, and add the metadata / arrangement face lookups
    // TODO error if a face is added twice
    // this can happen if the topology is altered too much 
    // ( an interior point is no longer interior to the original poly )
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment create face lookup" );
    
    // face lookup function...
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator pit;
        for ( pit = sourcePolys[i].begin(); pit != sourcePolys[i].end(); pit++ ) {
            if ( !pit->isEmpty() ) {
                const std::vector<cgalPoly_Point>& queryPoints = pit->getInteriorPoints();
                for ( unsigned int i=0; i<queryPoints.size(); i++ ) {
                    CGAL::Object obj = meshPointLocation.locate(queryPoints[i]);
                
                    meshArrFaceConstHandle      f;
                    meshArrHalfedgeConstHandle  e;
                    meshArrVertexConstHandle    v;
        
                    if (CGAL::assign(f, obj)) {
                        // point is in face - set the material
                        if ( !f->is_unbounded() ) {
                            metaLookup.push_back( tgMeshFaceMeta(f, pit->getMeta() ) );
                        } else {
                            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT " << i << " queryPoint found on unbounded FACE!" );
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
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMesh::cleanArrangment Complete" );

#if DEBUG_MESH_CLEANING
    // debug function...
    toShapefile( datasource, "arr_clean", meshArr );
        
    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer = NULL;
    
    poDS = openDatasource( datasource );
    if ( poDS ) {        
        poPointLayer = openLayer( poDS, wkbPoint25D, "arr_queryPoints" );
        
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

// insert the polygon segments into an arrangement
void tgMesh::arrangePolys( void )
{
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator poly_it;
        for ( poly_it = sourcePolys[i].begin(); poly_it != sourcePolys[i].end(); poly_it++ ) {
            // only add to arrangement if we have a result
            if ( !poly_it->isEmpty() ) {
                poly_it->calcInteriorPoints();
                arrangementInsert( poly_it );
            }
        }
    }
    
    // then add the source points
    std::vector<cgalPoly_Point>::iterator spit;
    for ( spit = sourcePoints.begin(); spit != sourcePoints.end(); spit++ ) {
        CGAL::insert_point( meshArr, *spit );
    }
    
#if DEBUG_MESH_CLEANING    
    toShapefile( datasource, "arr_raw", meshArr );
#endif
    
    meshPointLocation.attach( meshArr );
}

// lookup a face in the arrangement from a face in the arrangement
// seems silly, but we are looking for just faces that are in the 
// lookup table.
meshArrFaceConstHandle tgMesh::findPolyFace( meshArrFaceConstHandle f ) const
{
    meshArrFaceConstHandle face = (meshArrFaceConstHandle)NULL;
    bool found = false;
    
    for ( unsigned int i=0; i<metaLookup.size() && !found; i++ ) {
        if ( metaLookup[i].face == f ) {
            face = f;
            found = true;
        }
    }

    return face;
}

// lookup a face in the arrangement from a point in the triangulation
// need to convert the point from EPICK to EPECK
meshArrFaceConstHandle tgMesh::findMeshFace( const meshTriPoint& tPt ) const
{
    meshArrPoint aPt = toMeshArrPoint( tPt );
    
    CGAL::Object obj = meshPointLocation.locate(aPt);
    
    meshArrFaceConstHandle      f, result;
    meshArrHalfedgeConstHandle  e;
    meshArrVertexConstHandle    v;

    result = (meshArrFaceConstHandle)NULL;
    if (CGAL::assign(f, obj)) {
        // point is in face - this is what we want
        if ( !f->is_unbounded() ) {
            // we found a face - is it in our lookup table?
            if ( findPolyFace( f ) != (meshArrFaceConstHandle)NULL ) {
                result = f;
            }
        }
    } else if (CGAL::assign(e, obj)) {
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::findMeshFace - POINT " << tPt << " found on edge!" );                    
    } else if (CGAL::assign(v, obj)) {
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::findMeshFace - POINT " << tPt << " found on vertex!" );                    
    } else {
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::findMeshFace - POINT " << tPt << " not found!" );                    
    }
    
    return result;
}


void tgMesh::arrangementInsert( const std::vector<tgPolygonSet>::iterator pit )
{    
    //static unsigned int num_poly = 1;
    //char layername[64];
    
    //sprintf( layername, "arr_raw_%04d", num_poly++ );
    
    // insert the polygon boundaries ( not holes ) 
    // TODO - maybe we need holes, too?  - Haven't seen a need yet.
    std::vector<cgalPoly_Segment> segs;
    pit->toSegments( segs, false );    
    
    CGAL::insert( meshArr, segs.begin(), segs.end() );
    
    //toShapefile( datasource, layername, meshArr );
}