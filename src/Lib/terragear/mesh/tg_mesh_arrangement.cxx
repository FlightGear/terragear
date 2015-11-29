#include <simgear/debug/logstream.hxx>

#include <terragear/polygon_set/tg_polygon_accumulator.hxx>

// TODO - cluster used by vector intersection code, and mesh - let's clean it up
// to show how generic it is.
#include <terragear/tg_cluster.hxx>

#include "tg_mesh.hxx"
#include "../polygon_set/tg_polygon_set.hxx"

// perform polygon clipping ( the soup )
void tgMesh::clipPolys( void )
{
    tgAccumulator accum;
    cgalPoly_Polygon bucketPoly;
    
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

            accum.Diff_and_Add_cgal( current );                        
            if ( clipBucket ) {                
                // then clip against bucket
                current.intersection2( bucketPoly );
            }
            
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
  //tgCluster cluster( nodes, 0.0001000 );    
  //tgCluster cluster( nodes, 0.0000050 );
    tgCluster cluster( nodes, 0.0000025 );
  //tgCluster cluster( nodes, 0.0000010 );
  //cluster.toShapefile( datasource, "cluster" );

    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::cleanArrangment create new segments" );
    
    // collect the original segment list, and generate a new list
    // with clustered source / target points.
    // just add the segments that still exist
#if 1
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

    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::cleanArrangment clear old arr, and recreate" );
    
    // wipe the original arrangement clean and regenerate with the new segment 
    // list
    meshArr.clear();
    CGAL::insert( meshArr, segs.begin(), segs.end() );
#endif

    // now attach the point locater to quickly find faces from points
    meshPointLocation.attach( meshArr );
    
    // traverse the original polys, and add the metadata / arrangement face lookups
    // TODO error if a face is added twice
    // this can happen if the topology is altered too much 
    // ( an interior point is no longer interior to the original poly )
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::cleanArrangment create face lookup" );
    
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
                            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::tgMesh - POINT " << i << " found on unbounded FACE!" );
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
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::cleanArrangment Complete" );
    toShapefile( datasource, "arr_clean", meshArr );    
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

    meshPointLocation.attach( meshArr );
}

// lookup a face in the arrangement from a face in the arrangement
// seems silly, but we are looking for just faces that are in the 
// lookup table.
meshArrFaceConstHandle tgMesh::findPolyFace( meshArrFaceConstHandle f )
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
meshArrFaceConstHandle tgMesh::findMeshFace( const meshTriPoint& tPt )
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
    // insert the polygon boundaries ( not holes ) 
    // TODO - maybe we need holes, too?  - Haven't seen a need yet.
    std::vector<cgalPoly_Segment> segs;
    pit->toSegments( segs, false );    
    
    CGAL::insert( meshArr, segs.begin(), segs.end() );
}