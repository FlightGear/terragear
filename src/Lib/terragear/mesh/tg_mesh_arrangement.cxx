#include <simgear/debug/logstream.hxx>

#include <terragear/polygon_set/tg_polygon_accumulator.hxx>

#include "tg_mesh.hxx"
#include "../polygon_set/tg_polygon_set.hxx"

#define DEBUG_MESH_CLIPPING (0)

void tgMeshArrangement::initPriorities( const std::vector<std::string>& names )
{
    priorityNames = names;
    numPriorities = names.size();

    for ( unsigned int i=0; i<numPriorities; i++ ) {
        tgPolygonSetList lc;
        sourcePolys.push_back( lc );
    }
}

bool tgMeshArrangement::empty( void )
{
    bool empty = true;

    // check source polys
    for ( unsigned int i=0; empty && i<numPriorities; i++ ) {
        empty = sourcePolys[i].empty();
    }

    return empty;
}

void tgMeshArrangement::addPoly( unsigned int priority, const tgPolygonSet& poly )
{
    sourcePolys[priority].push_back( poly );
}

void tgMeshArrangement::addPolys( unsigned int priority, const tgPolygonSetList& polys )
{
    sourcePolys[priority].insert( sourcePolys[priority].end(), polys.begin(), polys.end() );
}

void tgMeshArrangement::addPoints( const std::vector<cgalPoly_Point>& points )
{
    sourcePoints.insert( sourcePoints.end(), points.begin(), points.end() );
}

tgPolygonSet tgMeshArrangement::join( unsigned int priority, const tgPolygonSetMeta& meta )
{
    return tgPolygonSet::join( sourcePolys[priority], meta );
}

void tgMeshArrangement::getPoints( std::vector<meshTriPoint>& points ) const
{
    meshArrVertexConstIterator  vit;

    for ( vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); vit++ ) {
        if ( vit->is_isolated() ) {
            points.push_back( toMeshTriPoint( vit->point() ) );
        }
    }
}

void tgMeshArrangement::getSegments( std::vector<meshTriSegment>& constraints  ) const
{
    meshArrEdgeConstIterator    eit;

    for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {
        meshTriPoint source = toMeshTriPoint( eit->curve().source() );
        meshTriPoint target = toMeshTriPoint( eit->curve().target() );

        if ( source != target ) {
            constraints.push_back( meshTriSegment(source, target ) );
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "tgMeshArrangement : found segment with source == target" );
        }
    }
}

// perform polygon clipping ( the soup )
void tgMeshArrangement::clipPolys( const SGBucket& b, bool clipBucket )
{
    tgAccumulator    accum;
    cgalPoly_Polygon bucketPoly;

#if DEBUG_MESH_CLIPPING
    static int polyNum = 1;
#endif

    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMeshArrangement::clipPolys : start" );

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
            GDALDataset* poDs = tgPolygonSet::openDatasource( mesh->getDebugPath().c_str() );

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



// insert the polygon segments into an arrangement
void tgMeshArrangement::arrangePolys( void )
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

    // then add the elevation points
    //mesh->lock->lock();
    std::vector<cgalPoly_Point>::iterator spit;
    for ( spit = sourcePoints.begin(); spit != sourcePoints.end(); spit++ ) {
        CGAL::insert_point( meshArr, toMeshArrPoint(*spit) );
        //CGAL::insert_point( meshArr, *spit );
    }
    //mesh->lock->unlock();

#if DEBUG_MESH_CLEANING    
    toShapefile( mesh->getDebugPath(), "arr_raw" );
#endif

    meshPointLocation.attach( meshArr );
}

// lookup a face in the arrangement from a face in the arrangement
// seems silly, but we are looking for just faces that are in the 
// lookup table.
meshArrFaceConstHandle tgMeshArrangement::findPolyFace( meshArrFaceConstHandle f ) const
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
meshArrFaceConstHandle tgMeshArrangement::findMeshFace( const meshTriPoint& tPt ) const
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

void tgMeshArrangement::toMeshArrSegs( const std::vector<cgalPoly_Segment>& inSegs, std::vector<meshArrSegment>& outSegs ) const
{
    for (unsigned int i=0; i<inSegs.size(); i++) {
        meshArrPoint srcPoint = toMeshArrPoint( inSegs[i].source() );
        meshArrPoint trgPoint = toMeshArrPoint( inSegs[i].target() );

        outSegs.push_back( meshArrSegment(srcPoint, trgPoint) );
    }
}

void tgMeshArrangement::arrangementInsert( const std::vector<tgPolygonSet>::iterator pit )
{
    // insert the polygon boundaries ( not holes ) 
    // TODO - maybe we need holes, too?  - Haven't seen a need yet.
    std::vector<cgalPoly_Segment> segs;
    std::vector<meshArrSegment>   arrSegs;

    pit->toSegments( segs, false );

    // convert poly segs to arr segs
    toMeshArrSegs( segs, arrSegs );

    mesh->lock->lock();
    CGAL::insert( meshArr, arrSegs.begin(), arrSegs.end() );
    mesh->lock->unlock();
}

void tgMeshArrangement::loadArrangement( const std::string& path )
{
    // the arrangement is a set of polygons - with a query point so we can find the face 
    // once the arrangement is set.
    std::vector<meshArrSegment> edgelist;
    std::string filePath;

    filePath = path + "/stage1_arrangement_faces.shp";
    fromShapefile( filePath, edgelist );

    // add edges to arrangement
    meshArr.clear();

    mesh->lock->lock();
    CGAL::insert( meshArr, edgelist.begin(), edgelist.end() );
    mesh->lock->unlock();

    // save it so we can see it...
    // toShapefile( mesh->getDebugPath(), "stage2_arrangement" );
}