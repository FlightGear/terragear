#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

void tgMesh::initPriorities( const std::vector<std::string>& names )
{
    meshArrangement.initPriorities( names );
    clipBucket = false;
}

void tgMesh::initDebug( const std::string& dbgRoot )
{
    debugPath = dbgRoot;
}

void tgMesh::clipAgainstBucket( const SGBucket& bucket ) 
{
    clipBucket = true;
    b = bucket;
}

void tgMesh::clear( void )
{
    meshArrangement.clear();
    meshTriangulation.clear();
}

bool tgMesh::empty( void )
{
    return meshArrangement.empty();
}

void tgMesh::addPoly( unsigned int priority, const tgPolygonSet& poly )
{
    meshArrangement.addPoly( priority, poly );
}

void tgMesh::addPolys( unsigned int priority, const tgPolygonSetList& polys )
{
    meshArrangement.addPolys( priority, polys );
}

void tgMesh::addPoints( const std::vector<cgalPoly_Point>& points )
{
    meshArrangement.addPoints( points );
}

tgPolygonSet tgMesh::join( unsigned int priority, const tgPolygonSetMeta& meta )
{
    return meshArrangement.join( priority, meta );
}

void tgMesh::generate( void )
{
    // mesh generation from polygon soup :)
    if ( !meshArrangement.empty() ) {
        // Step 1 - clip polys against one another - highest priority first ( on top )
        meshArrangement.clipPolys( b, clipBucket );

        // Step 2 - insert clipped polys into an arrangement.
        // From this point on, we don't need the individual polygons.
        meshArrangement.arrangePolys();

        // step 3 - clean up the arrangement - cluster nodes that are too close - don't want
        // really small triangles blowing up the refined mesh.
        // NOTE / TODO: 
        // The cluster size MUST be smaller than the minimum distance of interiorPoints.
        // we should remember be checking the delta in interiorPoints to see if we have 
        // polys that don't meat this criteria.
        // and if it doesn't - what do we do?
        meshArrangement.cleanArrangement( lock );

        // step 4 - create constrained triangulation with arrangement edges as the constraints
        meshTriangulation.constrainedTriangulateWithEdgeModification( meshArrangement );

        // step 5 - prepare for serialization
        meshTriangulation.prepareTds();
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "no source polys" );        
    }
}

bool tgMesh::loadStage1( const std::string& basePath, const SGBucket& bucket )
{
    std::string bucketPath = basePath + "/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
    bool isOcean = false;
    b = bucket;

    // now load the stage1 triangulation ( and lookup locations on the edges )
    if ( !meshTriangulation.loadTriangulation( basePath, bucket ) ) {
        isOcean = true;
    } else {
        meshTriangulation.prepareTds();

        // load the arrangement so we know what material each triangle is.
        meshArrangement.loadArrangement( bucketPath );
    }

    return isOcean;
}

tgArray* tgMesh::loadElevationArray( const std::string& demBase, const SGBucket& bucket )
{
    std::string arrayPath = demBase + "/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
    tgArray* array = new tgArray();

    if ( array->open(arrayPath) ) {
        SG_LOG(SG_GENERAL, SG_INFO, "Opened Array file " << arrayPath);

        array->parse( bucket );
        array->remove_voids( );
        array->close();
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "Could not open Array file " << arrayPath);
    }

    return array;
}


void tgMesh::calcElevation( const std::string& basePath )
{
    // load this, and surrounding tile elevation data
    std::vector<tgArray*> northArrays;
    std::vector<SGBucket> northBuckets;
    b.siblings( -1, 1, northBuckets );
    b.siblings(  0, 1, northBuckets );
    b.siblings(  1, 1, northBuckets );
    for ( unsigned int i=0; i<northBuckets.size(); i++ ) {
        northArrays.push_back( loadElevationArray( basePath, northBuckets[i] ) );
    }

    std::vector<tgArray*> southArrays;
    std::vector<SGBucket> southBuckets;
    b.siblings( -1, -1, southBuckets );
    b.siblings(  0, -1, southBuckets );
    b.siblings(  1, -1, southBuckets );
    for ( unsigned int i=0; i<southBuckets.size(); i++ ) {
        southArrays.push_back( loadElevationArray( basePath, southBuckets[i] ) );
    }

    tgArray* eastArray;
    SGBucket eastBucket = b.sibling( 1, 0);
    eastArray = loadElevationArray( basePath, eastBucket );

    tgArray* westArray;
    SGBucket westBucket = b.sibling(-1, 0);
    westArray = loadElevationArray( basePath, westBucket );

    tgArray* tileArray = loadElevationArray( basePath, b );

    // first calc the elevation of all nodes in this tile.
    meshTriangulation.calcTileElevations( tileArray );

#if 0 // shared edges - is it needed?

    // first, calc the average at the 4 corners - 
    // each corner has 3 contributing tiles
    // on the north and south borders, we can get different sized arrays of 
    // buckets, depending on the widths
    // east and west are constant.
    unsigned int nwIndexes[2], neIndexes[2], swIndexes[2], seIndexes[2];
    switch( northArrays.size() ) {
        // north buckets are the same width as us
        case 3:
            nwIndexes[0] = 0;
            nwIndexes[1] = 1;
            neIndexes[0] = 1;
            neIndexes[1] = 2;
            break;

        default:
            SG_LOG(SG_GENERAL, SG_ALERT, "Unhandled array size " << northArrays.size() );
            exit(0);
            break;
    }

    switch( southArrays.size() ) {
        // north buckets are the same width as us
        case 3:
            swIndexes[0] = 0;
            swIndexes[1] = 1;
            seIndexes[0] = 1;
            seIndexes[1] = 2;
            break;

        default:
            SG_LOG(SG_GENERAL, SG_ALERT, "Unhandled array size " << northArrays.size() );
            exit(0);
            break;
    }

    #define SG_BUCKET_SW    (0)
    #define SG_BUCKET_SE    (1)
    #define SG_BUCKET_NE    (2)
    #define SG_BUCKET_NW    (3)
    SGGeod nwCorner = b.get_corner( SG_BUCKET_NW );
    SGGeod neCorner = b.get_corner( SG_BUCKET_NE );
    SGGeod seCorner = b.get_corner( SG_BUCKET_SE );
    SGGeod swCorner = b.get_corner( SG_BUCKET_SW );

    double elv1, elv2, elv3, elv4;
    elv1 = northArrays[nwIndexes[0]]->altitude_from_grid( nwCorner.getLongitudeDeg() * 3600.0, nwCorner.getLatitudeDeg() * 3600.0 );
    elv2 = northArrays[nwIndexes[1]]->altitude_from_grid( nwCorner.getLongitudeDeg() * 3600.0, nwCorner.getLatitudeDeg() * 3600.0 );
    elv3 = westArray->altitude_from_grid( nwCorner.getLongitudeDeg() * 3600.0, nwCorner.getLatitudeDeg() * 3600.0 ); 
    elv4 = tileArray->altitude_from_grid( nwCorner.getLongitudeDeg() * 3600.0, nwCorner.getLatitudeDeg() * 3600.0 );

    SG_LOG(SG_GENERAL, SG_ALERT, "4 elevations calculated: " << elv1 << ", " << elv2 << ", " << elv3 << ", " << elv4 << ", "  );

    // then the shared edges


    // then the interior
#endif

    // now we can calc the face normals
}

void tgMesh::loadStage2( const std::string& basePath, const SGBucket& bucket )
{
    // load the stage2 triangulation as a mesh
    std::string bucketPath = basePath + "/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
    b = bucket;

    // now load the stage1 triangulation ( and lookup locations on the edges )
    meshSurface.loadTriangulation( basePath, bucket );
}


void tgMesh::calcFaceNormals( void )
{

}


void tgMesh::save( const std::string& path ) const
{
    meshArrangement.toShapefile( path, "stage1_arrangement" );
    meshTriangulation.saveSharedEdgeNodes( path );
    meshTriangulation.saveTds( path );
}

void tgMesh::save2( const std::string& path ) const
{
    meshTriangulation.saveTds( path );

    // generate edge node list
    // meshTriangulation.saveSharedEdgeFaces( path );
}