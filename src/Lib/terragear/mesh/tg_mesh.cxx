#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

void tgMesh::initPriorities( const std::vector<std::string>& names )
{
    priorityNames = names;
    numPriorities = names.size();
   
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        tgPolygonSetList lc;
        sourcePolys.push_back( lc );
    }
    
    clipBucket = false;
}

void tgMesh::initDebug( const std::string& dbgRoot )
{
    datasource = dbgRoot;
}

void tgMesh::clipAgainstBucket( const SGBucket& bucket ) 
{
    clipBucket = true;
    b = bucket;
}

void tgMesh::clear( void )
{
    // clear source polys
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        sourcePolys.clear();
    }
    sourcePoints.clear();
    
    meshPointLocation.detach();
    meshArr.clear();
    meshTriangulation.clear();
    metaLookup.clear();    
}

bool tgMesh::empty( void )
{
    bool empty = true;
    
    // check source polys
    for ( unsigned int i=0; empty && i<numPriorities; i++ ) {
        empty = sourcePolys.empty();
    }
    
    return empty;
}

void tgMesh::addPoly( unsigned int priority, const tgPolygonSet& poly )
{
    sourcePolys[priority].push_back( poly );
}

void tgMesh::addPolys( unsigned int priority, const tgPolygonSetList& polys )
{
    sourcePolys[priority].insert( sourcePolys[priority].end(), polys.begin(), polys.end() );
}

void tgMesh::addPoints( const std::vector<meshTriPoint>& points )
{
    sourcePoints.insert( sourcePoints.end(), points.begin(), points.end() );    
}

tgPolygonSet tgMesh::join( unsigned int priority, const tgPolygonSetMeta& meta )
{
    return tgPolygonSet::join( sourcePolys[priority], meta );    
}

void tgMesh::generate( void )
{    
    bool havePolys = false;

    for ( unsigned int i=0; i<numPriorities && !havePolys; i++ ) {
        std::vector<tgPolygonSet>::iterator poly_it;
        if ( !sourcePolys[i].empty() ) {
            havePolys = true;
        }
    }
    
    // mesh generation from polygon soup :)
    if ( havePolys ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "have source polys" );

        // Step 1 - clip polys against one another - highest priority first ( on top )
        clipPolys();
    
        // Step 2 - insert clipped polys into an arrangement.  
        // From this point on, we don't need the individual polygons.
        arrangePolys();
    
        // step 3 - clean up the arrangement - cluster nodes that are too close - don't want
        // really small triangles blowing up the refined mesh.
        // NOTE / TODO: 
        // The cluster size MUST be smaller than the minimum distance of interiorPoints.
        // we should remember be checking the delta in interiorPoints to see if we have 
        // polys that don't meat this criteria.
        // and if it doesn't - what do we do?
        cleanArrangement();
    
        // step 4 - create constrained triangulation with arrangement edges as the constraints
        constrainedTriangulate();
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "no source polys" );        
    }
}

void tgMesh::save( const std::string& path )
{
    toShapefile( path, "stage1_arrangement", meshArr );
    toShapefile( path, "stage1_triangles", meshTriangulation, true );
}