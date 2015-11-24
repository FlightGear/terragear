#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

void tgMesh::init( unsigned int pris, const std::vector<std::string>& names, const std::string& dbgRoot )
{
    numPriorities = pris;
    priorityNames = names;
    snprintf(datasource, sizeof(datasource), "./%s", dbgRoot.c_str() );
   
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        tgPolygonSetList lc;
        sourcePolys.push_back( lc );
    }
}

void tgMesh::addPoly( unsigned int priority, const tgPolygonSet& poly )
{
    sourcePolys[priority].push_back( poly );
}

void tgMesh::addPolys( unsigned int priority, const tgPolygonSetList& polys )
{
    sourcePolys[priority].insert( sourcePolys[priority].end(), polys.begin(), polys.end() );
}

tgPolygonSet tgMesh::join( unsigned int priority, const tgPolygonSetMeta& meta )
{
    return tgPolygonSet::join( sourcePolys[priority], meta );    
}

void tgMesh::generate( void )
{    
    // mesh generation from polygon soup :)

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
}