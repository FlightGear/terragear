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
    
    // Step 2 - insert clipped polys into an arrangement.  from now on, we will
    // use faces of the arrangement to identify polygon info
    arrangePolys();
    
//    toShapefile( datasource, "arr_raw", meshArr );

//    constrainedTriangulate( "tri_raw" );
//    meshTriangulation.clear();
    
    // step 2 - clean up the polys by clustering the nodes ( use arrangement )
    cleanPolys();

    // step 3 - add the polys to an arrangement to handle colinear nodes
    //          for each non hole face in resulting poly, add an internal ref point
    //          Then add each face to an arrangment.  lookup the face via ref point
    //          link original poly meta data to the arrangement face handle
    //
//    toShapefile( datasource, "arr_clean", meshArr );
    
    // step 4 - create constrained triangulation with arrangement edges as the constraints
    //
    constrainedTriangulate( "tri_clean" );
    
    // step 5 - meshify the triangulation to produce good triangles
    // 
    // step 6 - for each triangle, calc centroid, and lookup meta data
    //          create triangle - metadata linkage
    // step 7 - test the mesh    
}