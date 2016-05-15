#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

#define DEBUG_MESH_TRIANGULATION            (0)     // Generate intermediate shapefiles during triangulation and refinement
#define DEBUG_MESH_TRIANGULATION_DATAFILE   (0)     // generate text file readable by cgal_tri_test - for generating CGAL bug reports
#define DEBUG_MESH_TRIANGULATION_DATAFILE_2 (0)     // alternative file for saving the cdt mesh natively - for generating CGAL bug reports

#define TRACE_MESH_TRIANGULATION            SG_DEBUG

void tgMeshTriangulation::constrainedTriangulateWithEdgeModification( const tgMeshArrangement& arr )
{
    std::vector<meshTriPoint>   points;
    std::vector<meshTriSegment> constraints; 

    // generate a triangulation from the arrangement.
    // insert all segments as constraints
    // and all elevations as points    
    SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - insert points " );
    arr.getPoints( points );

#if DEBUG_MESH_TRIANGULATION    
    toShapefile( mesh->getDebugPath(), "stage1_elevation_points", points );
#endif

    SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - insert constraints " );    
    arr.getSegments( constraints );

#if DEBUG_MESH_TRIANGULATION_DATAFILE
    writeCdtFile( "./output_cdt.txt", points, constraints );
#endif

    // insert the points, then the constraints
    meshTriangulation.insert_constraints( constraints.begin(), constraints.end() );
    meshTriangulation.insert( points.begin(), points.end() );

    SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - valid? " << 
                                 (meshTriangulation.is_valid() ? "yes, and has " : "no, and has ") << 
                                  meshTriangulation.number_of_faces() << " faces ");

#if DEBUG_MESH_TRIANGULATION    
    toShapefile( mesh->getDebugPath(), "stage1_pre_refined_triangulation", false );
#endif

    if ( meshTriangulation.is_valid() ) {        
#if DEBUG_MESH_TRIANGULATION_DATAFILE_2 
        writeCdtFile2( "./output_cdt2.txt", meshTriangulation );
#endif

        // create a mesh from the triangulation
        SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - create mesher" );
        meshRefinerWithEdgeModification mesher(meshTriangulation);

        // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
        // 0.5 is the upper bound on the length of the longuest edge.
        // See reference manual for Delaunay_mesh_size_traits_2<K>.        
        // mesher.set_criteria(meshCriteria(0.125, 0.5));
        //SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - set criteria" );
        mesher.set_criteria(meshCriteria(0.1, 0.5));

        SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - refine mesh" );
        mesher.refine_mesh();

        SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - refined mesh number of faces: " << meshTriangulation.number_of_faces() );

        // set arrangement face info for looking up metadata of original polygons
        markDomains( arr );

#if DEBUG_MESH_TRIANGULATION    
        toShapefile( mesh->getDebugPath(), "stage1_refined_triangulation", true );
#endif
    }
}

void tgMeshTriangulation::constrainedTriangulateWithoutEdgeModification( const std::vector<movedNode>& movedPoints, const std::vector<meshTriPoint>& addedPoints )
{
    SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate before adding new nodes - valid? " << 
        (meshTriangulation.is_valid() ? "yes, and has " : "no, and has ") << meshTriangulation.number_of_faces() << " faces ");

    // first, let's move each node ( remove - and add back.  - this invalidates all the mappings
    // if a vertex is incident to a constrained edge, we need to remove the constraint, then re-add once new vertex is added.
    for( unsigned int i=0; i<movedPoints.size(); i++ )
    {
        // first, gather the constrained edges.  in debug mode, CGAL will assert if any incident edges are constrained
        std::vector<meshTriEdge>            constrainedEdges;
        std::vector<meshTriVertexHandle>    constrainedEndpoints;

        meshTriVertexHandle target = movedPoints[i].oldPositionHandle;
        meshTriangulation.incident_constraints( target, std::back_inserter( constrainedEdges ) );

        // save vertex handle to the other end of each constraint.
        for ( unsigned int j=0; j<constrainedEdges.size(); j++ ) {
            meshTriFaceHandle   face  = constrainedEdges[j].first;
            int                 index = constrainedEdges[j].second;

            constrainedEndpoints.push_back( face->vertex( face->ccw(index) ) );
            meshTriangulation.remove_constrained_edge( face, index );
        }

        SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate node to remove has " << constrainedEdges.size() << " constrained edges." ); 

        meshTriangulation.remove( movedPoints[i].oldPositionHandle );
        target = meshTriangulation.insert( movedPoints[i].newPosition );

        for ( unsigned int j=0; j<constrainedEndpoints.size(); j++ ) {
            meshTriangulation.insert_constraint( constrainedEndpoints[j], target );
        }
    }

    // now add new nodes
    SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate adding " << addedPoints.size() << " nodes" );
    meshTriangulation.insert( addedPoints.begin(), addedPoints.end() );

    SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate after adding new nodes - valid? " << 
        (meshTriangulation.is_valid() ? "yes, and has " : "no, and has ") << meshTriangulation.number_of_faces() << " faces ");

#if DEBUG_MESH_TRIANGULATION    
    toShapefile( mesh->getDebugPath(), "stage2_pre_refined_triangulation", false );
#endif

    if ( meshTriangulation.is_valid() ) {
#if DEBUG_MESH_TRIANGULATION_DATAFILE_2 
        writeCdtFile2( "./output_cdt2.txt", meshTriangulation );
#endif

        // create a mesh from the triangulation
        SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - create mesher" );
        meshRefinerWithoutEdgeModification mesher(meshTriangulation);

        // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
        // 0.5 is the upper bound on the length of the longuest edge.
        // See reference manual for Delaunay_mesh_size_traits_2<K>.        
        // mesher.set_criteria(meshCriteria(0.125, 0.5));
        mesher.set_criteria(meshCriteria(0.1, 0.5));

        SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - refine mesh" );
        mesher.refine_mesh();

        SG_LOG( SG_GENERAL, TRACE_MESH_TRIANGULATION, "tgMesh::constrainedTriangulate - refined mesh number of faces: " << meshTriangulation.number_of_faces() );

#if DEBUG_MESH_TRIANGULATION    
        toShapefile( mesh->getDebugPath(), "stage2_refined_triangulation", true );        
#endif
    }
}

// given a mesh face - mark all triangle faces within the constrained boundaries with the face handle from the arrangement
void tgMeshTriangulation::markDomains(meshTriFaceHandle start, meshArrFaceConstHandle face, std::list<meshTriEdge>& border )
{
    if( start->info().isVisited() ) {
        return;
    }

    std::list<meshTriFaceHandle> queue;
    queue.push_back(start);

    while( !queue.empty() ){

        // grab a new face
        meshTriFaceHandle fh = queue.front();
        queue.pop_front();

        if( !fh->info().isVisited() ) {
            // if it hasn't been handled yet
            // set it's face information
            fh->info().setFace( face );

            // then check all three of the triangles edges
            for(int i = 0; i < 3; i++) {

                // get the edge, and the face on the other side of it.
                meshTriEdge e(fh,i);
                meshTriFaceHandle n = fh->neighbor(i);

                // if the neighbor face has not been visited, we need 
                // to do 1 of two things.  add it to the queue for the same face,
                // or push the constrained edge to the list of future edges to check
                if( !n->info().isVisited() ) {
                    if(meshTriangulation.is_constrained(e)) {
                        // this edge is a constraint.
                        // push it into the todo list of different faces
                        border.push_back(e);
                    } else {
                        // edge was not a constraint.
                        // The face on the other side of this 
                        // edge has the same metadata.
                        queue.push_back(n);
                    }
                }
            }
        }
    }
}

// clear all face data ( so we can retriangulate during debug, etc )
void tgMeshTriangulation::clearDomains(void)
{
    for(meshTriCDT::All_faces_iterator it = meshTriangulation.all_faces_begin(); it != meshTriangulation.all_faces_end(); ++it) {
        it->info().clear();
    }
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void tgMeshTriangulation::markDomains( const tgMeshArrangement& arr )
{
    clearDomains();

    meshArrFaceConstHandle face = (meshArrFaceConstHandle)NULL;
    std::list<meshTriEdge> border;

    // first, mark all triangles on the arrangement infinite face with NULL.
    // this tells us that these triangles are junk, and will not be part of the final 
    // mesh
    markDomains(meshTriangulation.infinite_face(), face, border);

    // all edges that were constrained on the last boundary are then checked
    while( !border.empty() ) {
        // grab an edge
        meshTriEdge e = border.front();
        border.pop_front();

        // get the face on the 'other' side of this edge
        // edge is a tuple.
        // - first is the face on 'this' side of the edge
        // - second is the edge index of this edge in the face.
        // NOTE: this means each edge is referenced twice - once from each face
        meshTriFaceHandle n = e.first->neighbor(e.second);
        if( !n->info().isVisited() ) {
            meshTriangle tri = meshTriangulation.triangle( n );

            // get face handle for point inside this facet
            face = arr.findMeshFace( CGAL::centroid(tri) );
            markDomains(n, face, border);
        }
    }
}

void tgMeshTriangulation::calcTileElevations( const tgArray* tileArray )
{
    // set point info for the 2d triangulation
    SG_LOG(SG_GENERAL, SG_INFO, "Current elevations " );

    for (meshTriCDT::Finite_vertices_iterator vit = meshTriangulation.finite_vertices_begin(); vit != meshTriangulation.finite_vertices_end(); vit++ ) {
        vit->info().setElevation( tileArray->altitude_from_grid( vit->point().x() * 3600.0, vit->point().y() * 3600.0 ) );
        SG_LOG(SG_GENERAL, SG_INFO, vit->info().getElevation() );
    }
}

void tgMeshTriangulation::prepareTds( void )
{
    // save tds just like cgal - skip first = true, need infinite vertex.
    const meshTriTDS& tds = meshTriangulation.tds();
    int               inum;

    // saveConstrained("constrained1");

    // vertices
    vertexInfo.clear();
    vertexHandleToIndexMap.clear();

    inum = 0;
    for( meshTriTDS::Vertex_iterator vit = tds.vertices_begin(); vit != tds.vertices_end(); vit++, inum++) {
        if ( vit != meshTriTDS::Vertex_handle() ) {
            vertexHandleToIndexMap[vit] = inum;
            vertexInfo.push_back(meshVertexInfo(inum, vit));
        }
    }

    faceInfo.clear();
    faceHandleToIndexMap.clear();

    inum = 0;
    for( meshTriTDS::Face_iterator fit = tds.face_iterator_base_begin(); fit != tds.face_iterator_base_end(); fit++, inum++) {
        faceHandleToIndexMap[fit] = inum;
        faceInfo.push_back(meshFaceInfo(inum, fit, vertexHandleToIndexMap));
    }

    // neighbor pointers of the  faces
    for( unsigned int i=0; i<faceInfo.size(); i++ ) {
        faceInfo[i].setNeighbors( faceHandleToIndexMap );
   }

    SG_LOG( SG_GENERAL, SG_DEBUG, "tgMeshTriangulation::prepareTds: have " << vertexInfo.size() << " vertices and " << faceInfo.size() << " faces." );
}