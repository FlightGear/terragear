#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

#define DEBUG_MESH_TRIANGULATION            (0)     // Generate intermediate shapefiles during triangulation and refinement
#define DEBUG_MESH_TRIANGULATION_DATAFILE   (0)     // generate text file readable by cgal_tri_test - for generating CGAL bug reports
#define DEBUG_MESH_TRIANGULATION_DATAFILE_2 (0)     // alternative file for saving the cdt mesh natively - for generating CGAL bug reports

void tgMeshTriangulation::writeCdtFile( const char* filename, std::vector<meshTriPoint>& points,  std::vector<meshTriSegment>& constraints ) const
{
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
    } else {
        output_file << std::setprecision(64);
        
        output_file << points.size() << "\n";
        for ( unsigned int i = 0; i < points.size(); i++ ) {
            output_file << points[i] << "\n";
        }
    }
    output_file << "\n";
    
    output_file << constraints.size() << "\n";
    for ( unsigned int i = 0; i < constraints.size(); i++ ) {        
        output_file << constraints[i].source() << "\t" << constraints[i].target() << "\n";            
    }
    output_file.close();
}

void tgMeshTriangulation::writeCdtFile2( const char* filename, const meshTriCDTPlus& cdt) const
{
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        exit(0);
    }
    
    output_file << std::setprecision(64);
    output_file << cdt;
    output_file.close();    
}

void tgMeshTriangulation::saveAscii( const std::string& datasource, const char* file ) const
{
    std::string filename = datasource + "/" + file;
    std::ofstream output_file(filename.c_str());
    
    CGAL::set_ascii_mode(output_file);
    output_file << std::setprecision(64);
    output_file << meshTriangulation;
    output_file.close();
}

void tgMeshTriangulation::constrainedTriangulateWithEdgeModification( const tgMeshArrangement& arr )
{
    std::vector<meshTriPoint>   points;
    std::vector<meshTriSegment> constraints; 
    
    // generate a triangulation from the arrangement.
    // insert all segments as constraints
    // and all elevations as points    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - insert points " );
    arr.getPoints( points );

#if DEBUG_MESH_TRIANGULATION    
    toShapefile( mesh->getDebugPath(), "stage1_elevation_points", points );
#endif
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - insert constraints " );    
    arr.getSegments( constraints );
    
#if DEBUG_MESH_TRIANGULATION_DATAFILE
    writeCdtFile( "./output_cdt.txt", points, constraints );
#endif

    // insert the points, then the constraints
    meshTriangulation.insert_constraints( constraints.begin(), constraints.end() );
    meshTriangulation.insert( points.begin(), points.end() );
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - valid? " << 
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
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - create mesher" );
        meshRefinerWithEdgeModification mesher(meshTriangulation);
        
        // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
        // 0.5 is the upper bound on the length of the longuest edge.
        // See reference manual for Delaunay_mesh_size_traits_2<K>.        
        // mesher.set_criteria(meshCriteria(0.125, 0.5));
        //SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - set criteria" );
        mesher.set_criteria(meshCriteria(0.1, 0.5));
        
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - refine mesh" );
        mesher.refine_mesh();

        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - refined mesh number of faces: " << meshTriangulation.number_of_faces() );
        
        // set arrangement face info for looking up metadata of original polygons
        markDomains( arr );
        
#if DEBUG_MESH_TRIANGULATION    
        toShapefile( mesh->getDebugPath(), "stage1_refined_triangulation", true );
#endif        
    }
}

void tgMeshTriangulation::constrainedTriangulateWithoutEdgeModification( const std::vector<meshTriPoint>& points, const std::vector<meshTriSegment>& constraints )
{
    meshTriangulation.insert_constraints( constraints.begin(), constraints.end() );
    meshTriangulation.insert(points.begin(), points.end());
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - valid? " << 
        (meshTriangulation.is_valid() ? "yes, and has " : "no, and has ") << 
        meshTriangulation.number_of_faces() << " faces ");
    
#if DEBUG_MESH_TRIANGULATION    
    toShapefile( mesh->getDebugPath(), "stage2_pre_refined_triangulation", false );
#endif
    
    if ( meshTriangulation.is_valid() ) {        
#if DEBUG_MESH_TRIANGULATION_DATAFILE_2 
        writeCdtFile2( "./output_cdt2.txt", meshTriangulation );
#endif        
        
        // create a mesh from the triangulation
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - create mesher" );
        meshRefinerWithoutEdgeModification mesher(meshTriangulation);
        
        // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
        // 0.5 is the upper bound on the length of the longuest edge.
        // See reference manual for Delaunay_mesh_size_traits_2<K>.        
        // mesher.set_criteria(meshCriteria(0.125, 0.5));
        //SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - set criteria" );
        mesher.set_criteria(meshCriteria(0.1, 0.5));
        
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - refine mesh" );
        mesher.refine_mesh();
        
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - refined mesh number of faces: " << meshTriangulation.number_of_faces() );
        
#if DEBUG_MESH_TRIANGULATION    
        toShapefile( mesh->getDebugPath(), "stage2_refined_triangulation", true );        
#endif  
    }
}

#if 1
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
    for(meshTriCDTPlus::All_faces_iterator it = meshTriangulation.all_faces_begin(); it != meshTriangulation.all_faces_end(); ++it) {
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
#endif

void tgMeshTriangulation::calcTileElevations( const tgArray* tileArray )
{
    // set point info for the 2d triangulation
    SG_LOG(SG_GENERAL, SG_INFO, "Current elevations " );
    
    for (meshTriCDTPlus::Finite_vertices_iterator vit = meshTriangulation.finite_vertices_begin(); vit != meshTriangulation.finite_vertices_end(); vit++ ) {
        vit->info().setElevation( tileArray->altitude_from_grid( vit->point().x() * 3600.0, vit->point().y() * 3600.0 ) );
        SG_LOG(SG_GENERAL, SG_INFO, vit->info().getElevation() );
    }
}

void tgMeshTriangulation::prepareTds( void )
{
    // SG_LOG(SG_GENERAL, SG_INFO, "Prepare TDS for saving" );
    // save tds just like cgal - skip first = true, need infinite vertex.
    meshTriTDS tds = meshTriangulation.tds();
    
    //meshTriVertexHandle iv = meshTriangulation.infinite_vertex();
    //meshTriTDS::Vertex_handle iv = meshTriangulation.infinite_vertex();

    CGAL::Unique_hash_map<meshTriTDS::Vertex_handle, int>   V;
    CGAL::Unique_hash_map<meshTriTDS::Face_handle, int>     F;
    
    // vertices
    int inum = 0;
    vertexInfo.clear();
    for( meshTriTDS::Vertex_iterator vit= tds.vertices_begin(); vit != tds.vertices_end() ; ++vit) {
        if ( vit != meshTriTDS::Vertex_handle() ) {
            vertexInfo.push_back(meshVertexInfo(inum, vit));
            V[vit] = inum++;
        }
    }
    
    inum = 0;
    faceInfo.clear();
    for( meshTriTDS::Face_iterator ib = tds.face_iterator_base_begin(); ib != tds.face_iterator_base_end(); ++ib) {
        faceInfo.push_back(meshFaceInfo(inum, ib, V));
        F[ib] = inum++;
    }
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMeshTriangulation::prepareTds: have " << vertexInfo.size() << " vertices and " << faceInfo.size() << " faces." );
    
    // neighbor pointers of the  faces
    inum = 0;
    for( unsigned int i=0; i<faceInfo.size(); i++ ) {
        faceInfo[i].setNeighbors( F );        
   }
}

void tgMeshTriangulation::saveTds( const std::string& datasource, const char* layer ) const
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer = NULL;
    OGRLayer*     poFaceLayer = NULL;
    char          layerName[64];
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - saveTDS - begin" );

    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        sprintf( layerName, "%s_points", layer );
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_TDS_VERTEX, layerName );
        
        sprintf( layerName, "%s_faces", layer );
        poFaceLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_TDS_FACE, layerName );
    
        if ( poPointLayer ) {
            for (unsigned int i=0; i<vertexInfo.size(); i++) {
                toShapefile( poPointLayer, vertexInfo[i] );
            }
        }
    
        if ( poFaceLayer ) {
            for (unsigned int i=0; i<faceInfo.size(); i++) {
                toShapefile( poFaceLayer, faceInfo[i] );
            }
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - saveTDS - Error openeing face layer" );            
        }

        // close datasource
        GDALClose( poDS );
    }
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - saveTDS - end" );
}