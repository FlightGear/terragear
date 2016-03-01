#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

//#include <CGAL/Triangulation_conformer_2.h>

#define DEBUG_MESH_TRIANGULATION            (0)     // Generate intermediate shapefiles during triangulation and refinement
#define DEBUG_MESH_TRIANGULATION_DATAFILE   (0)     // generate text file readable by cgal_tri_test - for generating CGAL bug reports
#define DEBUG_MESH_TRIANGULATION_DATAFILE_2 (0)     // alternative file for saving the cdt mesh natively - for generating CGAL bug reports

void tgMesh::writeCdtFile( const char* filename, std::vector<meshTriPoint>& points,  std::vector<meshTriSegment>& constraints ) const
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

void tgMesh::writeCdtFile2( const char* filename, const meshTriCDTPlus& cdt) const
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

void tgMesh::constrainedTriangulateWithEdgeModification( void )
{
    // use int indicies to aid in debugging - we may not want to insert them all    
    meshArrEdgeConstIterator    eit;
    meshArrVertexConstIterator  vit;

    std::vector<meshTriPoint>   points;
    std::vector<meshTriSegment> constraints; 
    
    // generate a triangulation from the arrangement.
    // insert all segments as constraints
    // and all elevations as points    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - insert points " );
    for ( vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); vit++ ) {
        if ( vit->is_isolated() ) {
            points.push_back( meshTriPoint( CGAL::to_double(vit->point().x()), CGAL::to_double(vit->point().y()) ) );
        }
    }
#if DEBUG_MESH_TRIANGULATION    
    toShapefile( datasource, "elevation_points", points );
#endif
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - insert constraints " );    
    for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {
        meshTriPoint source = toMeshTriPoint( eit->curve().source() );
        meshTriPoint target = toMeshTriPoint( eit->curve().target() );
        
        if ( source != target ) {
            constraints.push_back( meshTriSegment(source, target ) );
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "meshTriangulation : found segment with source == target" );
        }
    }    
    
#if DEBUG_MESH_TRIANGULATION_DATAFILE
    writeCdtFile( "./output_cdt.txt", points, constraints );
#endif

    // insert the points, then the constraints
    meshTriangulation.insert_constraints( constraints.begin(), constraints.end() );
    meshTriangulation.insert(points.begin(), points.end());
    
    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - valid? " << 
                                 (meshTriangulation.is_valid() ? "yes, and has " : "no, and has ") << 
                                  meshTriangulation.number_of_faces() << " faces ");

#if DEBUG_MESH_TRIANGULATION    
    toShapefile( datasource, "pre_refined_triangulation", meshTriangulation, false );
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
        // markDomains();
        
#if DEBUG_MESH_TRIANGULATION    
        toShapefile( datasource, "refined_triangulation", meshTriangulation, true );
#endif        
    }
}

#if 0
// given a mesh face - mark all triangle faces within the constrained boundaries with the face handle from the arrangement
void tgMesh::markDomains(meshTriFaceHandle start, meshArrFaceConstHandle face, std::list<meshTriEdge>& border )
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
void tgMesh::clearDomains(void)
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
void tgMesh::markDomains(void)
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
            face = findMeshFace( CGAL::centroid(tri) );
            markDomains(n, face, border);
        }
    }
}
#endif