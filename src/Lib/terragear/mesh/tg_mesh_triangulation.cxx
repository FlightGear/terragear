#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

void tgMesh::constrainedTriangulate( const char* dbglayer_prefix )
{
    char layer[256];
    
    // generate a triangulation from the arrangement.
    // just insert all segments as constraints
    meshArrEdgeConstIterator    eit;

    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - insert constraints " );
    
    for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {        
        meshTriPoint source = toMeshTriPoint( eit->curve().source() );
        meshTriPoint target = toMeshTriPoint( eit->curve().target() );
        
        if ( source != target ) {
            meshTriangulation.insert_constraint( source, target );
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "meshTriangulation : found segment with source == target" );
        }
    }

    SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - valid? " << 
                                 (meshTriangulation.is_valid() ? "yes, and has " : "no, and has ") << 
                                 meshTriangulation.number_of_faces() << " faces ");

//    clearDomains();
//    snprintf( layer, sizeof(layer), "%s_deluany_unmarked", dbglayer_prefix );
//    toShapefile( datasource, layer, meshTriangulation, false ); 

//    markDomains();
//    snprintf( layer, sizeof(layer), "%s_deluany_marked", dbglayer_prefix );
//    toShapefile( datasource, layer, meshTriangulation, true );
    
    if ( meshTriangulation.is_valid() ) {

        // create a mesh from the triangulation
        meshRefiner mesher(meshTriangulation);
        
        // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
        // 0.5 is the upper bound on the length of the longuest edge.
        // See reference manual for Delaunay_mesh_size_traits_2<K>.
        //mesher.set_criteria(meshCriteria(0.125, 0.5));
        mesher.set_criteria(meshCriteria(0.125, 0.5));
        mesher.refine_mesh();
        SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - refined mesh number of faces: " << meshTriangulation.number_of_faces() );

        clearDomains();
//        snprintf( layer, sizeof(layer), "%s_refined_unmarked", dbglayer_prefix );
//        toShapefile( datasource, layer, meshTriangulation, false ); 

        markDomains();
        snprintf( layer, sizeof(layer), "%s_refined_marked", dbglayer_prefix );
        toShapefile( datasource, layer, meshTriangulation, true );
    }
}

void tgMesh::markDomains(meshCDTPlus::Face_handle start, meshArrangement::Face_const_handle face, std::list<meshCDTPlus::Edge>& border )
{    
    if( start->info().isVisited() ) {
        return;
    }

    std::list<meshCDTPlus::Face_handle> queue;
    queue.push_back(start);
    while( !queue.empty() ){
        meshCDTPlus::Face_handle fh = queue.front();
        queue.pop_front();
        if( !fh->info().isVisited() ) {
            fh->info().setFace( face );
            for(int i = 0; i < 3; i++) {
                meshCDTPlus::Edge e(fh,i);
                meshCDTPlus::Face_handle n = fh->neighbor(i);
                if( !n->info().isVisited() ) {
                    if(meshTriangulation.is_constrained(e)) {
                        border.push_back(e);
                    } else {
                        queue.push_back(n);
                    }
                }
            }
        }
    }
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void tgMesh::clearDomains(void)
{
    for(meshCDTPlus::All_faces_iterator it = meshTriangulation.all_faces_begin(); it != meshTriangulation.all_faces_end(); ++it) {
        it->info().clear();
    }
}

void tgMesh::markDomains(void)
{
    clearDomains();

    meshArrangement::Face_const_handle face = (meshArrangement::Face_const_handle)NULL;
    std::list<meshCDTPlus::Edge> border;

    markDomains(meshTriangulation.infinite_face(), face, border);
    
    while( !border.empty() ) {
        meshCDTPlus::Edge e = border.front();
        border.pop_front();
        meshCDTPlus::Face_handle n = e.first->neighbor(e.second);
        if( !n->info().isVisited() ) {
            meshTriangle tri = meshTriangulation.triangle( n );
            
            // get face handle for point inside this facet
            face = findMeshFace( CGAL::centroid(tri) );            
            markDomains(n, face, border);            
        }        
    }
}
