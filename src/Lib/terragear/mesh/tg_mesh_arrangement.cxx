#include <simgear/debug/logstream.hxx>

#include <terragear/polygon_set/tg_polygon_accumulator.hxx>
#include <terragear/tg_cluster.hxx>

#include "tg_mesh.hxx"

void tgMesh::clipPolys( void )
{
    tgAccumulator accum;
    
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator poly_it;
        for ( poly_it = sourcePolys[i].begin(); poly_it != sourcePolys[i].end(); poly_it++ ) {
            tgPolygonSet current = (*poly_it);

            accum.Diff_and_Add_cgal( current );
            poly_it->setPs( current.getPs() );
        }
    }    
}


static unsigned int poly_num = 1;

// debug : before and after polys - seperate layers - ugh... - number them
void tgMesh::cleanPolys( void )
{
    char layer[256];
        
    // we'll use lloyd voronoi relaxation to cluster and remove nodes too close to one another
    // create the point list
    meshArrVertexConstIterator vit;
    std::list<cgalPoly_Point>  nodes;
    
    for ( vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); vit++ ) {
        nodes.push_back( vit->point() );
    }
    
#if 0    
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator poly_it;
        for ( poly_it = sourcePolys[i].begin(); poly_it != sourcePolys[i].end(); poly_it++ ) {
            tgPolygonSet current = (*poly_it);
            
            std::list<cgalPoly_PolygonWithHoles>                 pwh_list;
            std::list<cgalPoly_PolygonWithHoles>::const_iterator it;
            cgalPoly_PolygonWithHoles::Hole_const_iterator       hit;

            current.getPs().polygons_with_holes( std::back_inserter(pwh_list) );
            SG_LOG(SG_GENERAL, SG_DEBUG, "tgMesh::cleanPolys: got " << pwh_list.size() << " polys with holes ");
                
            // save each poly with holes to the layer
            for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
                cgalPoly_PolygonWithHoles pwh = (*it);
                cgalPoly_Polygon poly = pwh.outer_boundary();

                cgalPoly_Polygon::Vertex_iterator vit;
                for ( vit = poly.vertices_begin(); vit != poly.vertices_end(); vit++ ) {
                    nodes.push_back( *vit );
                }
                
                for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
                    poly = *hit;
                    for ( vit = poly.vertices_begin(); vit != poly.vertices_end(); vit++ ) {
                        nodes.push_back( *vit );
                    }
                }
            }
        }
    }
#endif

    tgCluster cluster( nodes, 0.0000050 );
  //tgCluster cluster( nodes, 0.0000025 );
  //tgCluster cluster( nodes, 0.0000010 );
    cluster.toShapefile( datasource, "cluster" );
    

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

    meshArr.clear();
    CGAL::insert( meshArr, segs.begin(), segs.end() );
    meshPointLocation.attach( meshArr );
    
    // traverse the polys, and get their faces again
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator pit;
        for ( pit = sourcePolys[i].begin(); pit != sourcePolys[i].end(); pit++ ) {
            if ( !pit->isEmpty() ) {
                const std::vector<cgalPoly_Point>& queryPoints = pit->getInteriorPoints();
                for ( unsigned int i=0; i<queryPoints.size(); i++ ) {
                    CGAL::Object obj = meshPointLocation.locate(queryPoints[i]);
                
                    meshArrangement::Face_const_handle      f;
                    meshArrangement::Halfedge_const_handle  e;
                    meshArrangement::Vertex_const_handle    v;
        
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

    
    

    
#if 0    
    // traverse all polys again - and refine their shapes...
    // TODO: polygonset construction generalization
    // we want to use the same tecniques we do when reading
    // untrusted shapefiles...
    // maybe for each rig, just add points to a vector, then use the vector
    for ( unsigned int i=0; i<numPriorities; i++ ) {
        std::vector<tgPolygonSet>::iterator poly_it;
        for ( poly_it = sourcePolys[i].begin(); poly_it != sourcePolys[i].end(); poly_it++ ) {
            snprintf( layer, sizeof(layer), "%05d_before", poly_num );
            poly_it->toShapefile( datasource, layer );

            poly_it->clusterNodes( cluster );
                        
            snprintf( layer, sizeof(layer), "%05d_after", poly_num );
            poly_it->toShapefile( datasource, layer );            

            poly_num++;
        }
    }
#endif

}

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
    std::vector<cgalPoly_Segment> segs;
    pit->toSegments( segs, false );
    CGAL::insert( meshArr, segs.begin(), segs.end() );
    
    // don't do this yet - wait until after we clean the arrangement    
#if 0
    // then query and link the new faces back to the object
    meshArrLandmarks_pl landmarks_pl;
    landmarks_pl.attach(meshArr);
    
    const std::vector<cgalPoly_Point>& queryPoints = pit->getInteriorPoints();
    for ( unsigned int i=0; i<queryPoints.size(); i++ ) {
        CGAL::Object obj = landmarks_pl.locate(queryPoints[i]);
        meshArrangement::Face_const_handle      f;
        meshArrangement::Halfedge_const_handle  e;
        meshArrangement::Vertex_const_handle    v;
        
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
#endif    
}