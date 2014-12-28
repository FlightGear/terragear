#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <CGAL/assertions.h>
#include <CGAL/squared_distance_2.h>

#include <simgear/math/SGMath.hxx>
#include <simgear/debug/logstream.hxx>

#include "tg_euclidean.hxx"
#include "tg_segmentnetwork.hxx"
#include "tg_cluster.hxx"
#include "tg_shapefile.hxx"
#include "tg_cgal.hxx"

#define LOG_STAGES              SG_DEBUG
#define LOG_CLUSTERS            SG_DEBUG
#define LOG_FINGER_REMOVAL      SG_DEBUG
#define LOG_FINGER_EXTENSION    SG_DEBUG
#define LOG_SHORT_EDGES         SG_DEBUG
#define LOG_FIX_SHORT_SEGMENT   SG_DEBUG

tgSegmentNetwork::tgSegmentNetwork( const std::string debugRoot ) : invalid_vh()
{
    sprintf( datasource, "./edge_dbg/%s", debugRoot.c_str() );
}

void tgSegmentNetwork::Add( const SGGeod& source, const SGGeod& target, double width, unsigned int type )
{
#if DEBUG_STAGES    
    static int input_count = 1;
    char feat[16];    
    
    tgSegment input(s, e);
    sprintf( feat, "input_%05d", input_count++ );
    tgShapefile::FromSegment( input, false, datasource, "input", feat );
#endif
    
    segnetPoint snSource( source.getLongitudeDeg(), source.getLatitudeDeg() );
    segnetPoint snTarget( target.getLongitudeDeg(), target.getLatitudeDeg() );

    segnetCurve curve(snSource, snTarget);
    CurveData   data( width, type );
    
    CGAL::insert( arr, segnetCurveWithData(curve, data) );
}
            
void tgSegmentNetwork::Clean( bool clean )
{    
    ToShapefiles( "input" );
    
    if ( clean ) {
        // first, cluster the nodes
        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Cluster" );    
        Cluster();    
        ToShapefiles( "clustered" );
    
        // then remove very short fingers, or fingers that end close to their neighbors
        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::RemoveFingers" );    
        //RemoveFingers();
        ToShapefiles( "removed_fingers" );
    
        // then try to extend existing fingers to nearby lines or vertices
        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::ExtendFingers" );    
        ExtendFingers();    
        ToShapefiles( "extended_fingers" );

        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::FixShortSegments" );        
        FixShortSegments();
        ToShapefiles( "fixed_short_segments" );
    
        // really small snapround?  fix for Paris?
        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::RemoveColinearSegments" );        
        RemoveColinearSegments();
    }
    
    GenerateOutput();
}

bool tgSegmentNetwork::IsVertexHandleInList( segnetVertexHandle h, std::list<segnetVertexHandle>& vertexList )
{
    std::list<segnetVertexHandle>::iterator it;
    bool found = false;
    
    for ( it=vertexList.begin(); it != vertexList.end(); it++ ) {
        if ( (*it) == h ) {
            found = true;
            break;
        }
    }
    
    return found;
}

#if 0
// Delete any edges ( forever ) with both source and target in the given vertex list
void tgSegmentNetwork::ClusterVertex( segnetPoint newTargPoint, 
                                                  std::list<segnetVertexHandle>& vertexList, 
                                                  std::vector<segnetNonConstHalfedgeHandle>& delEdges, 
                                                  std::vector<segnetCurveWithData>& newEdges )
{
    // circulate eache vertex in the list as target, and find the edges with source in the list as well.
    std::list<segnetVertexHandle>::iterator             target_it;
    std::vector<segnetNonConstHalfedgeHandle>::iterator edge_it;

#if DEBUG_CLUSTERS
    double x, y;
    SGGeod qn;
    char desc[64];
    static int clust_id = 1;
#endif

#if DEBUG_CLUSTERS
    sprintf(desc, "clust_%d", clust_id++);
    x  = CGAL::to_double( newTargPoint.x() );
    y  = CGAL::to_double( newTargPoint.y() );
    qn = SGGeod::fromDeg( x, y );

    tgShapefile::FromGeod(qn, datasource, "clustcenter", desc );
#endif    
    
    // first, delete segments that won't survive the clustering ( both source and target are in cluster )
    for ( target_it=vertexList.begin(); target_it != vertexList.end(); target_it++ ) {
#if DEBUG_CLUSTERS
        x  = CGAL::to_double( (*target_it)->point().x() );
        y  = CGAL::to_double( (*target_it)->point().y() );
        qn = SGGeod::fromDeg( x, y );
    
        tgShapefile::FromGeod(qn, datasource, "clustnodes", desc );        
#endif                

        if ( !(*target_it)->is_isolated() ) {
            SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::ClusterVertex Checking vertex with degree " << (*target_it)->degree() );
            
            Halfedge_around_vertex_circulator first_edge = arr.non_const_handle( (*target_it)->incident_halfedges() );
            Halfedge_around_vertex_circulator cur_edge   = first_edge;

            do {
                segnetVertexHandle oldSource = cur_edge->source();
                
                if ( IsVertexHandleInList( oldSource, vertexList ) ) {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::ClusterVertex Deleting edge" );

                    // make sure we aren't adding a twin
                    segnetNonConstHalfedgeHandle twin_edge = arr.non_const_handle( cur_edge->twin() );
                    bool                         found     = false;
                    for ( edge_it = delEdges.begin(); edge_it != delEdges.end(); edge_it++ ) {
                        if ( (*edge_it == cur_edge) || (*edge_it == twin_edge) ) {
                            found = true;
                            break;
                        }
                    }
                    
                    if ( !found ) {
                        delEdges.push_back( cur_edge);
                    } else {
                        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::ClusterVertex edge already in del list" );
                    }
                
#if 1                
                    SGGeod s = SGGeod::fromDeg( CGAL::to_double( cur_edge->source()->point().x() ),
                                                CGAL::to_double( cur_edge->source()->point().y() ) );
                    SGGeod e = SGGeod::fromDeg( CGAL::to_double( cur_edge->target()->point().x() ),
                                                CGAL::to_double( cur_edge->target()->point().y() ) );
                    tgSegment delseg(s, e);
                    tgShapefile::FromSegment( delseg, true, datasource, "delete", desc );
#endif
                
                } else {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::ClusterVertex Need to modify this edge" );

                    if (cur_edge->curve().data().size() == 1) {                        
                        segnetNonConstHalfedgeHandle twin_edge = arr.non_const_handle( cur_edge->twin() );
                        bool                         found     = false;
                        for ( edge_it = delEdges.begin(); edge_it != delEdges.end(); edge_it++ ) {
                            if ( (*edge_it == cur_edge) || (*edge_it == twin_edge) ) {
                                found = true;
                                break;
                            }
                        }
                    
                        if ( !found ) {
                            segnetCurve curve( oldSource->point(), newTargPoint );
                            CurveData   data = cur_edge->curve().data().front();
                            newEdges.push_back( segnetCurveWithData( curve, data ) );
                            delEdges.push_back( cur_edge);                            
                        } else {
                            SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::ClusterVertex edge's twin already in del list" );
                        }
                    }
                }
         
                cur_edge++;
                
            } while ( cur_edge != first_edge );
        }
    }
    
    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::ClusterVertex - exit" );    
}
#endif


void tgSegmentNetwork::ModifyEdges( segnetVertexHandle oldTarget, segnetPoint newTargPoint, segnetVertexHandle ignoreSrc, 
                                    std::list<nodesPointHandle>& delNodes, 
                                    std::vector<segnetNonConstHalfedgeHandle>& delEdges, 
                                    std::vector<segnetCurveWithData>& newEdges )
{
#if 0    
    // use arrangement observer to detect when the vertex is removed                
    Halfedge_around_vertex_circulator first = arr.non_const_handle( oldTarget->incident_halfedges() );
    Halfedge_around_vertex_circulator curr  = first;

    do {
        segnetVertexHandle oldSource = curr->source();

        segnetCurve curve( oldSource->point(), newTargPoint );
        if (curr->curve().data().size() == 1) {     
            // look for the edge, (including twin ) in array
            // del old and add new if not found
            bool found = false;
            for ( unsigned int i=0; i<delEdges.size(); i++ ) {
                if ( (delEdges[i] == curr) || delEdges[i] == curr->twin() ) {
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                delEdges.push_back( arr.non_const_handle( curr ) );

                bool addBack = false;
                
                // check if this is the edge we will ignore ( not add back )
                if ( ignoreSrc != oldSource ) {
                    CurveData data = curr->curve().data().front();
                    addBack = true;
                }
                
                // check if the opposite node is in delNodes
                for ( std::list<nodesPointHandle>::const_iterator it = delNodes.begin(); it != delNodes.end(); it++ ) {
                    if ( it->point() == oldSource ) {
                        addBack = false;
                        break;
                    }
                }
                
                if ( addBack ) {
                    newEdges.push_back( segnetCurveWithData( curve, data ) );
                }
            }
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::Remove incident edge from vertex w/degree " << oldTarget->degree() << " COULDN'T GET DATA : size is " << curr->curve().data().size() );
        }
        curr++;
        
    } while ( curr != first );
#endif

}

// group nearby nodes together, and update edges to the incident edges to use the
// cluster center

// TODO: looping cluster:
// for each node, query 10 cm radius
// for each hit, recurse until result size == 1 - keep output unique

//typedef segnetKernel::Point_3   Point_3;
//typedef boost::tuple<int, Point_3, segnetVertexHandle> Point3WithHandle;

void tgSegmentNetwork::Cluster( void )
{
    // create the point list
    std::list<EPECPoint_2> nodes;
    segnetArrangement      tmp;
    
    segnetArrangement::Vertex_const_iterator vit;
    for ( vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit ) {        
        nodes.push_back( vit->point() );
    }
    
    tgCluster cluster( nodes, 0.0000025 );
    
    // traverse all edges in arr, and add as new clusterd edges in tmp;
    segnetArrangement::Edge_const_iterator eit;
    for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit ) {
        // look up edge source and target
        if ( eit->curve().data().size() == 1 ) {
            CurveData data = eit->curve().data().front();
        
            EPECPoint_2 source = eit->source()->point();
            EPECPoint_2 clust_source = cluster.Locate( source );

            EPECPoint_2 target = eit->target()->point();
            EPECPoint_2 clust_target = cluster.Locate( target );
        
            if ( clust_source != clust_target ) {
                segnetCurve curve( clust_source, clust_target );
            
                CGAL::insert( tmp, segnetCurveWithData(curve, data) );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::Cluster - curve data size != 1 (" << eit->curve().data().size() << ")" );
        }
    }
    
    // then assign back to arr
    arr.clear();
    arr = tmp;    
}
                
void tgSegmentNetwork::RemoveFingers( void )
{
    int   finger_id = 0;

#if DEBUG_FINGER_REMOVAL    
    char  layer[64];
#endif
    
#define dist_squared (0.0000025*0.0000025)    // 10 cm
    
    typename segnetArrangement::Vertex_const_iterator vit;
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {
        finger_id++;
        
        if ( vit->degree() == 1 ) {
            segnetArrangement::Halfedge_handle he = arr.non_const_handle( vit->incident_halfedges() );
            
            // let's mark this edge
            segnetVertexHandle src = he->source();            
            segnetVertexHandle trg = he->target();            
            
            // what's the distance
            if ( CGAL::squared_distance( src->point(), trg->point() ) < dist_squared ) {
                // remove this edge ( along with the target vertex )
                SG_LOG(SG_GENERAL, LOG_FINGER_REMOVAL, "tgSegmentNetwork:: finger " << finger_id << " Removing really short finger ");

#if DEBUG_FINGER_REMOVAL
                sprintf(layer, "finger_removal_short_%04d", finger_id);
                SGGeod geodTrg = SGGeod::fromDeg( CGAL::to_double(trg->point().x()), CGAL::to_double(trg->point().y()) );
                tgShapefile::FromGeod(geodTrg, datasource, layer, "short" );        
#endif

                arr.remove_edge( he, false, true );
            } else {
                // check the distance from the neighboring line ( around source ) - look for twin
                Halfedge_around_vertex_const_circulator first = src->incident_halfedges();
                Halfedge_around_vertex_const_circulator curr  = first;
                Halfedge_around_vertex_const_circulator prev, next;

                int  edge = 1;
                //bool found = false;
                
                do {
                    segnetArrangement::Halfedge_handle che = arr.non_const_handle( curr->twin() );
                        
                    if ( che == he ) {
                        bool removed = false;                        
                        //found = true;
                        
                        if (!removed) {
                            prev = curr; prev--;

                            // try to project trg onto the previous he
                            segnetLine    prevLine( prev->source()->point(), prev->target()->point() );
                            segnetPoint   prevProj = prevLine.projection( trg->point() );
                            segnetSegment prevSeg( prev->source()->point(), prev->target()->point() );

#if DEBUG_FINGER_REMOVAL         
                            sprintf(layer, "finger_removal_prv_proj_%04d", finger_id);
                            SGGeod geodPrevProj = SGGeod::fromDeg( CGAL::to_double(prevProj.x()), CGAL::to_double(prevProj.y()) );
                            tgShapefile::FromGeod(geodPrevProj, datasource, layer, "prvProj" );
#endif

                            if ( CGAL::do_overlap( prevSeg.bbox(), prevProj.bbox() ) ) {
                                // we have a winner - check distance between 
                                if ( CGAL::squared_distance( prevProj, trg->point() ) < dist_squared ) {
                                    SG_LOG(SG_GENERAL, LOG_FINGER_REMOVAL, "tgSegmentNetwork:: finger " << finger_id << " Removing finger close to previous edge" );
                                    
                                    removed = true;
                                    arr.remove_edge( he, false, true );                                
                                }
                            } 
                        }
                        
                        if (!removed) {
                            next = curr; next++;
                                                    
                            segnetLine    nextLine( next->source()->point(), next->target()->point() );
                            segnetPoint   nextProj = nextLine.projection( trg->point() );
                            segnetSegment nextSeg( next->source()->point(), next->target()->point() );

#if DEBUG_FINGER_REMOVAL         
                            sprintf(layer, "finger_removal_nxt_proj_%04d", finger_id);
                            SGGeod geodNextProj = SGGeod::fromDeg( CGAL::to_double(nextProj.x()), CGAL::to_double(nextProj.y()) );
                            tgShapefile::FromGeod(geodNextProj, datasource, layer, "nxtProj" );        
#endif

                            if ( CGAL::do_overlap( nextSeg.bbox(), nextProj.bbox() ) ) {
                                // we have a winner - check distance between 
                                if ( CGAL::squared_distance( nextProj, trg->point() ) < dist_squared ) {
                                    SG_LOG(SG_GENERAL, LOG_FINGER_REMOVAL, "tgSegmentNetwork:: finger " << finger_id << " Removing finger close to next edge" );
                        
                                    removed = true;
                                    arr.remove_edge( he, false, true );                                
                                }
                            }
                        }
                        break;
                    }
                    curr++;
                    edge++;
                    
                } while ( curr != first );
            }
        }
    }
    
    // remove any isolated verticies
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {
        if ( vit->is_isolated() ) {
            SG_LOG(SG_GENERAL, LOG_CLUSTERS, "tgSegmentNetwork::Removing isolated vertex ");
            arr.remove_isolated_vertex( arr.non_const_handle( vit )  );
        }
    }    
}

// TODO : currently, this looks for a nearby edge that lies directly in the path of the finger direction.
// TWO other cases exist
// 1) there's a nearby vertex ( within 1 m, but not our source, or sources source ) 
// 2) there's a nearby edge   ( within 1 m, right / left 90 degrees )
void tgSegmentNetwork::ExtendFingers( void )
{
    int  finger_id = 0;
    
#if DEBUG_FINGER_EXTENSION
    char layer[64];
#endif
    
    typename segnetArrangement::Vertex_const_iterator vit;
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {
        finger_id++;
        
        if ( vit->degree() == 1 ) {
            // project this vertex on the other halfedges connected to opposite node, 
            // to calculate the distance from this node to the prev/next edge
            SG_LOG(SG_GENERAL, LOG_FINGER_EXTENSION, "tgSegmentNetwork::Found finger candidate ");            
            
            // let's mark this edge
            segnetArrangement::Halfedge_handle fingerEdge = arr.non_const_handle( vit->incident_halfedges() );

            segnetVertexHandle src = fingerEdge->source();
            segnetVertexHandle trg = fingerEdge->target();
            
            // use tgEuclidean to find pos of next point
            SGGeod geodPrev, geodCurr;
            double course;
            
            geodPrev = SGGeod::fromDeg( CGAL::to_double(src->point().x()), CGAL::to_double(src->point().y()) );
            geodCurr = SGGeod::fromDeg( CGAL::to_double(trg->point().x()), CGAL::to_double(trg->point().y()) );
            
            // Walk from the nearest_vertex to the point p, using walk algorithm,
            // and find the location of the query point p. Note that the set fo edges
            // we have crossed so far is initially empty.
            course = TGEuclidean::courseDeg( geodPrev, geodCurr );
            segnetPoint minPoint;
            
            // First, try directly in front
            if ( ArbitraryRayShoot( trg, course, 5.0, minPoint, finger_id, "front" ) ) {
                if (fingerEdge->curve().data().size() == 1) {
                    segnetCurve curve( src->point(), minPoint );
                    CurveData   data = fingerEdge->curve().data().front();
                        
                    // remove the old ( if degree is one, this will remove the vertex as well )
                    arr.remove_edge( fingerEdge, false, false );

                    CGAL::insert( arr, segnetCurveWithData(curve, data) );
                } else {
                    SG_LOG(SG_GENERAL, LOG_FINGER_EXTENSION, "tgSegmentNetwork::Remove edge front ARS: COULDN'T GET DATA");
                }                                        
            } else if ( ArbitraryRayShoot( trg, course-90, 5.0, minPoint, finger_id, "left" ) ) {
                if (fingerEdge->curve().data().size() == 1) {
                    segnetCurve curve( src->point(), minPoint );
                    CurveData   data = fingerEdge->curve().data().front();
                        
                    // remove the old ( if degree is one, this will remove the vertex as well )
                    arr.remove_edge( fingerEdge, false, false );

                    CGAL::insert( arr, segnetCurveWithData(curve, data) );
                } else {
                    SG_LOG(SG_GENERAL, LOG_FINGER_EXTENSION, "tgSegmentNetwork::Remove edge left ARS: COULDN'T GET DATA");
                }
            } else if ( ArbitraryRayShoot( trg, course+90, 5.0, minPoint, finger_id, "right" ) ) {
                if (fingerEdge->curve().data().size() == 1) {
                    segnetCurve curve( src->point(), minPoint );
                    CurveData   data = fingerEdge->curve().data().front();
                        
                    // remove the old ( if degree is one, this will remove the vertex as well )
                    arr.remove_edge( fingerEdge, false, false );

                    CGAL::insert( arr, segnetCurveWithData(curve, data) );
                } else {
                    SG_LOG(SG_GENERAL, LOG_FINGER_EXTENSION, "tgSegmentNetwork::Remove edge right ARS: COULDN'T GET DATA");
                }
            } 
        }
    }
    
    // remove any isolated verticies
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {
        if ( vit->is_isolated() ) {
            SG_LOG(SG_GENERAL, LOG_CLUSTERS, "tgSegmentNetwork::Removing isolated vertex ");
            arr.remove_isolated_vertex( arr.non_const_handle( vit )  );
        }
    }    
}

void tgSegmentNetwork::FixShortSegments(void)
{
    std::vector<segnetNonConstHalfedgeHandle>   delEdges;
    std::vector<segnetCurveWithData>            newEdges;
    int  vertex_id = 1;
    
#define THRESH_REMOVE (0.0000125*0.0000125)    // 50 cm
#define THRESH_EXTEND (0.0000500*0.0000500)    //200 cm
    
#if DEBUG_SHORT_EDGES
    char layer[64];
#endif    
    
    // walk via vertices as we delete edges....
    typename segnetArrangement::Vertex_const_iterator vit;
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {
        if ( !vit->is_isolated() ) {
            Halfedge_around_vertex_circulator first = arr.non_const_handle( vit->incident_halfedges() );
            Halfedge_around_vertex_circulator curr  = first;

            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::Fix segments around vertex " << vertex_id );
            do {
                std::list<nodesPointHandle>              result;

                // only check edges where source or target degree > 2
                segnetVertexHandle sourceHandle = curr->source();
                segnetVertexHandle targetHandle = curr->target();

                int    src_degree  = sourceHandle->degree();
                int    trg_degree  = targetHandle->degree();        
                double sq_distance = CGAL::to_double(CGAL::squared_distance( sourceHandle->point(), targetHandle->point()));

                if (( src_degree > 2 ) || (trg_degree > 2 ) ) {    
                    SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::Fix segments around vertex " << vertex_id << " found an edge to check " );

                    if ( sq_distance < THRESH_REMOVE ) {
                        if ( ( src_degree > 2 ) && ( trg_degree <= 2 ) ) {
                            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::delete target" );

                            // delete target node - edges that target that node will now target source
                            ModifyEdges( targetHandle, sourceHandle->point(), sourceHandle, result, delEdges, newEdges );
                        } else if ( ( trg_degree > 2 ) && ( src_degree <= 2 ) ) {
                            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::delete source" );

                            // delete source node - edges that target that node will now target target
                            ModifyEdges( sourceHandle, targetHandle->point(), targetHandle, result, delEdges, newEdges );
                        } else {
                            // TODO : delete both nodes, and target midpoint
                            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::Fix segment: TODO : Delete both nodes use midpoint");
                        }
                    }    
                    else if ( sq_distance < THRESH_EXTEND ) {
                        SGGeod sourceGeod = SGGeod::fromDeg( CGAL::to_double(sourceHandle->point().x()), CGAL::to_double(sourceHandle->point().y()) );
                        SGGeod targetGeod = SGGeod::fromDeg( CGAL::to_double(targetHandle->point().x()), CGAL::to_double(targetHandle->point().y()) );
                        double course     = TGEuclidean::courseDeg(sourceGeod, targetGeod);
                    
                        // need to figure out which way to extend ( or both )
                        if ( ( src_degree > 2 ) && ( trg_degree <= 2 ) ) {
                            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::extend from source" );

                            // calculate new target from source, and magnitude
                            SGGeod newTargetGeod  = TGEuclidean::direct(sourceGeod, course, 0.2);
                            segnetPoint newTarget = segnetPoint( newTargetGeod.getLongitudeDeg(), newTargetGeod.getLatitudeDeg() );
                        
                            //ModifyEdges( targetHandle, newTarget, invalid_vh, delEdges, newEdges );
                        } else if ( ( trg_degree > 2 ) && ( src_degree <= 2 ) ) {
                            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::extend from target" );

                            // calculate new target from source, and magnitude
                            SGGeod newSourceGeod  = TGEuclidean::direct(targetGeod, course, -0.2);
                            segnetPoint newSource = segnetPoint( newSourceGeod.getLongitudeDeg(), newSourceGeod.getLatitudeDeg() );
                        
                            //ModifyEdges( sourceHandle, newSource, invalid_vh, delEdges, newEdges );
                        } else {
                            // calculate new source and target, from midpoint and magnitude/2
                            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::Fix segment: TODO : Extend both nodes away from midpoint");
                        }
                    }
                }
                
                curr++;
            } while ( curr != first );
        }        
        vertex_id++;
    }
    
    // now remove the old edges
    for ( unsigned int i=0; i<delEdges.size(); i++ ) {
        arr.remove_edge( delEdges[i], false, false );
    }
        
    // remove any isolated verticies
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {
        if ( vit->is_isolated() ) {
            SG_LOG(SG_GENERAL, LOG_FIX_SHORT_SEGMENT, "tgSegmentNetwork::FixShortSegments Removing isolated vertex ");
            arr.remove_isolated_vertex( arr.non_const_handle( vit )  );
        }
    }
    
    // add new edges
    for ( unsigned int i=0; i<newEdges.size(); i++ ) {
        CGAL::insert( arr, newEdges[i] );
    }    
}

void tgSegmentNetwork::RemoveColinearSegments( void )
{
    double pixelSize = 0.0000005;   // 2 cm
    
    std::list<segnetSegment>        srinput;
    segnetPolylineList              sroutput;
    std::list<segnetCurveWithData>  snapRounded;

    SGGeod                   start, end;
    double                   max_width;
    unsigned int             max_type;
    
    segnetArrangement::Edge_const_iterator eit;
    segnetPolylineList::const_iterator     oit;

    // generate input list from arrangement segments
    for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
    {
        srinput.push_back( segnetSegment( eit->source()->point(), eit->target()->point() ) );
    }
    
    // snapround
    CGAL::snap_rounding_2<segnetSRTraits, segnetSegmentList::const_iterator, segnetPolylineList>(srinput.begin(), srinput.end(), sroutput, pixelSize, true, false, 1);
        
    // done - move directly from snapround output to segnet output ( with curve data )
    oit = sroutput.begin();
    eit = arr.edges_begin();
    while( oit != sroutput.end() ) {
        // first, get the curve data to assign each output segment
        segnetTraits::Data_container::const_iterator    dit;
        max_width = 0.0;
        max_type  = 0;
        
        if ( eit->curve().data().size() == 0 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::GenerateOutput found an edge with no DATA\n");
        }
        
        for (dit = eit->curve().data().begin(); dit != eit->curve().data().end();  ++dit) {
            if (dit->width > max_width) {
                max_width = dit->width;
            }
            if (dit->type > max_type) {
                max_type = dit->type;
            }
        }

        if ( max_width < 0.1 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::GenerateOutput found an edge with width < 0.1\n");
        }
        
        if ( max_type == 0 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::GenerateOutput found an edge with type == 0\n");
        }
        
        CurveData   data( max_width, max_type );


        // output is a polyline - which corresponds to a single segment in input
        // move the output back into a list of segnetCurvesWithData
        
        segnetPolyline::const_iterator plsrcit, pltrgit;
        for (plsrcit = oit->begin(); plsrcit != oit->end(); plsrcit++) {
            pltrgit = plsrcit; pltrgit++;
            
            if ( pltrgit != oit->end() ) {
                if ( (*plsrcit) != (*pltrgit) ) {
#if 0                    
                    start = SGGeod::fromDeg( CGAL::to_double( plsrcit->x() ),
                                             CGAL::to_double( plsrcit->y() ) );
                    end   = SGGeod::fromDeg( CGAL::to_double( pltrgit->x() ),
                                             CGAL::to_double( pltrgit->y() ) );
                    
                    output.push_back( segnetEdge( start, end, max_width, max_type ) ); 
#endif
                    segnetCurve curve( *plsrcit, *pltrgit );
                    snapRounded.push_back( segnetCurveWithData(curve, data) );
                }
            }
        }

        oit++;
        eit++;
    }
    
    arr.clear();
    
    for ( std::list<segnetCurveWithData>::const_iterator it = snapRounded.begin(); it != snapRounded.end(); it++ ) {
        CGAL::insert( arr, *it );
    }
}

void tgSegmentNetwork::GenerateOutput( void ) 
{
    segnetArrangement::Edge_const_iterator eit;
    
    SGGeod       start, end;
    double       max_width;
    unsigned int max_type;
    
    for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
    {
        // convert to Geod
        start = SGGeod::fromDeg( CGAL::to_double( eit->source()->point().x() ),
                                 CGAL::to_double( eit->source()->point().y() ) );
        end   = SGGeod::fromDeg( CGAL::to_double( eit->target()->point().x() ),
                                 CGAL::to_double( eit->target()->point().y() ) );
        
        // Go over the incident edges of the current vertex and examine their width and type
        segnetTraits::Data_container::const_iterator    dit;
        max_width = 0.0;
        max_type  = 0;
        
        if ( eit->curve().data().size() == 0 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::GenerateOutput found an edge with no DATA\n");
        }
        
        for (dit = eit->curve().data().begin(); dit != eit->curve().data().end();  ++dit) {
            if (dit->width > max_width) {
                max_width = dit->width;
            }
            if (dit->type > max_type) {
                max_type = dit->type;
            }
        }

        if ( max_width < 0.1 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::GenerateOutput found an edge with width < 0.1\n");
        }
        
        if ( max_type == 0 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgSegmentNetwork::GenerateOutput found an edge with type == 0\n");
        }

        output.push_back( segnetEdge( start, end, max_width, max_type ) ); 
    }
}

void tgSegmentNetwork::BuildTree( void )
{
    tree.clear();
    
    segnetPoint none(0,0);
    
    typename segnetArrangement::Vertex_const_iterator vit;
    for ( vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit ) {
        nodesPointHandle nh( vit->point(), vit, none );
    
        tree.insert( nh );
    }
    
    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Built kd-tree with " << tree.size() << " nodes" );    
}

void tgSegmentNetwork::DumpPolys( void )
{
    segnetArrangement::Face_const_iterator fit;
    
    for( fit = arr.faces_begin(); fit != arr.faces_end(); fit++ ) {
        segnetArrangement::Face face = (*fit);
        if( face.has_outer_ccb() ) {
            segnetArrangement::Ccb_halfedge_const_circulator ccb = face.outer_ccb();
            segnetArrangement::Ccb_halfedge_const_circulator cur = ccb;
            segnetArrangement::Halfedge_const_handle         he;
            std::vector<tgSegment>                           segList;
            double  area = 0.0;
            SGVec2d a, b;
            
            segList.clear();
            do
            {
                he = cur;

                // ignore inner antenna
                if ( he->face() != he->twin()->face() ) {                    
                    //std::cout << "   [" << he->curve() << "]   " << "(" << he->target()->point() << ")";
                    SGGeod start = SGGeod::fromDeg( CGAL::to_double( he->source()->point().x() ), 
                                                    CGAL::to_double( he->source()->point().y() ) );
                    SGGeod end   = SGGeod::fromDeg( CGAL::to_double( he->target()->point().x() ), 
                                                    CGAL::to_double( he->target()->point().y() ) );
                
                    tgSegment seg( start, end );
                    segList.push_back( seg );
                    
                    a = SGGeod_ToSGVec2d( start );
                    b = SGGeod_ToSGVec2d( end );
                    
                    area += (b.x() + a.x()) * (b.y() - a.y());
                }
                
                ++cur;
            } while (cur != ccb);
            
            if ( fabs( area ) < 0.00000000001 ) {
                tgShapefile::FromSegmentList( segList, false, datasource, "polys", "poly" );
            }
        }
    }
}

void tgSegmentNetwork::ToShapefiles( const char* prefix )
{
    char layer[128];
    
    CGAL_precondition( arr.is_valid () );
    
    // first - write the vertex layer
    std::vector<SGGeod> vertex_list;
    typename segnetArrangement::Vertex_const_iterator vit;
    for ( vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit ) {
        vertex_list.push_back( SGGeod::fromDeg( CGAL::to_double(vit->point().x()), 
                                                CGAL::to_double(vit->point().y())) );
    }
    
    sprintf(layer, "%s_sn_vertices", prefix );
    tgShapefile::FromGeodList( vertex_list, false, datasource, layer, "vertex" );

    std::vector<tgSegment> segment_list;
    typename segnetArrangement::Edge_const_iterator eit;
    
    for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit ) {
        tgSegment   tgseg( SGGeod::fromDeg( CGAL::to_double( eit->source()->point().x() ),
                                            CGAL::to_double( eit->source()->point().y() ) ),
                           SGGeod::fromDeg( CGAL::to_double( eit->target()->point().x() ),
                                            CGAL::to_double( eit->target()->point().y() ) ) );
        segment_list.push_back( tgseg );
    }

    sprintf(layer, "%s_sn_edges", prefix );
    tgShapefile::FromSegmentList( segment_list, false, datasource, layer, "edge" );
}