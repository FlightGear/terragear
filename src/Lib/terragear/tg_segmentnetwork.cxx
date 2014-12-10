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
#include "tg_shapefile.hxx"
#include "tg_cgal.hxx"

tgSegmentNetwork::tgSegmentNetwork( const std::string debugRoot )
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
            
void tgSegmentNetwork::Clean( void )
{
    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Build kd-tree" );    
    
    // first, cluster the nodes
    Cluster();
    
    // then remove very short fingers, or fingers that end close to their neighbors
    RemoveFingers();
    
    // then try to extend existing fingers to nearby lines or vertices
    ExtendFingers();
    
    // genrate output vector
    GenerateOutput();
}

// group nearby nodes together, and update edges to the incident edges to use the
// cluster center
void tgSegmentNetwork::Cluster( void )
{
    // traverse the arrangement verticies, and see if they need to be grouped
    std::vector<segnetCurveWithData>  newEdges;

#if DEBUG_CLUSTERS
    int  clust_id = 0;    
    char layer[64];
#endif
    
    // need to llok up nearby nodes quickly
    BuildTree();
    
    typename segnetArrangement::Vertex_const_iterator vit;
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {        
        // first, query for any vertices nearby ( within max width )
        nodesPoint      center( CGAL::to_double( vit->point().x() ), 
                                CGAL::to_double( vit->point().y() ) );
        nodesFuzzyCir   query_circle(center, 0.0000025);  // approx 25 cm
        
        // list of tuples as a result
        std::list<nodesPointHandle>              result;

        // perform the query
        tree.search(std::back_inserter( result ), query_circle);

        // If we have a result, we may need to check for nearest...
        if ( result.size() > 1 ) {                        
            // Debug out, while calculating cluster center
            double total_x = 0.0f, total_y=0.0f; 

            std::list<nodesPointHandle>::iterator rit;
            for ( rit=result.begin(); rit != result.end(); rit++ ) {
                segnetVertexHandle handle = boost::get<1>(*rit);

                double x = CGAL::to_double( handle->point().x() );
                double y = CGAL::to_double( handle->point().y() );
                
                total_x += x;
                total_y += y;

#if DEBUG_CLUSTERS
                sprintf(layer, "clust_%04d", clust_id);                
                SGGeod qn = SGGeod::fromDeg( x, y );
                if ( vit == handle ) {
                    tgShapefile::FromGeod(qn, datasource, layer, "query_center" );        
                } else {
                    tgShapefile::FromGeod(qn, datasource, layer, "query_node" );
                }
#endif                
            }
            
            segnetPoint newNode( total_x/result.size(), total_y/result.size() );
            segnetVertexHandle newTarget = CGAL::insert_point( arr, newNode );

            // for each old vertex, cirulate the old incident edges 
            // (w/that node as target, and remember the sources)
            for ( rit=result.begin(); rit != result.end(); rit++ ) {
                segnetVertexHandle          oldHandle = boost::get<1>(*rit);

                // use arrangement observer to detect when the vertex is removed                
                while ( !oldHandle->is_isolated() ) {
                    segnetArrangement::Halfedge_handle he = arr.non_const_handle( oldHandle->incident_halfedges() );
                    segnetVertexHandle                 oldSource = he->source();

                    segnetCurve curve( oldSource->point(), newTarget->point() );
                    if (he->curve().data().size() == 1) {
                        // CurveData data = he->curve().data().front();
                        CurveData data = he->curve().data().front();

                        // We need to use free function as we want to keep the curve history
                        newEdges.push_back( segnetCurveWithData( curve, data ) );
                        
                        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Remove incident edge from vertex w/degree " << oldHandle->degree() << " width was " << data.width << ", type was " << data.type );
                        
                        // remove the old ( if degree is one, this will remove the vertex as well )
                        arr.remove_edge( he, false, false );
                    } else {
                        SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Remove incident edge from vertex w/degree " << oldHandle->degree() << " COULDN'T GET DATA");
                    }                        
                }
            }
        }
    }
    
    // remove any isolated verticies
    for( vit = arr.vertices_begin() ; vit != arr.vertices_end(); vit++ ) {
        if ( vit->is_isolated() ) {
            SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Removing isolated vertex ");
            arr.remove_isolated_vertex( arr.non_const_handle( vit )  );
        }
    }
    
    // add new edges
    for ( unsigned int i=0; i<newEdges.size(); i++ ) {
        CGAL::insert( arr, newEdges[i] );
    }
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
                SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork:: finger " << finger_id << " Removing really short finger ");

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
                                    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork:: finger " << finger_id << " Removing finger close to previous edge" );
                                    
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
                                    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork:: finger " << finger_id << " Removing finger close to next edge" );
                        
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
            SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Found finger candidate ");            
            
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
                    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Remove edge front ARS: COULDN'T GET DATA");
                }                                        
            } else if ( ArbitraryRayShoot( trg, course-90, 5.0, minPoint, finger_id, "left" ) ) {
                if (fingerEdge->curve().data().size() == 1) {
                    segnetCurve curve( src->point(), minPoint );
                    CurveData   data = fingerEdge->curve().data().front();
                        
                    // remove the old ( if degree is one, this will remove the vertex as well )
                    arr.remove_edge( fingerEdge, false, false );

                    CGAL::insert( arr, segnetCurveWithData(curve, data) );
                } else {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Remove edge left ARS: COULDN'T GET DATA");
                }
            } else if ( ArbitraryRayShoot( trg, course+90, 5.0, minPoint, finger_id, "right" ) ) {
                if (fingerEdge->curve().data().size() == 1) {
                    segnetCurve curve( src->point(), minPoint );
                    CurveData   data = fingerEdge->curve().data().front();
                        
                    // remove the old ( if degree is one, this will remove the vertex as well )
                    arr.remove_edge( fingerEdge, false, false );

                    CGAL::insert( arr, segnetCurveWithData(curve, data) );
                } else {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgSegmentNetwork::Remove edge right ARS: COULDN'T GET DATA");
                }
            } 
        }
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
    typename segnetArrangement::Vertex_const_iterator vit;
    for ( vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit ) {
        nodesPoint np( CGAL::to_double( vit->point().x() ), CGAL::to_double( vit->point().y() ) );
        nodesPointHandle nh(np, vit);
    
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

void tgSegmentNetwork::ToShapefiles( )
{
    CGAL_precondition( arr.is_valid () );
    
    // first - write the vertex layer
    std::vector<SGGeod> vertex_list;
    typename segnetArrangement::Vertex_const_iterator vit;
    for ( vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit ) {
        vertex_list.push_back( SGGeod::fromDeg( CGAL::to_double(vit->point().x()), 
                                                CGAL::to_double(vit->point().y())) );
    }
    
    tgShapefile::FromGeodList( vertex_list, false, datasource, "segnet_verticies", "vertex" );

    std::vector<tgSegment> segment_list;
    typename segnetArrangement::Edge_const_iterator eit;
    
    for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit ) {
        tgSegment   tgseg( SGGeod::fromDeg( CGAL::to_double( eit->source()->point().x() ),
                                            CGAL::to_double( eit->source()->point().y() ) ),
                           SGGeod::fromDeg( CGAL::to_double( eit->target()->point().x() ),
                                            CGAL::to_double( eit->target()->point().y() ) ) );
        segment_list.push_back( tgseg );
    }

    tgShapefile::FromSegmentList( segment_list, false, datasource, "segnet_edges", "edge" );
}