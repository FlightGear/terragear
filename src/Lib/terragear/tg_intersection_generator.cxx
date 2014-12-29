#include "tg_segmentnetwork.hxx"
#include "tg_polygon.hxx"
#include "tg_shapefile.hxx"

#include "tg_intersection_generator.hxx"

#define DEBUG_INTERSECTIONS (0)
#define LOG_INTERSECTION    (SG_DEBUG)

void tgIntersectionGenerator::Insert( const SGGeod& s, const SGGeod& e, double w, unsigned int t )
{    
    segNet.Add( s, e, w, t );
}

void tgIntersectionGenerator::ToShapefile( const char* prefix )
{
#if DEBUG_INTERSECTIONS    
    char name[32];
    
    for (tgintersectionedge_it it = edgelist.begin(); it != edgelist.end(); it++) {
        tgSegment seg( (*it)->start->GetPosition(), (*it)->end->GetPosition() );
        sprintf( name, "seg_%04ld", (*it)->id );
        tgShapefile::FromSegment( seg, true, datasource, prefix, name );    
    }
#endif    
}

void tgIntersectionGenerator::Execute( bool clean )
{
    // Segnet has all of the edges and nodes in an arrangement : clean it
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Clean network " );
    edgelist.clear();
    
    segNet.Clean( clean );
        
    // now retreive the cleaned edges 
    for ( segnetedge_it snit = segNet.output_begin(); snit != segNet.output_end(); snit++ ) {
        // get the start and end nodes
        tgIntersectionNode* start = nodelist.Add( snit->start );
        tgIntersectionNode* end   = nodelist.Add( snit->end );

        if ( start != end ) {
            tgIntersectionEdge* newEdge = new tgIntersectionEdge( start, end, snit->width, snit->type, debugRoot );
            edgelist.push_back( newEdge );
        }
    }
    
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Saving Cleaned to " << debugRoot );
    ToShapefile("cleaned");

    // add end cap segments
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Execute:AddCaps");
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: ConstrainEdges at node " << i << " of " << nodelist.size() );
        nodelist[i]->AddCapEdges( nodelist, edgelist );
    }
    
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Saving Capped to " << debugRoot );
    ToShapefile("Capped");
    
    //find shared edge constraints at each node
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Execute:AddConstraints");
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: ConstrainEdges at node " << i << " of " << nodelist.size() );
        nodelist[i]->ConstrainEdges();
    }

    // Generate the edge from each node
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Execute:GenerateEdges");
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: GenerateEdges at node " << i << " of " << nodelist.size() );
        nodelist[i]->GenerateEdges();
    }

    // Remove any edges that didn't get intersected
    // verifty all edges have been intersected
    tgintersectionedge_it it = edgelist.begin();
    while ( it != edgelist.end() ) {
        if ( !(*it)->Verify( FLAGS_INTERSECT_CONSTRAINTS_COMPLETE) ) {
            (*it)->start->DelEdge( true, *it );
            (*it)->end->DelEdge( false, *it );
            it = edgelist.erase( it );
        } else {
            it++;
        }
    }
        
    // fix multisegment intersections - happens when segments do not diverge from each other before the end of the edge
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::FixMultiSegment");
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: FixSpecialIntersections at node " << i << " of " << nodelist.size() );
        nodelist[i]->CompleteSpecialIntersections();
    }
    
    // verifty all edges have been intersected
    for (tgintersectionedge_it it = edgelist.begin(); it != edgelist.end(); it++) {
        (*it)->Verify( FLAGS_INTERSECT_CONSTRAINTS_COMPLETE);
    }
    
    // complete the edges
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::CompletePolygon");
    for (tgintersectionedge_it it = edgelist.begin(); it != edgelist.end(); it++) {
        (*it)->Complete();
    }

#if DEBUG_INTERSECTIONS    
    SG_LOG(SG_GENERAL, SG_INFO, "Saving complete to " << datasource );    

    for (tgintersectionedge_it it = edgelist.begin(); it != edgelist.end(); it++) {
        char feat[32];
        sprintf(feat, "edge_%05ld", (*it)->id );
        tgPolygon poly = (*it)->GetPoly("complete");
        tgShapefile::FromPolygon( poly, false, false, datasource, "complete", feat );

        (*it)->ToShapefile();        
    }
#endif
    
    // to texture, start at caps, and try to cross intersections nicely ( push the start_v )
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Texture");        
    for (unsigned int i=0; i<nodelist.size(); i++) {
        // can't always assume all features have caps ( think circle )
        if ( /* nodelist[i]->IsCap() && */ !nodelist[i]->IsTextureComplete() ) {
            // start here
            SG_LOG(SG_GENERAL, SG_DEBUG, "tgIntersectionGenerator::Start texture at node " << i << " of " << nodelist.size() );
            tgIntersectionNode*     cur_node = nodelist[i];
            tgIntersectionEdgeInfo* cur_info = nodelist[i]->GetFirstEdgeInfo();
            bool done      = false;
            bool reset_v   = false;
            double start_v = 0.0f;
            
            std::stack<tgIntersectionNode*> untextured_node_stack;
            
            while(!done) {
                cur_node->SetStartV( start_v );
                if ( cur_info ) {
                    if ( cur_info->GetEdge() ) {
                        start_v = cur_info->Texture(start_v, texInfoCb);
            
                        SG_LOG(SG_GENERAL, SG_DEBUG, "tgIntersectionGenerator::Get next node " << cur_node );
                        done = cur_node->GetNextConnectedNodeAndEdgeInfo( cur_info, cur_node, untextured_node_stack, reset_v );
                        SG_LOG(SG_GENERAL, SG_DEBUG, "tgIntersectionGenerator::Got next node " << cur_node << " info " << cur_info << " done? " << done );
                
                        if ( reset_v ) {
                            start_v = cur_node->GetStartV();
                        }
                    } else {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "tgIntersectionGenerator::cur_info->edge is NULL");
                        done = true;
                    }
                } else {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "tgIntersectionGenerator::cur_info is NULL");
                    done = true;
                }
            }
        }
    }

    for (tgintersectionedge_it it = edgelist.begin(); it != edgelist.end(); it++) {
        (*it)->Verify( FLAGS_TEXTURED );
    }
    
    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionGenerator::Complete");    
}