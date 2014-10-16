#include "tg_polygon.hxx"
#include "tg_shapefile.hxx" 
#include "tg_graph.hxx"
#include "tg_misc.hxx"
#include "tg_euclidean.hxx"

// TODO:
// 1) Fix clipcontour 1/2 : done
// 2) Determine which edges to fixup (3rd stage) : done
// 3) GetNext edge should be node based, not edgelist based
//   (walk the nodes)
      
tgIntersectionNode::tgIntersectionNode( const SGGeod& pos )
{
    position = pos;
    edgeList.clear();
}

void tgIntersectionNode::AddEdge( bool originated, tgIntersectionEdge* edge ) 
{
    edgeList.insert( new tgIntersectionEdgeInfo( originated, edge ) );
}

tgIntersectionEdgeInfo* tgIntersectionNode::GetPrevEdgeInfo( tgIntersectionEdgeInfo* cur_info )
{
    tgintersectionedgeinfo_it prv, cur;
    tgIntersectionEdgeInfo*   prv_info = NULL;
    const tgIntersectionEdge* cur_edge = cur_info->GetEdge();

    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::GetPrevEdgeInfo: enter - node has " << edgeList.size() << " edges " );
    
    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
        // edge info is unique from node to node - but the edge itself is shared
        if( (*cur)->GetEdge() == cur_edge ) {
            // we found the current edge info - return prv;
            if ( cur != edgeList.begin() ) {
                prv = cur;
                prv--;
            } else {
                prv = edgeList.end();
                prv--;
            }
            prv_info = (*prv);
            break;
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::GetPrevEdgeInfo: exit - prv_info is " << prv_info );
    
    return prv_info;
}

tgIntersectionEdgeInfo* tgIntersectionNode::GetNextEdgeInfo( tgIntersectionEdgeInfo* cur_info )
{
    tgintersectionedgeinfo_it cur, nxt;
    tgIntersectionEdgeInfo*   nxt_info = NULL;
    const tgIntersectionEdge* cur_edge = cur_info->GetEdge();

    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::GetNextEdgeInfo: enter - node has " << edgeList.size() << " edges ");

    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
        // edge info is unique from node to node - but the edge itself is shared
        if( (*cur)->GetEdge() == cur_edge ) {
            // we found the current edge info - return nxt;
            nxt = cur;
            nxt++;

            if ( nxt == edgeList.end() ) {
                nxt = edgeList.begin();
            }
            nxt_info = (*nxt);
            break;
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::GetNextEdgeInfo: exit - nxt_info is " << nxt_info );
    
    return nxt_info;
}

// Step 1 - for nodes with multiple edges, bisect the angles
void tgIntersectionNode::ConstrainEdges( void ) 
{
    tgintersectionedgeinfo_it cur, prev;
    
    switch( edgeList.size() ) {
        case 1:
            // add cap
            break;
            
        case 2:
        default:
        {
            // Step 1 
            // Generate the bisector rays and add to the edges
            GenerateBisectRays();            
            break;
        }    
    }
}

// Step 2 - create edge shapes
void tgIntersectionNode::GenerateEdges()
{
    switch( edgeList.size() ) {
        case 1:
            // add cap
            break;
            
        case 2:
        default:
        {              
            // Step 2
            // Intersect constraints / edges
            IntersectBisectRays();
            break;
        }    
    }
}

// Step 3 - fix edges intersecting past 1 segment
void tgIntersectionNode::FixMultisegmentIntersections( void )
{
    tgintersectionedgeinfo_it cur, nxt;
        
    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
        tgIntersectionEdgeInfo* cur_info = (*cur);
        tgIntersectionEdgeInfo* nxt_info;

        // detecting MultiSegment in stage2 is difficult 
        // (but should be possible)
        // use brute force for now
        if ( cur_info->IsMultiSegment() ) {
            // get next edge info - with wrap around
            nxt = cur;
            nxt++;
            if ( nxt == edgeList.end() ) {
                nxt = edgeList.begin();
            }
            
            nxt_info = (*nxt);
            CompleteIntersection( cur_info, nxt_info );
        }
    }
}

void tgIntersectionNode::GenerateBisectRays( void ) 
{
    tgintersectionedgeinfo_it prv, cur, nxt;
        
    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
        // Get previous edge info - with wrap arounnd
        if ( cur == edgeList.begin() ) {
            prv = edgeList.end();
        } else {
            prv = cur;
        }
        prv--;

        // get next edge info - with wrap around
        nxt = cur;
        nxt++;
        if ( nxt == edgeList.end() ) {
            nxt = edgeList.begin();
        }
        
        // we need to get the edge info to determine the edge origination info
        tgIntersectionEdgeInfo* prv_info = (*prv);
        tgIntersectionEdgeInfo* cur_info = (*cur);
        tgIntersectionEdgeInfo* nxt_info = (*nxt);

        double prv_heading;
        double cur_heading;
        double nxt_heading;
        
        double bprv_heading;
        double bnxt_heading;
        
        // the edge info always has heading with respect to the node
        cur_heading = cur_info->GetHeading();
        prv_heading = prv_info->GetHeading();
        nxt_heading = nxt_info->GetHeading();
        
        // we want right turns - so calc angle a,b,c from cur,origin,prv
        bprv_heading = Bisect( position, cur_heading, prv_heading, true );
        tgRay bp_ray = tgRay( position, bprv_heading );

        // we want right turns - so calc angle a,b,c from nxt,origin,cur
        bnxt_heading = Bisect( position, nxt_heading, cur_heading, true );
        tgRay bn_ray = tgRay( position, bnxt_heading );
                
        // Add the bisecting ray between prev and current as a constraint
        if ( cur_info->IsOriginating() ) {
            SG_LOG(SG_GENERAL, SG_INFO, "GenerateBisectRays: Add BR constraint to edge " << cur_info->GetEdge()->id << " from " << position << " heading " << std::setprecision(16) << bnxt_heading << " from cur (" << std::setprecision(16) << cur_heading << ", originating) and next (" << std::setprecision(16) << nxt_heading << ")" );
            cur_info->GetEdge()->AddBottomRightConstraint(bn_ray);

            SG_LOG(SG_GENERAL, SG_INFO, "GenerateBisectRays: Add BL constraint to edge " << cur_info->GetEdge()->id << " from " << position << " heading " << std::setprecision(16) << bprv_heading << " from cur (" << std::setprecision(16) << cur_heading << ", originating) and prev (" << std::setprecision(16) << prv_heading << ")" );
            cur_info->GetEdge()->AddBottomLeftConstraint(bp_ray);
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "GenerateBisectRays: Add TL constraint to edge " << cur_info->GetEdge()->id << " from " << position << " heading " << std::setprecision(16) << bnxt_heading << " from cur (" << std::setprecision(16) << cur_heading << ", originating) and next (" << std::setprecision(16) << nxt_heading << ")" );
            cur_info->GetEdge()->AddTopLeftConstraint(bn_ray);

            SG_LOG(SG_GENERAL, SG_INFO, "GenerateBisectRays: Add TR constraint to edge " << cur_info->GetEdge()->id << " from " << position << " heading " << std::setprecision(16) << bprv_heading << " from cur (" << std::setprecision(16) << cur_heading << ", originating) and prev (" << std::setprecision(16) << prv_heading << ")" );
            cur_info->GetEdge()->AddTopRightConstraint(bp_ray);
        }
    }
}

// this should be an edge traversal, not node traversal
// TODO : Add a tgIntersectionEdge function to handle each constraint
// we do the same thing 4 times in this function
void tgIntersectionNode::IntersectBisectRays( void )
{
    tgintersectionedgeinfo_it cur;
    
    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) { 
        tgIntersectionEdgeInfo* cur_info = (*cur);        
        tgIntersectionEdge*     cur_edge = cur_info->GetEdge();
        
        if ( cur_edge ) {
            if ( cur_edge->IntersectConstraintsAndSides( cur_info->IsOriginating() ) ) {
                SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::IntersectBisectRays: originating " << cur_info->IsOriginating() << " edge " << cur_edge->id << " is multisegment" );
                cur_info->SetMultiSegment(true);
            } else {
                cur_info->SetMultiSegment(false);
            }
            
            tgPolygon p = cur_edge->GetPoly("IBR");
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: NO cur edge" );
        }
    }
}

void tgIntersectionNode::CompleteIntersection( tgIntersectionEdgeInfo* cur_info, tgIntersectionEdgeInfo* nxt_info )
{
    // current position and heading of the bisector
    double ce_heading = 0.0, ne_heading = 0.0, bisect_heading;
    SGGeod bisect_position = position;
    
    // remember how to traverse current and next edge iterators (prev or next)
    bool ce_originating;
    bool ne_originating;

    tgIntersectionEdge* ce;
    tgIntersectionEdge* ne;

    char layer[128];
    char prefix[64];
    int  cur_iteration = 1;

    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: edge " << cur_info->GetEdge()->id << " and " << nxt_info->GetEdge()->id );
    
    // iterate until the end of the intersection is found
    bool done = false;    
    while (!done) {
        ce_originating = cur_info->IsOriginating();
        ne_originating = nxt_info->IsOriginating();
        
        ce = cur_info->GetEdge();
        ne = nxt_info->GetEdge();

        // first, calculate the bisect heading
        ce_heading = ce->GetHeading( ce_originating );
        ne_heading = ne->GetHeading( ne_originating );
        
        sprintf( prefix, "CI%02d", cur_iteration );
        tgPolygon p;
        p = ce->GetPoly(prefix);
        p = ne->GetPoly(prefix);
            
        // add the bisect position to constraint
        ce->AddRightSegmentConstraint( bisect_position, ce_originating, false );
        ne->AddLeftSegmentConstraint( bisect_position, ne_originating, false );
            
        bisect_heading = Bisect( bisect_position, ne_heading, ce_heading, true );

        SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: ce_heading is " << ce_heading << " ne_heading is " << ne_heading << " bisct heading is " << bisect_heading);
            
        tgRay bisector( bisect_position, bisect_heading );
            
        // find the edge whose constraint we intersect with first
        // current top right, or next top left (w.r.t. originating)
        tgRay ce_constraint = ce->GetTopRightConstraint( ce_originating );
        tgRay ne_constraint = ne->GetTopLeftConstraint( ne_originating );

        tgLine ce_side      = ce->GetRightSide( ce_originating );
        tgLine ne_side      = ne->GetLeftSide(  ne_originating );
            
        SGGeod ce_con_intersect, ne_con_intersect;
        double ce_con_dist = -1000.00, ne_con_dist = -1000.00;
            
        SGGeod ce_side_intersect, ne_side_intersect;
        double ce_side_dist = -1000.00, ne_side_dist = -1000.00;
            
        sprintf( layer, "%03ld_%03ld_%d_BISECTOR", ce->id, ne->id, cur_iteration );
        tgShapefile::FromRay( bisector, "./edge_dbg", layer, "bisector" );

        sprintf( layer, "%03ld_%03ld_%d_CE_TR", ce->id, ne->id, cur_iteration );
        tgShapefile::FromRay( ce_constraint, "./edge_dbg", layer, "ce" );

        sprintf( layer, "%03ld_%03ld_%d_NE_TL", ce->id, ne->id, cur_iteration );
        tgShapefile::FromRay( ne_constraint, "./edge_dbg", layer, "ne" );

        sprintf( layer, "%03ld_%03ld_%d_CE_RS", ce->id, ne->id, cur_iteration );
        tgShapefile::FromLine( ce_side, "./edge_dbg", layer, "ce" );

        sprintf( layer, "%03ld_%03ld_%d_NE_LS", ce->id, ne->id, cur_iteration );
        tgShapefile::FromLine( ne_side, "./edge_dbg", layer, "ne" );
            
        if ( bisector.Intersect( ce_constraint, ce_con_intersect ) ) {
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ce_con_dist = TGEuclidean::distanceM( bisect_position, ce_con_intersect );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: ERROR - can't intersect bisector with ce top right constraint");
                
            done = true;
            break;
        }
 
        if ( bisector.Intersect( ne_constraint, ne_con_intersect ) ) {
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ne_con_dist = TGEuclidean::distanceM( bisect_position, ne_con_intersect );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: ERROR - can't intersect bisector with ne top left constraint");
               
            done = true;
            break;
        }

        if ( bisector.Intersect( ce_side, ce_side_intersect ) ) {
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ce_side_dist = TGEuclidean::distanceM( bisect_position, ce_side_intersect );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: INFO - can't intersect bisector with ce right side");
        }
 
        if ( bisector.Intersect( ne_side, ne_side_intersect ) ) {
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ne_side_dist = TGEuclidean::distanceM( bisect_position, ne_side_intersect );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: INFO - can't intersect bisector with ne left side");
        }

        SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: ce_con_dist is " << std::setprecision(16) << ce_con_dist << " ne_con_dist is " << std::setprecision(16) << ne_con_dist);
        SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: ce_side_dist is " << std::setprecision(16) << ce_side_dist << " ne_side_dist is " << std::setprecision(16) << ne_side_dist);
            
        if ( ce_con_dist > 0.0 && ne_con_dist > 0.0 ) {
            // if the side intersection is lower than the constraint distance, than we are done
            if ( (ce_side_dist >= 0.0) && (ce_side_dist < ce_con_dist) ) {
                // current edge is done - finish it
                ce->AddRightSegmentConstraint( ce_side_intersect, ce_originating, true );

                // next edge should be done as well - check
                if ( (ne_side_dist >= 0.0) && (ne_side_dist < ne_con_dist) ) {
                    ne->AddLeftSegmentConstraint( ne_side_intersect, ne_originating, true );
                } else {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionNode::CompleteIntersection: ERROR - cur edge complete w/side intersect, but ne is NOT");
                }
                    
                done = true;
            } else {
                // Finish the correct edge, and increment to next unfinished edge
                if ( ce_con_dist < ne_con_dist ) {
                    bisect_position = ce_con_intersect;
                    
                    // ne will get bisector position added at the beginning of next loop
                    ce->AddRightSegmentConstraint( ce_con_intersect, ce_originating, true );
                    // ne->AddLeftSegmentConstraint( ce_con_intersect, ne_originating, false );
                    
                    if ( ce_originating ) {
                        // get the next edge (CCW) from the END node
                        cur_info = ce->end->GetNextEdgeInfo( cur_info );
                    } else {
                        // get the next edge (CCW) from the START node
                        cur_info = ce->start->GetNextEdgeInfo( cur_info );
                    }
                    
                    if (cur_info == NULL) {
                        done = true;
                    }
                } else {
                    bisect_position = ne_con_intersect;
                    
                    // ce will get bisector position added at the beginning of next loop
                    // ce->AddRightSegmentConstraint( ne_con_intersect, ce_originating, false );
                    ne->AddLeftSegmentConstraint( ne_con_intersect, ne_originating, true );
                    
                    if ( ne_originating ) {
                        // get the prev edge (CW) from the END node
                        nxt_info = ne->end->GetPrevEdgeInfo( nxt_info );
                    } else {
                        // get the prev edge (CW) from the START node
                        nxt_info = ne->start->GetPrevEdgeInfo( nxt_info );
                    }
                    
                    if (nxt_info == NULL) {
                        done = true;
                    }
                }
            }
        }

        sprintf( prefix, "CI_DONE%02d", cur_iteration );
        p = ce->GetPoly(prefix);
        p = ne->GetPoly(prefix);
                
        cur_iteration++;
    }
}



// generate intersection edge in euclidean space
tgIntersectionEdge::tgIntersectionEdge( tgIntersectionNode* s, tgIntersectionNode* e, double w, unsigned int t ) 
{
    static unsigned int ge_count = 0;

    start = s;
    end   = e;
    width = w;
    type  = t;
    
    id = ++ge_count;
    sprintf(datasource, "./edge_dbg/edge_%02ld/", id );
    
    // we need to add this edge between start and end to handle multiple edges at a node
    s->AddEdge( true, this );
    e->AddEdge( false, this );
    
    double ecourse   = TGEuclidean::courseDeg(start->GetPosition(), end->GetPosition());
    double elcourse  = SGMiscd::normalizePeriodic(0, 360, ecourse - 90);
        
    botLeft   = TGEuclidean::direct( start->GetPosition(), elcourse,  width/2 );    
    botRight  = TGEuclidean::direct( start->GetPosition(), elcourse, -width/2 );

    topLeft   = TGEuclidean::direct( end->GetPosition(), elcourse,  width/2 );
    topRight  = TGEuclidean::direct( end->GetPosition(), elcourse, -width/2 );
    
    side_l    = tgLine( botLeft,  topLeft );
    side_r    = tgLine( botRight, topRight );
}
    
void tgIntersectionEdge::ToShapefile( void ) const
{            
    // draw line from start to end
    tgSegment skel = tgSegment( start->GetPosition(), end->GetPosition() );
    tgShapefile::FromSegment( skel, datasource, "skeleton", "edge" );
    
    // just write the array of constraints
    tgShapefile::FromRayList( constrain_bl, datasource, "constraint_bl", "edge" );
    tgShapefile::FromRayList( constrain_br, datasource, "constraint_br", "edge" );
    tgShapefile::FromRayList( constrain_tr, datasource, "constraint_tr", "edge" );
    tgShapefile::FromRayList( constrain_tl, datasource, "constraint_tl", "edge" );
    
    tgShapefile::FromLine( side_l, datasource, "side_l", "edge" );
    tgShapefile::FromLine( side_r, datasource, "side_r", "edge" );        
}

SGGeod tgIntersectionEdge::IntersectCorner( const SGGeod& pos, tgray_list& constraint1, tgray_list& constraint2, 
                                     tgLine& side, bool& constraint_nearest, 
                                     const char* c1name, const char* c2name, const char* sname )
{
    char   layer[128];
    double side_intersect_dist = -1;
    double ray_intersect_dist  = -1;

    SGGeod side_intersect;
    SGGeod ray_intersect;
    SGGeod nearest;
    
    if ( constraint1.size() == 1 ) {        
        // 1) intersect 1st constraint ray with side
        sprintf( layer, "%s_%s_INT", c1name, sname );
        SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: Edge " << id << ": " << layer );

        tgShapefile::FromLine( side, datasource, layer, sname );
        tgShapefile::FromRay( constraint1[0], datasource, layer, c1name );

        if ( constraint1[0].Intersect( side, side_intersect ) ) {
            sprintf( layer, "%s_%s_PT", c1name, sname );
            tgShapefile::FromGeod( side_intersect, datasource, layer, "int" );

            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            side_intersect_dist = TGEuclidean::distanceM( pos, side_intersect );
            SG_LOG(SG_GENERAL, SG_INFO, "  - intersection is " << side_intersect_dist << " away " );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "  - no intersection" );
        }

        // 2) intersect constraint1 with constraint2 
        if ( constraint2.size() == 1 ) {
            sprintf( layer, "%s_%s_INT", c1name, c2name );
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: Edge " << id << ": " << layer );

            tgShapefile::FromRay( constraint2[0], datasource, layer, c2name );
            tgShapefile::FromRay( constraint1[0], datasource, layer, c1name );

            if ( constraint1[0].Intersect( constraint2[0], ray_intersect ) ) {
                sprintf( layer, "%s_%s_PT", c1name, c2name );
                tgShapefile::FromGeod( ray_intersect, datasource, layer, "int" );

                // if we have a point intersection, what is the distance between 
                // the node and the intersection
                ray_intersect_dist = TGEuclidean::distanceM( pos, ray_intersect );
                SG_LOG(SG_GENERAL, SG_INFO, "  - intersection is " << ray_intersect_dist << " away " );
            } else {
                SG_LOG(SG_GENERAL, SG_INFO, "  - no intersection" );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: cur edge has " << constraint2.size() << " constraints - expecting 1" );
        }
        
        // the final nodes will be ordered from start, and circle CCW.
        // for the the left side nodes will be added to the back from bottom
        // and to the front from top
        if ( (side_intersect_dist > 0.0) && (ray_intersect_dist < -0.1 ) ) {
            nearest = side_intersect;
            constraint_nearest = false;
        } else if ( (ray_intersect_dist > 0.0) && (side_intersect_dist < -0.1 ) ) {
            nearest = ray_intersect;
            if ( ray_intersect_dist <= width/2 ) {
                constraint_nearest = true;
            } else {
                constraint_nearest = false;
            }
        } else if ( side_intersect_dist < ray_intersect_dist ) {
            nearest = side_intersect;
            constraint_nearest = false;            
        } else if ( ray_intersect_dist < side_intersect_dist ) {
            nearest = ray_intersect;
            if ( ray_intersect_dist <= width/2 ) {
                constraint_nearest = true;
            } else {
                constraint_nearest = false;
            }
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: CAN'T DETERMINE INTERSECT" );
        }
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: cur edge has " << constraint1.size() << " constraints - expecting 1" );                    
    }
    
    return nearest;
}

bool tgIntersectionEdge::IntersectConstraintsAndSides(bool originating)
{    
    SGGeod nearest;
    bool   edge_needs_fixing = false;
    bool   dummy = false;
    
    SG_LOG( SG_GENERAL, SG_INFO, "IntersectConstraintsAndSides: edge " << id << " originating " << originating );
    if ( originating ) {
        if ( constrain_bl.size() == 1 ) {
            nearest = IntersectCorner( start->GetPosition(), constrain_bl, constrain_tl, 
                                       side_l, dummy, "BL", "TL", "LS" );
                        
            SG_LOG( SG_GENERAL, SG_INFO, "                            : push_back bottom left at lc pos " << left_contour.size() << " " << nearest );
            AddLeftContour( nearest, originating );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: cur edge has " << constrain_bl.size() << " bl constraints - expecting 1" );                    
        }
        SG_LOG( SG_GENERAL, SG_INFO, "                            : push_back bottom middle at lc pos " << left_contour.size() << " " << start->GetPosition() );
        AddLeftContour( start->GetPosition(), originating );
        
        if ( constrain_br.size() == 1 ) {
            nearest = IntersectCorner( start->GetPosition(), constrain_br, constrain_tr, 
                                       side_r, edge_needs_fixing, "BR", "TR", "RS" );
            
            // TEST TEST TEST: only fix edge if nearest is closer to end than start
            if ( TGEuclidean::distanceM( start->GetPosition(), nearest ) <
                 TGEuclidean::distanceM( end->GetPosition(), nearest ) ) {
                edge_needs_fixing = false;
            }

            SG_LOG( SG_GENERAL, SG_INFO, "                            : push_front bottom right at rc pos " << right_contour.size() );
            AddRightContour( nearest, originating );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: cur edge has " << constrain_br.size() << " br constraints - expecting 1" );                    
        }        
        SG_LOG( SG_GENERAL, SG_INFO, "                            : push_front bottom middle at rc pos " << right_contour.size() );
        AddRightContour( start->GetPosition(), originating );        
    } else {        
        if ( constrain_tl.size() == 1 ) {
            nearest = IntersectCorner( end->GetPosition(), constrain_tl, constrain_bl, 
                                       side_l, edge_needs_fixing, "TL", "BL", "LS" );
            
            // TEST TEST TEST: only fix edge if nearest is closer to start than end
            if ( TGEuclidean::distanceM( end->GetPosition(), nearest ) <
                 TGEuclidean::distanceM( start->GetPosition(), nearest ) ) {
                edge_needs_fixing = false;
            }

            SG_LOG( SG_GENERAL, SG_INFO, "                            : push_front top left at lc pos " << left_contour.size() );            
            AddLeftContour( nearest, originating );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: cur edge has " << constrain_tl.size() << " tl constraints - expecting 1" );                    
        }
        SG_LOG( SG_GENERAL, SG_INFO, "                            : push_front top middle at lc pos " << left_contour.size() );            
        AddLeftContour( end->GetPosition(), originating );

        if ( constrain_tr.size() == 1 ) {
            nearest = IntersectCorner( end->GetPosition(), constrain_tr, constrain_br, 
                                       side_r, dummy, "TR", "BR", "RS" );
            
            SG_LOG( SG_GENERAL, SG_INFO, "                            : push_back top right at rc pos " << right_contour.size() );
            AddRightContour( nearest, originating );
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "IntersectBisectRays: cur edge has " << constrain_tr.size() << " tr constraints - expecting 1" );                    
        }        
        SG_LOG( SG_GENERAL, SG_INFO, "                            : push_back top middle at rc pos " << right_contour.size() );
        AddRightContour( end->GetPosition(), originating );
    }
    
    return edge_needs_fixing;
}

std::list<SGGeod> tgIntersectionEdge::ClipContour( std::list<SGGeod>& contour, std::list<SGGeod>& constraint )
{
    char layer[64];
    
    // find where we start / end inserting the constraint
    std::list<SGGeod>::iterator         pos1, pos2;
    std::list<SGGeod>::reverse_iterator rpos1, rpos2;
    
    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: " << id << " original contour has " << contour.size() << " nodes, constraint has " << constraint.size() << " nodes" ); 
    
    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: build constraintStart" ); 

    // find starting insertion point by creating a ray from second point to first point
    pos1 = constraint.begin();
    pos2 = pos1++;
    // tgRay constraintStart( (*pos1), (*pos2) );
    // tgShapefile::FromRay( constraintStart, datasource, "contourConstraintStart", "ray" );
    tgLine constraintStart( (*pos1), (*pos2) );
    tgShapefile::FromLine( constraintStart, datasource, "contourConstraintStart", "line" );

    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: build constraintEnd" ); 
    
    // find ending insertion point by creating a ray from second point to first point
    rpos1 = constraint.rbegin();
    rpos2 = rpos1++;
    //tgRay constraintEnd( (*rpos1), (*rpos2) );    
    //tgShapefile::FromRay( constraintEnd, datasource, "contourConstraintEnd", "ray" );
    tgLine constraintEnd( (*rpos1), (*rpos2) );
    tgShapefile::FromLine( constraintEnd, datasource, "contourConstraintEnd", "line" );

    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: build segArray" ); 
    
    // build an array of tgSegments for each part of the current contour
    tgsegment_list segs;
    for ( pos1 = contour.begin(); pos1 != contour.end(); pos1++ ) {
        pos2 = pos1; pos2++;
        if ( pos2 != contour.end() ) {
            segs.push_back( tgSegment( (*pos1), (*pos2) ) );
        }
    }
    
    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: dump segArray" ); 

    // dump the segments
    for ( unsigned int i=0; i<segs.size(); i++ ) {
        sprintf(layer, "contourSeg_%d", i );
        tgShapefile::FromSegment( segs[i], datasource, layer, "seg" );
    }
    
    // find the first segment that intersects the start line
    std::list<SGGeod>::iterator iStart  = contour.end();
    std::list<SGGeod>::iterator iInsert = contour.end();
    SGGeod startPos;
    pos1 = contour.begin();
    pos1++; // we want to delete starting at the intersecting segment end pos.
    for ( unsigned int i=0; i<segs.size(); i++ ) {
        SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: intersect constraintStart and seg " << i ); 
        
        if ( constraintStart.Intersect( segs[i], startPos ) ) {
            SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: segment " << i << " intersects start - Set start iterator to pt " << i+1 );
            iStart = pos1;
            break;
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: segment " << i << " does not intersect ");  
        }
        pos1++;
    }
    
    // find the last segment that intersects the end line
    std::list<SGGeod>::iterator iEnd = contour.end();
    SGGeod endPos;
    pos1 = contour.end();
    pos1--;
    for ( int i=segs.size()-1; i>= 0; i-- ) {
        SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: intersect constraintEnd and seg " << i ); 

        if ( constraintEnd.Intersect( segs[i], endPos ) ) {
            SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: segment " << i << " intersects end - Set end iterator to pt " << i );  
            iEnd = pos1;
            break;
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: segment " << i << " does not intersect ");  
        }
        pos1--;
    }

    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: erase (nodes before) " << contour.size() ); 

    // delete the contour points inside of these intersection points
    iInsert = contour.erase( iStart, iEnd );
    contour.insert( iInsert, constraint.begin(), constraint.end() );

    segs.clear();
    for ( pos1 = contour.begin(); pos1 != contour.end(); pos1++ ) {
        pos2 = pos1; pos2++;
        if ( pos2 != contour.end() ) {
            segs.push_back( tgSegment( (*pos1), (*pos2) ) );
        }
    }
    
    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: dump segArray" ); 

    // dump the segments
    for ( unsigned int i=0; i<segs.size(); i++ ) {
        sprintf(layer, "contourSegAfter_%d", i );
        tgShapefile::FromSegment( segs[i], datasource, layer, "seg" );
    }

    SG_LOG(SG_GENERAL, SG_INFO, "ClipContour: complete" ); 
    
    return contour;
}

void tgIntersectionEdge::AddLeftContour( const SGGeod& pt, bool originating )
{
    tgShapefile::FromGeod( pt, datasource, "LeftContour", "pt" );

    // add the new node
    if ( originating ) {
        if ( left_contour.empty() ) {
            left_contour.push_back( pt );
        } else if (!SGGeod_isEqual2D(pt, left_contour.back())) {
            left_contour.push_back( pt );
        }
    } else {
        if ( left_contour.empty() ) {
            left_contour.push_front( pt );
        } else if (!SGGeod_isEqual2D(pt, left_contour.front())) {
            left_contour.push_front( pt );
        }
    }    
}

void tgIntersectionEdge::AddRightContour( const SGGeod& pt, bool originating )
{
    tgShapefile::FromGeod( pt, datasource, "LeftContour", "pt" );

    // add the new node
    if ( originating ) {
        if ( right_contour.empty() ) {
            right_contour.push_front( pt );
        } else if (!SGGeod_isEqual2D(pt, right_contour.front())) {
            right_contour.push_front( pt );
        }
    } else {
        if ( right_contour.empty() ) {
            right_contour.push_back( pt );
        } else if (!SGGeod_isEqual2D(pt, right_contour.back())) {
            right_contour.push_back( pt );
        }
    }    
}

void tgIntersectionEdge::AddLeftSegmentConstraint( const SGGeod& pt, bool originating, bool complete )
{
    tgShapefile::FromGeod( pt, datasource, "LeftSegConst", "pt" );

    // add the new node
    if ( originating ) {
        if ( constrain_msl.empty() ) {
            constrain_msl.push_back( pt );
        } else if (!SGGeod_isEqual2D(pt, constrain_msl.back())) {
            constrain_msl.push_back( pt );
        }
        
        if ( complete ) {
            left_contour = ClipContour( left_contour, constrain_msl );
        }
    } else {
        if ( constrain_msr.empty() ) {
            constrain_msr.push_front( pt );
        } else if (!SGGeod_isEqual2D(pt, constrain_msr.front())) {
            constrain_msr.push_front( pt );
        }

        if ( complete ) {
            right_contour = ClipContour( right_contour, constrain_msr );
        }
    }    
}

void tgIntersectionEdge::AddRightSegmentConstraint( const SGGeod& pt, bool originating, bool complete )
{
    tgShapefile::FromGeod( pt, datasource, "RightSegConst", "pt" );

    // add the new node
    if ( originating ) {
        if ( constrain_msr.empty() ) {
            constrain_msr.push_back( pt );
        } else if (!SGGeod_isEqual2D(pt, constrain_msr.back())) {
            constrain_msr.push_back( pt );
        }
        
        if ( complete ) {
            right_contour = ClipContour( right_contour, constrain_msr );
        }
    } else {
        if ( constrain_msl.empty() ) {
            constrain_msl.push_front( pt );
        } else if (!SGGeod_isEqual2D(pt, constrain_msl.front())) {
            constrain_msl.push_front( pt );
        }

        if ( complete ) {
            left_contour = ClipContour( left_contour, constrain_msl );
        }
    }
}

tgPolygon tgIntersectionEdge::CreatePolygon( int& id, double& heading, double& dist, double& w, SGGeod& tref, const char* debug_layer )
{
    // just return a poly with the four nodes
    tgPolygon poly;
    
    poly.AddNode( 0, botLeft );
    poly.AddNode( 0, botRight );
    poly.AddNode( 0, topRight );
    poly.AddNode( 0, topLeft );

    heading      = SGGeodesy::courseDeg( start->GetPosition(), end->GetPosition() );
    dist         = SGGeodesy::distanceM( start->GetPosition(), end->GetPosition() );
    id           = type;
    w            = width;
    tref         = botLeft;    
    
    return poly;
}

tgPolygon tgIntersectionEdge::GetPoly(const char* prefix)
{
    char layer[64];
    int  idx;
    std::list<SGGeod>::iterator i;
    tgPolygon    poly;

    SG_LOG(SG_GENERAL, SG_INFO, "tgIntersectionEdge::GetPoly: rc size " << right_contour.size() << " lc size " << left_contour.size() );
    
    for ( i = right_contour.begin(), idx = 1; i != right_contour.end(); i++, idx++) {
        sprintf(layer, "%s_GP_rc_%d", prefix, idx );
        tgShapefile::FromGeod( *i, datasource, layer, "pt" );
        poly.AddNode( 0, *i );
    }
    
    for ( i = left_contour.begin(), idx = 1; i != left_contour.end(); i++, idx++) {
        sprintf(layer, "%s_GP_lc_%d", prefix, idx );
        tgShapefile::FromGeod( *i, datasource, layer, "pt" );
        poly.AddNode( 0, *i );
    }

    // poly = tgPolygon::RemoveDups( poly );
    
    return poly;
}

tgIntersectionEdgeInfo::tgIntersectionEdgeInfo( bool orig, tgIntersectionEdge* e ) 
{    
    edge         = e;
    originating  = orig;
    multiSegment = false;
    
    if ( originating ) {
        heading = TGEuclidean::courseDeg( edge->start->GetPosition(), edge->end->GetPosition() );
    } else {
        heading = TGEuclidean::courseDeg( edge->end->GetPosition(), edge->start->GetPosition() );
    }
}