#include <simgear/sg_inlines.h>

#include "tg_polygon.hxx"
#include "tg_shapefile.hxx" 
#include "tg_intersection_node.hxx"
#include "tg_intersection_edge.hxx"
#include "tg_misc.hxx"

tgIntersectionNode::tgIntersectionNode( const SGGeod& pos )
{
    static unsigned int cur_id = 1;
    
    position = pos;
    position2 = edgeArrPoint( pos.getLongitudeDeg(), pos.getLatitudeDeg() );
    
    edgeList.clear();
    start_v = NODE_UNTEXTURED;
    endpoint = false;
    id = cur_id++;
}

tgIntersectionNode::tgIntersectionNode( const edgeArrPoint& pos )
{
    static unsigned int cur_id = 1;
    
    position = SGGeod::fromDeg( CGAL::to_double( pos.x() ), CGAL::to_double( pos.y() ) );
    position2 = pos;
    
    edgeList.clear();
    start_v = NODE_UNTEXTURED;
    endpoint = false;
    id = cur_id++;
}

void tgIntersectionNode::CheckEndpoint( void )
{
    tgintersectionedgeinfo_it cur;

    // any intersection is an endpoint 
    if ( edgeList.size() > 2 ) {
        endpoint = true;
    }
    
    // caps are NOT endpoints - but its neighbor is
    if ( !endpoint ) {
        for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
            // check if the other node of this edge is a cap
            if ( (*cur)->IsOriginating() ) {
                if ( (*cur)->GetEdge()->end->IsCap() ) {
                    endpoint = true;
                }
            } else {
                if ( (*cur)->GetEdge()->start->IsCap() ) {
                    endpoint = true;
                }
            }
        }
    }
    
    // only other way to have an endpoint is with two edges with different 
    // textures
    if ( !endpoint ) {
        if ( edgeList.size() == 2 ) {
            unsigned int texture;
            
            cur = edgeList.begin();
            texture = (*cur)->GetEdge()->type;
            
            cur++;
            if ( (*cur)->GetEdge()->type != texture ) {
                endpoint = true;
            }
        }
    }
}

void tgIntersectionNode::AddEdge( bool originated, tgIntersectionEdge* edge ) 
{
    edgeList.insert( new tgIntersectionEdgeInfo( originated, edge ) );
}

void tgIntersectionNode::DelEdge( bool originated, tgIntersectionEdge* edge ) 
{
    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::DelEdge: enter - node has " << edgeList.size() << " edges " );
    
    for (tgintersectionedgeinfo_it cur = edgeList.begin(); cur != edgeList.end(); cur++) {
        // edge info is unique from node to node - but the edge itself is shared
        if( (*cur)->GetEdge() == edge ) {
            // we found the current edge info - delete it
            edgeList.erase(cur);
            break;
        }
    }

    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::DelEdge: exit - node has " << edgeList.size() << " edges ");    
}

tgIntersectionEdgeInfo* tgIntersectionNode::GetPrevEdgeInfo( tgIntersectionEdgeInfo* cur_info, const tgConstraint& bisector, const edgeArrPoint& bisect_pos, const char* prefix )
{
    tgintersectionedgeinfo_it prv, cur;
    tgIntersectionEdgeInfo*   prv_info = NULL;
    const tgIntersectionEdge* ce       = cur_info->GetEdge();
    unsigned int              ce_id    = ce->id;
    
    if ( edgeList.size() > 1 ) {
        for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
            // edge info is unique from node to node - but the edge itself is shared
            if( (*cur)->GetEdge() == ce ) {
                // we found the current edge info - decrement to prv
                if ( cur != edgeList.begin() ) {
                    prv = cur;
                    prv--;
                } else {
                    prv = edgeList.end();
                    prv--;
                }
            }
        }
        
        // now, loop around until we find an edge where the top/bottom intersects the bisector at the same position
        while ( (!prv_info) && ((*prv)->GetEdge()->id != ce_id) ) {
            bool ve_originating    = (*prv)->IsOriginating();
            tgIntersectionEdge* ve = (*prv)->GetEdge();

            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::GetPrevEdgeInfo: ce " << ce_id << " check pe " << (*prv)->GetEdge()->id );
            
            // bisect the shared constraint to verify
            if ( !prv_info ) {
                if ( ve->VerifyIntersectionLocation( ve_originating, bisector, bisect_pos ) ) {
                    prv_info = (*prv);
                } else {
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::GetPrevEdgeInfo: bisector does not intersect at same loc" );
                }
            }
            
            if ( prv != edgeList.begin() ) {
                prv--;
            } else {
                prv = edgeList.end();
                prv--;
            }            
        }                                    
    }
    
    return prv_info;
}

tgIntersectionEdgeInfo* tgIntersectionNode::GetNextEdgeInfo( tgIntersectionEdge* cur_edge )
{
    tgintersectionedgeinfo_it cur, nxt;
    tgIntersectionEdgeInfo* nxt_info = NULL;
    
    if ( edgeList.size() > 1 ) {
        for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
            // edge info is unique from node to node - but the edge itself is shared
            if( (*cur)->GetEdge() == cur_edge ) {
                // we found the current edge info - increment to next
                nxt = cur;
                nxt++;

                if ( nxt == edgeList.end() ) {
                    nxt = edgeList.begin();
                }
                
                nxt_info = (*nxt);
            }
        }
    } else {
        nxt_info = NULL;
    }
    
    return nxt_info;
}

tgIntersectionEdgeInfo* tgIntersectionNode::GetNextEdgeInfo( tgIntersectionEdgeInfo* cur_info, const tgConstraint& bisector, const edgeArrPoint& bisect_pos, const char* prefix )
{
    tgintersectionedgeinfo_it cur, nxt;
    tgIntersectionEdgeInfo*   nxt_info = NULL;
    const tgIntersectionEdge* ce       = cur_info->GetEdge();
    unsigned int              ce_id    = ce->id;
    
    SG_LOG( SG_GENERAL, SG_INFO, "GetNextEdgeInfo: cur_edge " << cur_info->GetEdge()->id );
    if ( edgeList.size() > 1 ) {
        for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
            // edge info is unique from node to node - but the edge itself is shared
            if( (*cur)->GetEdge() == ce ) {
                // we found the current edge info - increment to next
                nxt = cur;
                nxt++;

                if ( nxt == edgeList.end() ) {
                    nxt = edgeList.begin();
                }
            }
        }

        // now, loop around until we find an edge where the top/bottom intersects the bisector at the same position
        while ( (!nxt_info) && ((*nxt)->GetEdge()->id != ce_id) ) {
            SG_LOG( SG_GENERAL, SG_INFO, "GetNextEdgeInfo: nxt " << (*nxt)->GetEdge()->id << " cur " << ce_id );
            
            bool ve_originating    = (*nxt)->IsOriginating();
            tgIntersectionEdge* ve = (*nxt)->GetEdge();
            
            // bisect the shared constraint to verify
            if ( !nxt_info ) {
                if ( ve->VerifyIntersectionLocation( ve_originating, bisector, bisect_pos ) ) {
                    nxt_info = (*nxt);
                }
            }

            nxt++;
            if ( nxt == edgeList.end() ) {
                nxt = edgeList.begin();
            }        
        }            
    }
    
    return nxt_info;
}

double tgIntersectionNode::CalcDistanceToNextEndpoint( tgIntersectionEdgeInfo* cur_info, unsigned int& num_edges )
{
    double total_dist = 0.0f;
    tgIntersectionNode* next_node = NULL;
    tgIntersectionEdge* cur_edge = NULL;
    bool done = false;

    do {
        cur_edge = cur_info->GetEdge();        
        total_dist += cur_edge->GetGeodesyLength();
        num_edges++;
        
        if ( cur_info->IsOriginating() ) {
            next_node = cur_edge->end;
        } else {
            next_node = cur_edge->start;
        }

        if ( next_node == this ) {
            SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::CalcDistanceToNextEndpoint ERROR : We looped around ");
            done = true;
        }
        
        if ( next_node->IsEndpoint() ) {
            done = true;
        } else {
            cur_info = next_node->GetNextEdgeInfo( cur_edge );            
        }
        
        if ( !cur_info ) {
            SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::CalcDistanceToNextEndpoint ERROR: getnextInfo returned NULL" );
            done = true;
        }
        
    } while (!done);
    
    return total_dist;
}

void tgIntersectionNode::TextureToNextEndpoint( tgIntersectionEdgeInfo* cur_info, tgIntersectionGeneratorTexInfoCb texInfoCb, double ratio )
{
    double start_v = 0.0f;
    
    tgIntersectionNode* next_node = NULL;
    tgIntersectionEdge* cur_edge = NULL;
    bool done = false;

    do {
        cur_edge = cur_info->GetEdge();        
        start_v = cur_info->Texture(start_v, texInfoCb, ratio);
        
        if ( cur_info->IsOriginating() ) {
            next_node = cur_edge->end;
        } else {
            next_node = cur_edge->start;
        }

        if ( next_node == this ) {
           SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureToNextEndpoint ERROR : We looped around ");
           done = true;
        }
        
        if ( next_node->IsEndpoint() ) {
            SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureToNextEndpoint FINISHED - start_v is " << start_v);
            done = true;
        } else {
            cur_info = next_node->GetNextEdgeInfo( cur_edge );
        }

        if ( !cur_info ) {
            SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::CalcDistanceToNextEndpoint ERROR: getnextInfo returned NULL" );
            done = true;
        }
        
    } while (!done);
}

void tgIntersectionNode::TextureEdges( tgIntersectionGeneratorTexInfoCb texInfoCb ) 
{
    if (!endpoint) {
        return;
    }
    
    SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges node has " << edgeList.size() << " edges" );
    
    int i = 1;
    for (tgintersectionedgeinfo_it cur = edgeList.begin(); cur != edgeList.end(); cur++) {    
        SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges edge " << i++ );
        
        if (*cur) {
            // check for start CAP
            if ( (*cur)->IsStartCap() ) {
                SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges found Start CAP " );
                (*cur)->TextureStartCap(texInfoCb);
            } else if ( (*cur)->IsEndCap() ) {
                SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges found End CAP " );
                (*cur)->TextureEndCap(texInfoCb);
            } else if ( !(*cur)->GetEdge()->IsTextured() ) {
                unsigned int num_edges = 0;
                
                SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges found non cap " );
                
                // we need to get the total distance between this, and the next endpoint
                // we know that each node from here to the next endpoint is degree 2
                // ( but we'll check just to be sure )
                double total_dist = CalcDistanceToNextEndpoint( (*cur), num_edges );
                
                // now calculate compression factor - need to get the repeating v length for this
                unsigned int type = (*cur)->GetEdge()->type;
                
                std::string material;
                double      texAtlasStartU, texAtlasEndU;
                double      texAtlasStartV, texAtlasEndV;
                double      v_dist;
    
                // Get the v_dist for this texture
                texInfoCb( type, false, material, texAtlasStartU, texAtlasEndU, texAtlasStartV, texAtlasEndV, v_dist );

                // get remainder
                double frac = fmod( total_dist, v_dist ) / v_dist;
                bool stretch = false;
                int num_sections;
                if ( (0.0 <= frac) &&  (frac <= 0.5) ) {
                    // we should use less sections and stretch
                    num_sections = total_dist / v_dist;
                    stretch = true;    
                } else {
                    // we should use 1 more section and shrink
                    num_sections = (total_dist / v_dist) + 1;
                }
                
                double section_length = total_dist / num_sections;
                double ratio = section_length / v_dist;
                
                if ( stretch ) {
                    SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges found " << num_edges << " with total_dist of " << total_dist << " texture dist is " << v_dist << " stretch_ratio " << ratio );
                } else {
                    SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges found " << num_edges << " with total_dist of " << total_dist << " texture dist is " << v_dist << " shrink_ratio " << ratio );
                }
                
                TextureToNextEndpoint( (*cur), texInfoCb, ratio );
            } else {
                SG_LOG(SG_GENERAL, LOG_TEXTURE, "tgIntersectionNode::TextureEdges skip already textured edge " );
            }                
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::TextureEdges found NULL pointer in edge list");
        }
    }
}

void tgIntersectionNode::AddCapEdges( tgIntersectionNodeList& nodelist, tgintersectionedge_list& edgelist ) 
{
    // if this node has just one edge, we may need to add a cap edge
    if ( edgeList.size() == 1 ) {
        tgintersectionedgeinfo_it cur = edgeList.begin();

        // we need to get the edge info to determine the edge origination info
        tgIntersectionEdgeInfo* cur_info = (*cur);
        
#if 0        
        //double cur_heading = cur_info->GetHeading();
        double cur_heading = cur_info->GetGeodesyHeading();
        
        tgIntersectionEdge* cur_edge = cur_info->GetEdge();
        double cur_length = cur_edge->GetGeodesyLength();
        
        // euclidean distance is in degrees :( - TODO - need distanceM and distanceE
        if ( cur_length > 1.0 ) {
            // Add a new node 0.5 M away
            tgIntersectionNode* newNode = nodelist.Add( SGGeodesy::direct( position, cur_heading, 0.5 ) );
            tgIntersectionEdge* newEdge = cur_edge->Split( cur_info->IsOriginating(), newNode );
            edgelist.push_back( newEdge );
        }
#else
        tgIntersectionEdge* cur_edge = cur_info->GetEdge();

        // to split, we need to use an exact position for the new node. ( not doubles )
        // we use side intersections to determine where to direct the constraints.
        // if the new edge is just nearly paralell to the old, we can get strange intersection locations
        CGAL::Vector_2<edgeArrKernel> vec = cur_edge->ToVector();

        // get the length of the vector
        edgeArrKernel::FT length = sqrt( CGAL::to_double(vec.squared_length()) );

        SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::AddCapEdges: current edge " << cur_edge->id << " length is " << length << "(deg) " << length*111000 << "(meters)");
        
        // approx meters
        if ( (length * 111000) > 1.0 ) {
            // we want to have a translate unit vector
            vec = vec / length;
            vec = vec / 222000;
            
            edgeArrKernel::FT length2 = sqrt( CGAL::to_double(vec.squared_length()) );
            SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::AddCapEdges: new edge length is " << length2 << "(deg) " << length2*111000 << "(meters)");
            
            if (!cur_info->IsOriginating() ) {
                vec = -vec;
            }
            edgeArrTransformation translate(CGAL::TRANSLATION, vec);
            edgeArrPoint splitPt  = translate(position2);
            
            tgIntersectionNode* newNode = nodelist.Add( splitPt );
            tgIntersectionEdge* newEdge = cur_edge->Split( cur_info->IsOriginating(), newNode );
            edgelist.push_back( newEdge );
        }
        else
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::AddCapEdges: current edge " << cur_edge->id << " length is " << length*111000 );
        }
#endif
    }
}

void tgIntersectionNode::ConstrainEdges( void ) 
{
    tgintersectionedgeinfo_it cur, prev;
    
    switch( edgeList.size() ) {
        case 1:
            // add cap
            GenerateCapRays();
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
    // traverse the constraints from the edge start node
    // note that the start node CAN be outside the final shape.
    // This only happens when a segment is thin, and does not fully 
    // reach the intersection node.  We know where the new
    // start is in this case...
    tgintersectionedgeinfo_it cur;
    
    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
        if ((*cur)->IsOriginating() ) {
            (*cur)->GetEdge()->Generate();
        }
    }
    switch( edgeList.size() ) {
        case 1:
            // add cap
            IntersectBisectRays();
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

void tgIntersectionNode::CompleteSpecialIntersections( void )
{
    tgintersectionedgeinfo_it prv, cur, nxt;
    
    switch( edgeList.size() ) {
        //case 1:
        //    // TODO : is it possible for caps to participate in ms intersections?
        //    // looks like sharp edge could cause odd geometry here.
        //    CompleteCap( (*edgeList.begin()) );
        //   break;
            
        default:
            for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {
                nxt = cur;
                nxt++;
                if ( nxt == edgeList.end() ) {
                    nxt = edgeList.begin();
                }

                CompleteMultiSegmentIntersections( (*cur), (*nxt) );
            }
            break;
    }
}


inline SGGeod CalculateLinearLocation( const SGGeod& p0, const SGGeod& p1, const double t )
{
    // these are 2d approximations using lon, and lat as x,y cartesion coords
    // how expensive would this be to do in Geodetic math?
    SGVec2d v0 = SGVec2d( p0.getLongitudeDeg(), p0.getLatitudeDeg() );
    SGVec2d v1 = SGVec2d( p1.getLongitudeDeg(), p1.getLatitudeDeg() );
    SGVec2d result;

    // we do this - backwards. if t=1, we want p0
    double term1 = t;
    double term2 = (1.0f - t);

    result = (v0 * term1) + (v1 * term2);

    return SGGeod::fromDeg( result.x(), result.y() );
}

bool tgIntersectionNode::IntersectCurRightSideWithNextLeftSide( tgIntersectionEdgeInfo* cur_info, tgIntersectionEdgeInfo* nxt_info, edgeArrPoint& intersectionLocation )
{
    bool intersects = false;
    
    tgIntersectionEdge* cur_edge = cur_info->GetEdge();
    tgIntersectionEdge* nxt_edge = nxt_info->GetEdge();

    edgeArrLine curRight = cur_edge->GetRightSide( cur_info->IsOriginating() );
    edgeArrLine nxtLeft  = nxt_edge->GetLeftSide( nxt_info->IsOriginating() );

    // special case if the lines are paralell
    if ( CGAL::parallel(curRight, nxtLeft) ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::IntersectCurRightSideWithNextLeftSide: current edge " << cur_edge->id << " is paralell with " << nxt_edge->id );
        // generate perpendicular ray, and intersect with side
        CGAL::Vector_2<edgeArrKernel> vec = cur_edge->ToVector();
        if ( !cur_info->IsOriginating() ) {
            vec = -vec;
        }        
        CGAL::Vector_2<edgeArrKernel> perp = vec.perpendicular( CGAL::CLOCKWISE );
        
        // translate nodes position by vector to get 2nd point
        edgeArrTransformation translateRight(CGAL::TRANSLATION, perp);
        edgeArrPoint target = translateRight( position2 );
        edgeArrLine  bisector( position2, target );
        
        CGAL::Object result;
        result = CGAL::intersection( curRight, bisector );
        if (const edgeArrPoint *ipoint = CGAL::object_cast<edgeArrPoint>(&result)) {
            // handle the point intersection case with *ipoint.
            intersectionLocation = *ipoint;
            intersects = true;
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::IntersectCurRightSideWithNextLeftSide: current edge " << cur_edge->id << " PARALELL but CAN'T INTERSECT " << nxt_edge->id );                            
        }
        
    } else {    
        CGAL::Object result;
        result = CGAL::intersection( curRight, nxtLeft );
        if (const edgeArrPoint *ipoint = CGAL::object_cast<edgeArrPoint>(&result)) {
            // handle the point intersection case with *ipoint.
            intersectionLocation = *ipoint;
            intersects = true;
            if ( ( cur_edge->id == 1 ) && (nxt_edge->id == 58 ) ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::IntersectCurRightSideWithNextLeftSide: current edge " << cur_edge->id << " REALLY INTERSECTS with " << nxt_edge->id );                
            }
        }
    }
    
    return intersects;
}

// we only use bisector rays as a helper with multisegment intersections.
void tgIntersectionNode::GenerateBisectRays( void ) 
{
    tgintersectionedgeinfo_it cur, nxt;

#if DEBUG_INTERSECTIONS
    char layer[128];
#endif
    
    sgDebugPriority LOG_NODE_DBG = SG_DEBUG;
    
    edgeArrPoint eaPosition( position.getLongitudeDeg(), position.getLatitudeDeg() );
    
    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) {        
        // get next edge info - with wrap around
        nxt = cur; nxt++;
        if ( nxt == edgeList.end() ) {
            nxt = edgeList.begin();
        }
        
        // we need to get the edge info to determine the edge origination info
        tgIntersectionEdgeInfo* cur_info = (*cur);
        tgIntersectionEdgeInfo* nxt_info = (*nxt);
        
        tgIntersectionEdge*     cur_edge = cur_info->GetEdge();
        tgIntersectionEdge*     nxt_edge = nxt_info->GetEdge();
        
        //double cur_heading;
        //double nxt_heading;
        //double bisect_heading;
        
        // the edge info always has heading with respect to the node
        //cur_heading = cur_info->GetHeading();
        //nxt_heading = nxt_info->GetHeading();
                
        // we parametize the thinner segment.  The origin of the secondary
        // constraint is the ration 0..1 of the thin segment width / thick 
        // segment width
        double cur_width = cur_edge->width;
        double nxt_width = nxt_edge->width;
        char constraint_desc[128];

        SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays: current edge is " << cur_edge->id << " width " << cur_width );
        SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays:    next edge is " << nxt_edge->id << " width " << nxt_width );
        
        if ( cur_width == nxt_width ) {
            // instead of using bisect_heading - find the intersection of the two lines making up cur edge right side, and next edge left side
            //tgLine curRight = cur_edge->GetRightSide( cur_info->IsOriginating() );
            // edgeArrLine curRight = cur_edge->GetRightSide( cur_info->IsOriginating() );
            // tgLine nxtLeft  = nxt_edge->GetLeftSide( nxt_info->IsOriginating() );
            // edgeArrLine nxtLeft = nxt_edge->GetLeftSide( nxt_info->IsOriginating() );
            edgeArrPoint rayIntersect;
            if ( IntersectCurRightSideWithNextLeftSide( cur_info, nxt_info, rayIntersect ) ) {
            
                // debug the intersect point... why is it different below...
                char desc[128];
                sprintf( desc, "intersection_%06ld_rightside_and_%06ld_leftside", cur_edge->id, nxt_edge->id );
                cur_edge->AddDebugPoint( rayIntersect, desc );
            
                if ( cur_info->IsOriginating() ) {
                    sprintf(constraint_desc, "BR_%ld_equal_%ld_position_via_bisector", cur_edge->id, nxt_edge->id );                
                    cur_edge->AddConstraint( BOT_RIGHT_CONSTRAINT, tgConstraint::fromRay(eaPosition, rayIntersect, nxt_edge->id, constraint_desc) );
                } else {
                    sprintf(constraint_desc, "TL_%ld_equal_%ld_position_via_bisector", cur_edge->id, nxt_edge->id );                
                    cur_edge->AddConstraint( TOP_LEFT_CONSTRAINT, tgConstraint::fromRay(eaPosition, rayIntersect, nxt_edge->id, constraint_desc) );
                }
            
                if ( nxt_info->IsOriginating() ) {                    
                    sprintf(constraint_desc, "BL_%ld_equal_%ld_position_via_bistector", nxt_edge->id, cur_edge->id );
                    nxt_edge->AddConstraint( BOT_LEFT_CONSTRAINT, tgConstraint::fromRay(eaPosition, rayIntersect, cur_edge->id, constraint_desc) );
                } else {
                    sprintf(constraint_desc, "TR_%ld_equal_%ld_position_via_bistector", nxt_edge->id, cur_edge->id );
                    nxt_edge->AddConstraint( TOP_RIGHT_CONSTRAINT, tgConstraint::fromRay(eaPosition, rayIntersect, cur_edge->id, constraint_desc) );
                }
            } else {
                SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays: ERROR - no intersect between " << cur_edge->id << " and " << nxt_edge->id );
            }
        } else {
#if 0            
            // we have a more complex intersection.
            // calculate the bisect ratio to determine which edge we will use 
            // to offset the bisector rays
            double bisect_ratio = cur_width / nxt_width;
            bool valid = false;
            
            SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays: bisect_ratio" << bisect_ratio );
            SGGeod leftIntersection, rightIntersection, offsetOrigin;
       
            if ( bisect_ratio > 1.0 ) {
                // the ratio is greater than 1.  The current edge is the thicker of the two.
                // we want to find where the thin edge centerline intersects with
                // both this current edge's right side, and the left side of the edge after the next edge
              
                tgintersectionedgeinfo_it nxt_nxt = nxt; nxt_nxt++;
                if ( nxt_nxt == edgeList.end() ) {
                    nxt_nxt = edgeList.begin();
                }

                tgIntersectionEdgeInfo* nxt_nxt_info = (*nxt_nxt);        
                tgIntersectionEdge*     nxt_nxt_edge = nxt_nxt_info->GetEdge();
                double                  nxt_nxt_heading = nxt_nxt_info->GetHeading();
                
                // right and left intersection points are reletive to the thin ( next ) edge
                if ( nxt_edge->ToLine().Intersect( cur_edge->GetRightSide( cur_info->IsOriginating() ), rightIntersection ) ) {
                    if ( nxt_edge->ToLine().Intersect( nxt_nxt_edge->GetLeftSide( nxt_nxt_info->IsOriginating() ), leftIntersection ) ) {
                        // both intersection successfull - which is closer to node position?
                        double rightDist = SGGeodesy::distanceM( position, rightIntersection );
                        double leftDist  = SGGeodesy::distanceM( position, leftIntersection );
                        
                        SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays1: rightDist " << rightDist );
                        SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays1: leftDist " << leftDist );
                        
                        if ( rightDist <= leftDist ) {
                            SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays1: use right intersection" );
                            
                            // use the right ( current ) intersection to interpolate
                            offsetOrigin = CalculateLinearLocation( position, rightIntersection, 1/bisect_ratio );
                            bisect_heading = Bisect( position, nxt_heading, cur_heading, true );
                            
                        } else {
                            SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays1: use left intersection" );
                        
                            // use the left ( next next ) intersection to interpolate                            
                            offsetOrigin = CalculateLinearLocation( position, leftIntersection, 1/bisect_ratio );
                            bisect_heading = SGMiscd::normalizePeriodic( 0, 360, Bisect( position, nxt_nxt_heading, nxt_heading, true ) );
                        }
                        
                        valid = true;
                    } else {
                        SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::GenerateBisectRays: no intersection between next ( thin ) edge and next next left side. node deg is " << Degree() );
                    }
                } else {
                    SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::GenerateBisectRays: no intersection between next ( thin ) edge and cur right side" );                    
                }
            
                if ( valid ) {
                    // for the current (thick) edge, we have a segment and ray
                    if ( cur_info->IsOriginating() ) {
                        sprintf(constraint_desc, "BR_%ld_thick_%ld_pos_to_origin", cur_edge->id, nxt_edge->id );
                        cur_edge->AddConstraint( BOT_RIGHT_CONSTRAINT, tgConstraint::fromSegment(position, offsetOrigin, nxt_edge->id, constraint_desc) );
                    
                        sprintf(constraint_desc, "BR_%ld_thick_%ld_origin_via_bisector", cur_edge->id, nxt_edge->id );
                        cur_edge->AddConstraint( BOT_RIGHT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, SGMiscd::normalizePeriodic( 0, 360, bisect_heading ), nxt_edge->id, constraint_desc) );
                    } else {
                        sprintf(constraint_desc, "TL_%ld_thick_%ld_pos_to_origin", cur_edge->id, nxt_edge->id );
                        cur_edge->AddConstraint( TOP_LEFT_CONSTRAINT, tgConstraint::fromSegment(position, offsetOrigin, nxt_edge->id, constraint_desc) );
                    
                        sprintf(constraint_desc, "TL_%ld_thick_%ld_origin_via_bisector", cur_edge->id, nxt_edge->id );
                        cur_edge->AddConstraint( TOP_LEFT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, SGMiscd::normalizePeriodic( 0, 360, bisect_heading ), nxt_edge->id, constraint_desc) );
                    }
                
                    // for the next (thin) edge, we just have a ray
                    if ( nxt_info->IsOriginating() ) {
                        sprintf(constraint_desc, "BL_%ld_thin_%ld_origin_via_bisector", nxt_edge->id, cur_edge->id );
                        nxt_edge->AddConstraint( BOT_LEFT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, SGMiscd::normalizePeriodic( 0, 360, bisect_heading ), cur_edge->id, constraint_desc) );
                    } else {
                        sprintf(constraint_desc, "TR_%ld_thin_%ld_origin_via_bisector", nxt_edge->id, cur_edge->id );
                        nxt_edge->AddConstraint( TOP_RIGHT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, SGMiscd::normalizePeriodic( 0, 360, bisect_heading ), cur_edge->id, constraint_desc) );
                    }
                }
            } else {
                // the ratio is less than 1.  The current edge is the thinner of the two.
                // we want to find where the thin edge centerline intersects with
                // both the previous edge's right side, and the left side of the next edge
                tgintersectionedgeinfo_it prv;

                if ( cur == edgeList.begin() ) {
                    prv = edgeList.end();
                } else {
                    prv = cur;
                }
                prv--;

                tgIntersectionEdgeInfo* prv_info = (*prv);        
                tgIntersectionEdge*     prv_edge = prv_info->GetEdge();
                double                  prv_heading = prv_info->GetHeading();
                
                // right and left intersection points are reletive to the thin ( next ) edge
                if ( cur_edge->ToLine().Intersect( prv_edge->GetRightSide( prv_info->IsOriginating() ), rightIntersection ) ) {
                    if ( cur_edge->ToLine().Intersect( nxt_edge->GetLeftSide( nxt_info->IsOriginating() ), leftIntersection ) ) {
                        // both intersection successfull - which is closer to node position?
                        double rightDist = SGGeodesy::distanceM( position, rightIntersection );
                        double leftDist  = SGGeodesy::distanceM( position, leftIntersection );

                        SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays2: rightDist " << rightDist );
                        SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays2: leftDist " << leftDist );
                        
                        if ( rightDist <= leftDist ) {
                            SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays2: use right intersection" );

                            // use the right ( prv ) intersection to interpolate
                            offsetOrigin = CalculateLinearLocation( position, rightIntersection, bisect_ratio );
                            //bisect_heading = SGMiscd::normalizePeriodic( 0, 360, Bisect( position, cur_heading, prv_heading, true ) + 90 );
                            bisect_heading = SGMiscd::normalizePeriodic( 0, 360, Bisect( position, cur_heading, prv_heading, true ) );
                        } else {
                            SG_LOG(SG_GENERAL, LOG_NODE_DBG, "tgIntersectionNode::GenerateBisectRays2: use left intersection" );

                            // use the left ( nxt ) intersection to interpolate
                            offsetOrigin = CalculateLinearLocation( position, leftIntersection, bisect_ratio );
                            bisect_heading = Bisect( position, nxt_heading, cur_heading, true );
                        }
                        
                        valid = true;
                    } else {
                        SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::GenerateBisectRays: no intersection between cur ( thin ) edge and next left side" );
                    }
                } else {
                    SG_LOG(SG_GENERAL, SG_ALERT, "tgIntersectionNode::GenerateBisectRays: no intersection between cur ( thin ) edge and prev right side" );                    
                }
                 
                if ( valid ) {
                    // for the current (thin) edge, we just have a ray originating from 
                    // the calculated origin
                    if ( cur_info->IsOriginating() ) {
                        sprintf(constraint_desc, "BR_%ld_thin_%ld_origin_via_bisector", cur_edge->id, nxt_edge->id );
                        cur_edge->AddConstraint( BOT_RIGHT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, bisect_heading, nxt_edge->id, constraint_desc) );                        
                    } else {
                        sprintf(constraint_desc, "TL_%ld_thin_%ld_origin_via_bisector", cur_edge->id, nxt_edge->id );
                        cur_edge->AddConstraint( TOP_LEFT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, bisect_heading, nxt_edge->id, constraint_desc) );                        
                    }            

                    // for the thick edge, we have the constraint from the node position to origin, then a ray
                    if ( nxt_info->IsOriginating() ) {
                        sprintf(constraint_desc, "BL_%ld_thick_%ld_pos_to_origin", nxt_edge->id, cur_edge->id );
                        nxt_edge->AddConstraint( BOT_LEFT_CONSTRAINT, tgConstraint::fromSegment(position, offsetOrigin, cur_edge->id, constraint_desc) );

                        sprintf(constraint_desc, "BL_%ld_thick_%ld_origin_via_bisector", nxt_edge->id, cur_edge->id );
                        nxt_edge->AddConstraint( BOT_LEFT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, bisect_heading, cur_edge->id, constraint_desc) );
                    } else {
                        sprintf(constraint_desc, "TR_%ld_thick_%ld_pos_to_origin", nxt_edge->id, cur_edge->id );
                        nxt_edge->AddConstraint( TOP_RIGHT_CONSTRAINT, tgConstraint::fromSegment(position, offsetOrigin, cur_edge->id, constraint_desc) );

                        sprintf(constraint_desc, "TR_%ld_thick_%ld_origin_via_bisector", nxt_edge->id, cur_edge->id );
                        nxt_edge->AddConstraint( TOP_RIGHT_CONSTRAINT, tgConstraint::fromRay(offsetOrigin, bisect_heading, cur_edge->id, constraint_desc) );
                    }
                }
            }
#endif
        }                        
    }
}

void tgIntersectionNode::GenerateCapRays( void ) 
{
    tgintersectionedgeinfo_it cur = edgeList.begin();
    tgIntersectionEdgeInfo*   cur_info = (*cur);
    tgIntersectionEdge*       cur_edge = cur_info->GetEdge();    
    char                      constraint_desc[128];
  
    CGAL::Vector_2<edgeArrKernel> vec = cur_edge->ToVector();
    if ( !cur_info->IsOriginating() ) {
        vec = -vec;
    }
    
    // transform the start point to the right and left by width/2
    CGAL::Vector_2<edgeArrKernel> perp = vec.perpendicular( CGAL::CLOCKWISE );    
    edgeArrTransformation translateRight(CGAL::TRANSLATION, perp);
    edgeArrTransformation translateLeft(CGAL::TRANSLATION, -perp);
    
    edgeArrPoint pr = translateRight( position2 );
    edgeArrPoint pl = translateLeft( position2 );
    
    if ( cur_edge ) {
        if ( cur_info->IsOriginating() ) {
            sprintf(constraint_desc, "BR_%ld_Cap", cur_edge->id );
            cur_edge->AddConstraint( BOT_RIGHT_CONSTRAINT, tgConstraint::fromRay(position2, pr, 0, constraint_desc) );

            sprintf(constraint_desc, "BL_%ld_Cap", cur_edge->id );
            cur_edge->AddConstraint( BOT_LEFT_CONSTRAINT, tgConstraint::fromRay(position2, pl, 0, constraint_desc) );
        } else {
            sprintf(constraint_desc, "TR_%ld_Cap", cur_edge->id );
            cur_edge->AddConstraint( TOP_RIGHT_CONSTRAINT, tgConstraint::fromRay(position2, pl, 0, constraint_desc) );

            sprintf(constraint_desc, "TL_%ld_Cap", cur_edge->id );
            cur_edge->AddConstraint( TOP_LEFT_CONSTRAINT, tgConstraint::fromRay(position2, pr, 0, constraint_desc) );
        }
    } else {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "GenerateCapRays: no cur edge!!! " );
    }
}

// this should be an edge traversal, not node traversal
// TODO : Add a tgIntersectionEdge function to handle each constraint
// we do the same thing 4 times in this function
void tgIntersectionNode::IntersectBisectRays( void )
{
#if 0    
    tgintersectionedgeinfo_it cur;
    
    for (cur = edgeList.begin(); cur != edgeList.end(); cur++) { 
        tgIntersectionEdgeInfo* cur_info = (*cur);
        tgIntersectionEdge*     cur_edge = cur_info->GetEdge();
        
        if ( cur_edge ) {
            cur_edge->IntersectConstraintsAndSides( cur_info ); 
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "IntersectBisectRays: NO cur edge" );
        }
    }
#endif    
}

// this may or may not be multisegment

// TODO: don't set constraints until we know we are successful
// 1) we reach intersection with a side
// 2) the edges merge again - intersection at a node
// 3) and edge ends at a cap

void tgIntersectionNode::CompleteMultiSegmentIntersections( tgIntersectionEdgeInfo* cur_info, tgIntersectionEdgeInfo* nxt_info )
{
#if 0    
    char   original_int_name[64];
    
    // list of edges we traverse
    tgintersectionedge_list applyList;
    
    // current position and heading of the bisector
    double ce_heading = 0.0, ne_heading = 0.0, bisect_heading;
    SGGeod bisect_position = position;

    std::list<SGGeod> ce_constraint;
    std::list<SGGeod> ne_constraint;

    std::list<SGGeod> ce_projectlist;
    std::list<SGGeod> ne_projectlist;
    
    // remember how to traverse current and next edge iterators (prev or next)
    bool ce_originating;
    bool ne_originating;
    
    tgIntersectionEdge* ce;
    tgIntersectionEdge* ne;
    
    char ce_layer[128];
    char ne_layer[128];
    
    char ce_cons_name[64];
    char ne_cons_name[64];
    
    int  cur_iteration = 0;

    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ENTER: current edge " << cur_info->GetEdge()->id << " and next edge " << nxt_info->GetEdge()->id );
    
    // iterate until the end of the intersection is found, or we discover this is NOT multisegment
    bool done  = false;
    bool apply = false;

    sprintf( original_int_name, "INT_CE_%04ld_WITH_NE_%04ld", cur_info->GetEdge()->id, nxt_info->GetEdge()->id );
    
    ce_originating = cur_info->IsOriginating();
    ne_originating = nxt_info->IsOriginating();
    
    ce = cur_info->GetEdge();
    ne = nxt_info->GetEdge();

    int ce_id = ce->id;
    int ne_id = ne->id;
    
    while (!done) {        
        // first, calculate the bisect heading
        ce_heading = ce->GetHeading( ce_originating );
        ne_heading = ne->GetHeading( ne_originating );
        bisect_heading = Bisect( bisect_position, ne_heading, ce_heading, true );
        
        if ( ce_originating ) {
            strcpy( ce_cons_name, "BotRight" );
        } else {
            strcpy( ce_cons_name, "TopLeft" );
        }
        
        if ( ne_originating ) {
            strcpy( ne_cons_name, "BotLeft" );
        } else {
            strcpy( ne_cons_name, "TopRight" );
        }

        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " heading is " << ce_heading << " ne " << ne_id << " heading is " << ne_heading << " bisct heading is " << bisect_heading);

        // add the bisect position to constraints        
        ce_constraint.push_back( bisect_position );
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Added intersection position to " << ce_cons_name << " constraint.  size is " << ce_constraint.size() );

        ne_constraint.push_back( bisect_position );
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Added intersection position to " << ne_cons_name << " constraint.  size is " << ne_constraint.size() );
                
//        tgRay bisector( bisect_position, bisect_heading );
        SGGeod bp_next = SGGeodesy::direct( bisect_position, bisect_heading, 1.0 );
        tgRay bisector = tgRay( bisect_position, bp_next );
        
        // find the edge whose constraint we intersect with first
        // current top right, or next top left (w.r.t. originating)
        tgRay ce_end        = ce->GetTopRightConstraint( ce_originating );
        tgRay ne_end        = ne->GetTopLeftConstraint( ne_originating );
        
        tgLine ce_side      = ce->GetRightSide( ce_originating );
        tgLine ne_side      = ne->GetLeftSide(  ne_originating );
        
        SGGeod ce_end_intersect, ne_end_intersect;
        double ce_end_dist = -1000.00, ne_end_dist = -1000.00;
        
        SGGeod ce_side_intersect, ne_side_intersect;
        double ce_side_dist = -1000.00, ne_side_dist = -1000.00;
        
        // add the debug to correct edges
        sprintf( ce_layer, "%s_%03ld_BISECTOR_iter_%d", original_int_name, ne->id, cur_iteration );
        sprintf( ne_layer, "%s_%03ld_BISECTOR_iter_%d", original_int_name, ce->id, cur_iteration );
        
#if DEBUG_INTERSECTIONS
        tgShapefile::FromRay( bisector, ce->GetDatasource(), ce_layer, "bisector" );
        tgShapefile::FromRay( bisector, ne->GetDatasource(), ne_layer, "bisector" );          
#endif

        sprintf( ce_layer, "%s_%03ld_INTERSECTIONS_iter_%d", original_int_name, ne->id, cur_iteration );
        sprintf( ne_layer, "%s_%03ld_INTERSECTIONS_iter_%d", original_int_name, ce->id, cur_iteration );
        
        //todo SGGeodesy exception case- what is it?
        if ( bisector.Intersect( ce_end, ce_end_intersect ) ) {
#if DEBUG_INTERSECTIONS                                    
            tgShapefile::FromGeod( ce_end_intersect, ce->GetDatasource(), ce_layer, "ce_end_intersect" );
#endif            
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ce_end_dist = SGGeodesy::distanceM( bisect_position, ce_end_intersect );
        } else {
            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ERROR - can't intersect bisector " << ce_layer << " with ce " << ce_id << " top right constraint");
            
            done = true;
        }
        
        if ( bisector.Intersect( ne_end, ne_end_intersect ) ) {
#if DEBUG_INTERSECTIONS                        
            tgShapefile::FromGeod( ne_end_intersect, ce->GetDatasource(), ce_layer, "ne_end_intersect" );
#endif
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ne_end_dist = SGGeodesy::distanceM( bisect_position, ne_end_intersect );
        } else {
            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ERROR - can't intersect bisector " << ce_layer << " with ne " << ne_id << " top left constraint");
            
            done = true;
        }
        
        if ( bisector.Intersect( ce_side, ce_side_intersect ) ) {
#if DEBUG_INTERSECTIONS                        
            tgShapefile::FromGeod( ce_side_intersect, ce->GetDatasource(), ce_layer, "ce_side_intersect" );
#endif
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ce_side_dist = SGGeodesy::distanceM( bisect_position, ce_side_intersect );
        }
        
        if ( bisector.Intersect( ne_side, ne_side_intersect ) ) {
#if DEBUG_INTERSECTIONS                        
            tgShapefile::FromGeod( ne_side_intersect, ce->GetDatasource(), ce_layer, "ne_side_intersect" );
#endif
            // if we have a point intersection, what is the distance between 
            // the node and the intersection
            ne_side_dist = SGGeodesy::distanceM( bisect_position, ne_side_intersect );
        }

        sprintf( ce_layer, "%s_%03ld_GETNEXT_iter_%d", original_int_name, ne->id, cur_iteration );
        sprintf( ne_layer, "%s_%03ld_GETPREV_iter_%d", original_int_name, ce->id, cur_iteration );
        
        if ( ce_end_dist > 0.0 && ne_end_dist > 0.0 ) {
            // if the side intersection distances are lower than the constraint distances, than we are done
            if ( (ce_side_dist >= 0.0) && (ce_side_dist < ce_end_dist) && (ne_side_dist >= 0.0) && (ne_side_dist < ne_end_dist) ) {
                
                // verify multiseg ( current_iteration > 1 )
                if ( cur_iteration > 0 ) {
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: all sides are closer than ends, and we've traversed at least one segment - multisegment complete");
                    
                    ce_constraint.push_back(ce_side_intersect);
                    ce_projectlist.push_back(ce_side_intersect);
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Adding intersection with side to " << ce_cons_name << " constraint.  size is " << ce_constraint.size() );
                
                    ne_constraint.push_back(ne_side_intersect);
                    ne_projectlist.push_back(ne_side_intersect);
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Adding intersection with side to " << ne_cons_name << " constraint.  size is " << ne_constraint.size() );
                
                    // set the constraint ( and flag that we intersected the sides )
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Saving " << ce_cons_name << " Constraint with " << ce_constraint.size() << " nodes "); 
                    ce->SetRightConstraint( ce_originating, ce_constraint );
                    ce->SetRightProjectList( ce_originating, ce_projectlist );
                    applyList.push_back( ce );
                    
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Saving " << ne_cons_name << " Constraint with " << ne_constraint.size() << " nodes "); 
                    ne->SetLeftConstraint( ne_originating, ne_constraint );
                    ne->SetLeftProjectList( ne_originating, ne_projectlist );
                    applyList.push_back( ne );
                    
                    done = true;
                    apply = true;
                } else {
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: all sides are closer than ends - looks like a normal edge - not multisegment");
                    done = true;
                }
            } else {
                // Finish the correct edge, and increment to next unfinished edge
                if ( ce_end_dist < ne_end_dist ) {
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " end distance < ne " << ne_id << " distance - finish " << ce_id );
                    
                    bisect_position = ce_end_intersect;
                                        
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Adding end bisect position to " << ce_cons_name << " constraint.  size is " << ce_constraint.size() );
                    ce_constraint.push_back(bisect_position);
                    
                    // ne will get bisector position added to it's constraint at the beginning of the next loop.
                    // But we want to project this vertex to the other side of next edge, so remember it
                    ne_projectlist.push_back(bisect_position);
                    
                    if ( ce_originating ) {
                        // get the next edge (CCW) from the END node : not enough - need to find an edge that intersects the same bisector at the same point...
                        // this function should return NULL if such an edge does not exist
                        cur_info = ce->end->GetNextEdgeInfo( cur_info, bisector, bisect_position, ce_layer );
                    } else {
                        // get the next edge (CCW) from the START node
                        cur_info = ce->start->GetNextEdgeInfo( cur_info, bisector, bisect_position, ce_layer );
                    }
                    
                    if (cur_info == NULL) {
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: can't find next edge for ce " << ce_id );
                        done = true;
                    } else {
                        // set the constraint and project list
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: " << ce->id << " Saving " << ce_cons_name << " Constraint with " << ce_constraint.size() << " nodes "); 
                        ce->SetRightConstraint( ce_originating, ce_constraint );
                        ce->SetRightProjectList( ce_originating, ce_projectlist );
                        applyList.push_back( ce );
                        
                        ce_constraint.clear();
                        ce_projectlist.clear();

                        ce_originating  = cur_info->IsOriginating();
                        ce              = cur_info->GetEdge();
                        ce_id           = ce->id;
                    }
                } else {
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " end distance < ce " << ce_id << " distance - finish " << ne_id );

                    bisect_position = ne_end_intersect;
                    
                    ne_constraint.push_back( ne_end_intersect );
                    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Added end bisect position to " << ne_cons_name << " constraint.  size is " << ne_constraint.size() );
                    
                    // ce will get bisector position added to it's constraint at the beginning of the next loop.
                    // But we want to project this vertex to the other side of cur edge, so remember it
                    ce_projectlist.push_back(bisect_position);

                    if ( ne_originating ) {
                        // get the prev edge (CW) from the END node
                        nxt_info = ne->end->GetPrevEdgeInfo( nxt_info, bisector, bisect_position, ne_layer );
                    } else {
                        // get the prev edge (CW) from the START node
                        nxt_info = ne->start->GetPrevEdgeInfo( nxt_info, bisector, bisect_position, ne_layer );
                    }
                    
                    if (nxt_info == NULL) {
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: can't find next edge for ne " << ne_id );
                        done = true;
                    } else {
                        // set the constraint
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: " << ne->id << " Saving " << ne_cons_name << " Constraint with " << ne_constraint.size() << " nodes "); 
                        ne->SetLeftConstraint( ne_originating, ne_constraint );
                        ne->SetLeftProjectList( ne_originating, ne_projectlist );
                        applyList.push_back( ne );
                        
                        ne_constraint.clear();
                        ne_projectlist.clear();
                        
                        ne_originating  = nxt_info->IsOriginating();
                        ne              = nxt_info->GetEdge();
                        ne_id           = ne->id;
                    }
                }
            }
        } else {
            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: bisector does not intersect both ends");
            done = true;
        }        
        
        cur_iteration++;
    }
    
    for ( tgintersectionedge_it it = applyList.begin(); it != applyList.end(); it++ ) {
        (*it)->ApplyConstraint( apply );
    }
#else
    // list of edges we traverse
    tgintersectionedge_list applyList;

    // current position and heading of the bisector
    // double ce_heading = 0.0, ne_heading = 0.0, bisect_heading;
    edgeArrPoint bisect_position( position.getLongitudeDeg(), position.getLatitudeDeg() );

    std::list<edgeArrPoint> ce_constraint;
    std::list<edgeArrPoint> ne_constraint;

    std::list<edgeArrPoint> ce_projectlist;
    std::list<edgeArrPoint> ne_projectlist;

    // remember how to traverse current and next edge iterators (prev or next)
    bool ce_originating;
    bool ne_originating;

    tgIntersectionEdge* ce;
    tgIntersectionEdge* ne;

    char ce_layer[128];
    char ne_layer[128];

    char ce_cons_name[64];
    char ne_cons_name[64];
    char bisector_name[64];
    
    int  cur_iteration = 0;

    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ENTER: current edge " << cur_info->GetEdge()->id << " and next edge " << nxt_info->GetEdge()->id );

    // iterate until the end of the intersection is found, or we discover this is NOT multisegment
    bool done  = false;
    bool apply = false;

    ce_originating = cur_info->IsOriginating();
    ne_originating = nxt_info->IsOriginating();

    ce = cur_info->GetEdge();
    ne = nxt_info->GetEdge();

    int ce_id = ce->id;
    int ne_id = ne->id;

    while (!done) {        
        // first, calculate the bisect heading
//        ce_heading = ce->GetHeading( ce_originating );
//        ne_heading = ne->GetHeading( ne_originating );
//        bisect_heading = Bisect( bisect_position, ne_heading, ce_heading, true );

        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: iter " << cur_iteration );
        
        // instead of using bisect_heading - find the intersection of the two lines making up cur edge right side, and next edge left side
        edgeArrPoint rayIntersect;
        if ( IntersectCurRightSideWithNextLeftSide( cur_info, nxt_info, rayIntersect ) ) {
            if ( ce_originating ) {
                strcpy( ce_cons_name, "BotRight" );
            } else {
                strcpy( ce_cons_name, "TopLeft" );
            }
        
            if ( ne_originating ) {
                strcpy( ne_cons_name, "BotLeft" );
            } else {
                strcpy( ne_cons_name, "TopRight" );
            }

            // add the bisect position to constraints        
            ce_constraint.push_back( bisect_position );
            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Added intersection position to " << ce_cons_name << " constraint.  size is " << ce_constraint.size() );
        
            ne_constraint.push_back( bisect_position );
            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Added intersection position to " << ne_cons_name << " constraint.  size is " << ne_constraint.size() );
        
            sprintf( bisector_name, "bisector_cur_%04d_nxt_%04d", ce_id, ne_id );
            tgConstraint bisector = tgConstraint::fromRay( bisect_position, rayIntersect, 0, bisector_name );
        
            // this should be a function of the edges - just pass in the bisector ray.  
            // return nearest intersection, and wether it was side or end
            edgeArrPoint ce_end_intersect, ne_end_intersect;
            double       ce_end_dist,      ne_end_dist;
        
            edgeArrPoint ce_side_intersect, ne_side_intersect;
            double       ce_side_dist,      ne_side_dist;
        
            ce->IntersectWithBisector( cur_info->IsOriginating(), true,  bisector, ce_end_intersect, ce_end_dist, ce_side_intersect, ce_side_dist );
            ne->IntersectWithBisector( nxt_info->IsOriginating(), false, bisector, ne_end_intersect, ne_end_dist, ne_side_intersect, ne_side_dist );
        
            if( ce_end_dist == std::numeric_limits< double >::infinity() ) {
                SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ERROR - can't intersect bisector with ce " << ce_id << " top right constraint");
                done = true;
            }
                
            if( ne_end_dist == std::numeric_limits< double >::infinity() ) {
                SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ERROR - can't intersect bisector with ne " << ne_id << " top left constraint");
                done = true;
            }

            if (!done) {
                // if the side intersection distances are lower than the constraint distances, than we are done
                if ( (ce_side_dist != std::numeric_limits< double >::infinity()) && (ce_side_dist < ce_end_dist) && 
                     (ne_side_dist != std::numeric_limits< double >::infinity()) && (ne_side_dist < ne_end_dist) ) {
                
                    // verify multiseg ( current_iteration > 1 )
                    if ( cur_iteration > 0 ) {
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: all sides are closer than ends, and we've traversed at least one segment - multisegment complete");
                    
                        ce_constraint.push_back(ce_side_intersect);
                        ce_projectlist.push_back(ce_side_intersect);
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Adding intersection with side to " << ce_cons_name << " constraint.  size is " << ce_constraint.size() );
                    
                        ne_constraint.push_back(ne_side_intersect);
                        ne_projectlist.push_back(ne_side_intersect);
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Adding intersection with side to " << ne_cons_name << " constraint.  size is " << ne_constraint.size() );
                    
                        // set the constraint ( and flag that we intersected the sides )
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Saving " << ce_cons_name << " Constraint with " << ce_constraint.size() << " nodes "); 
                        ce->SetRightConstraint( ce_originating, ce_constraint );
                        ce->SetRightProjectList( ce_originating, ce_projectlist );
                        applyList.push_back( ce );
                    
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Saving " << ne_cons_name << " Constraint with " << ne_constraint.size() << " nodes "); 
                        ne->SetLeftConstraint( ne_originating, ne_constraint );
                        ne->SetLeftProjectList( ne_originating, ne_projectlist );
                        applyList.push_back( ne );
                    
                        done = true;
                        apply = true;
                    } else {
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: all sides are closer than ends - looks like a normal edge - not multisegment");
                        done = true;
                    }
                } else {
                    // Finish the correct edge, and increment to next unfinished edge
                    if ( ce_end_dist < ne_end_dist ) {
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " end distance < ne " << ne_id << " distance - finish " << ce_id );
                        
                        bisect_position = ce_end_intersect;
                        
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ce " << ce_id << " Adding end bisect position to " << ce_cons_name << " constraint.  size is " << ce_constraint.size() );
                        ce_constraint.push_back(bisect_position);
                        
                        // ne will get bisector position added to it's constraint at the beginning of the next loop.
                        // But we want to project this vertex to the other side of next edge, so remember it
                        ne_projectlist.push_back(bisect_position);
                        
                        if ( ce_originating ) {
                            // get the next edge (CCW) from the END node : not enough - need to find an edge that intersects the same bisector at the same point...
                            // this function should return NULL if such an edge does not exist
                            cur_info = ce->end->GetNextEdgeInfo( cur_info, bisector, bisect_position, ce_layer );
                        } else {
                            // get the next edge (CCW) from the START node
                            cur_info = ce->start->GetNextEdgeInfo( cur_info, bisector, bisect_position, ce_layer );
                        }
                        
                        if (cur_info == NULL) {
                            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: can't find next edge for ce " << ce_id );
                            done = true;
                        } else {
                            // set the constraint and project list
                            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: " << ce->id << " Saving " << ce_cons_name << " Constraint with " << ce_constraint.size() << " nodes "); 
                            ce->SetRightConstraint( ce_originating, ce_constraint );
                            ce->SetRightProjectList( ce_originating, ce_projectlist );
                            applyList.push_back( ce );
                            
                            ce_originating  = cur_info->IsOriginating();
                            ce              = cur_info->GetEdge();
                            ce_id           = ce->id;
                            
                            ce_constraint.clear();
                            ce_constraint.push_back( ce->GetStart( cur_info->IsOriginating() ) );

                            ce_projectlist.clear();                                                        
                        }
                    } else {
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " end distance < ce " << ce_id << " distance - finish " << ne_id );
                        
                        bisect_position = ne_end_intersect;
                        
                        ne_constraint.push_back( ne_end_intersect );
                        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: ne " << ne_id << " Added end bisect position to " << ne_cons_name << " constraint.  size is " << ne_constraint.size() );
                        
                        // ce will get bisector position added to it's constraint at the beginning of the next loop.
                        // But we want to project this vertex to the other side of cur edge, so remember it
                        ce_projectlist.push_back(bisect_position);
                        
                        if ( ne_originating ) {
                            // get the prev edge (CW) from the END node
                            nxt_info = ne->end->GetPrevEdgeInfo( nxt_info, bisector, bisect_position, ne_layer );
                        } else {
                            // get the prev edge (CW) from the START node
                            nxt_info = ne->start->GetPrevEdgeInfo( nxt_info, bisector, bisect_position, ne_layer );
                        }
                        
                        if (nxt_info == NULL) {
                            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: can't find next edge for ne " << ne_id );
                            done = true;
                        } else {
                            // set the constraint
                            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: " << ne->id << " Saving " << ne_cons_name << " Constraint with " << ne_constraint.size() << " nodes "); 
                            ne->SetLeftConstraint( ne_originating, ne_constraint );
                            ne->SetLeftProjectList( ne_originating, ne_projectlist );
                            applyList.push_back( ne );
                            
                            ne_originating  = nxt_info->IsOriginating();
                            ne              = nxt_info->GetEdge();
                            ne_id           = ne->id;

                            ne_constraint.clear();
                            ne_projectlist.clear();
                            ne_constraint.push_back( ne->GetStart( nxt_info->IsOriginating() ) );
                        }
                    }
                }
            } else {
                SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: bisector does not intersect both ends");
                done = true;
            }
        } else {
            SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::CompleteIntersection: cur right side does not intersect with nxt left side ");
            done = true;
        }
        
        cur_iteration++;
#endif
        
    }

    for ( tgintersectionedge_it it = applyList.begin(); it != applyList.end(); it++ ) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionNode::APPLY COPNSTRAINT");
        (*it)->ApplyConstraint( apply );
    }

}

void tgIntersectionNode::CompleteCap( tgIntersectionEdgeInfo* cur_info )
{
#if 0    
    tgIntersectionEdge* ce = cur_info->GetEdge();
    bool ce_originating = cur_info->IsOriginating();
    
    std::list<SGGeod> cons;
    
    if ( ce_originating ) {
        cons.clear();
        cons.push_back( ce->start->GetPosition() );
        cons.push_back( ce->conBotRight );
        ce->SetRightConstraint( ce_originating, cons );
                
        cons.clear();
        cons.push_back( ce->start->GetPosition() );
        cons.push_back( ce->conBotLeft );
        ce->SetLeftConstraint( ce_originating, cons );        
    } else {
        cons.clear();
        cons.push_back( ce->end->GetPosition() );
        cons.push_back( ce->conTopLeft );
        ce->SetRightConstraint( ce_originating, cons );
        
        cons.clear();
        cons.push_back( ce->end->GetPosition() );
        cons.push_back( ce->conTopRight );
        ce->SetLeftConstraint( ce_originating, cons );        
    }
    
    ce->ApplyConstraint( true );    
#endif    
}