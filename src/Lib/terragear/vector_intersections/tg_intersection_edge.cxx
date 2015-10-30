#include <simgear/sg_inlines.h>
#include <simgear/timing/timestamp.hxx>

#include "tg_polygon.hxx"
#include "tg_shapefile.hxx" 
#include "tg_intersection_edge.hxx"
#include "tg_intersection_node.hxx"
#include "tg_misc.hxx"

tgIntersectionEdge::tgIntersectionEdge( tgIntersectionNode* s, tgIntersectionNode* e, double w, int z, unsigned int t, const std::string& db ) : constraints()
{
    static unsigned int ge_count = 0;

    start  = s;
    end    = e;
    width  = w;
    zorder = z;
    type   = t;
    
    id     = ++ge_count;
    flags  = 0;

    // we need to add this edge between start and end to handle multiple edges at a node
    s->AddEdge( true, this );
    e->AddEdge( false, this );
    
    // add a vertex to the arrangement where we will start face construction
    SGGeod gStart = SGGeodesy::direct( start->GetPosition(), GetHeading(true), 0.1 );
    vStart = cgalPoly_Point( gStart.getLongitudeDeg(), gStart.getLatitudeDeg() );
    
    // setup debug database, and layer information for this edge
    debugDataset = db;
    std::stringstream ssp;  ssp << id << "_points";    debugPointLayer = ssp.str();
    std::stringstream ssl;  ssl << id << "_lines";     debugLineLayer  = ssl.str();
    std::stringstream ssap; ssp << id << "_conspoints"; debugConsPointLayer = ssp.str();
    std::stringstream ssal; ssl << id << "_conslines";  debugConsLineLayer  = ssl.str();
    
    // create side constraints.
    GenerateSideConstraints();
}
    
double tgIntersectionEdge::GetHeading( bool originating ) const 
{
    if ( originating ) {
        return SGGeodesy::courseDeg( start->GetPosition(), end->GetPosition() );
    } else {
        return SGGeodesy::courseDeg( end->GetPosition(), start->GetPosition() );
    }
}
    
double tgIntersectionEdge::GetGeodesyLength( void ) const 
{
    return SGGeodesy::distanceM( start->GetPosition(), end->GetPosition() );
}

void tgIntersectionEdge::GenerateSideConstraints( void ) {
    // generate the side constraints:
    // enerate a paralell vector of length width/2
    CGAL::Vector_2<edgeArrKernel> vec = ToVector();
    edgeArrKernel::FT length = sqrt( CGAL::to_double( vec.squared_length()) );
    vec = vec / length;
    vec = vec * ( (width/2)/111000);
    
    // then create a perpendicular vector by rotating clockwise
    CGAL::Vector_2<edgeArrKernel> perp = vec.perpendicular( CGAL::CLOCKWISE );
    
    // create Transformations to translate points by the perp vector
    edgeArrTransformation translateRight(CGAL::TRANSLATION, perp);
    edgeArrTransformation translateLeft(CGAL::TRANSLATION, -perp);
    
    // transform the start point to the right and left by width/2
    edgeArrPoint br = translateRight( start->GetPosition2() );
    edgeArrPoint bl = translateLeft( start->GetPosition2() );
    edgeArrPoint tr = translateRight( end->GetPosition2() );
    edgeArrPoint tl = translateLeft( end->GetPosition2() );
    
    // TODO : do we need these anymore ( we should be able to get them from contraint array
    side_l = edgeArrLine( bl, tl );
    side_r = edgeArrLine( br, tr );
        
    char constraint_desc[64];
    sprintf( constraint_desc, "%06ld_left_side", id );
    constraints[ LEFT_SIDE_CONSTRAINT ].clear();
    AddConstraint( LEFT_SIDE_CONSTRAINT,  tgConstraint::fromLine(bl, tl, 0, constraint_desc) );
    
    sprintf( constraint_desc, "%06ld_right_side", id );
    constraints[ RIGHT_SIDE_CONSTRAINT ].clear();
    AddConstraint( RIGHT_SIDE_CONSTRAINT, tgConstraint::fromLine(br, tr, 0, constraint_desc) );
    
    // remember this edge's texture reference points
    texRefTopRight = tr;
    texRefBotLeft  = bl;
}

tgIntersectionEdge* tgIntersectionEdge::Split( bool originating, tgIntersectionNode* newNode )
{    
    // first - inform the ending node that we aren't associated anymore
    tgIntersectionNode* oldNode = NULL;
    tgIntersectionEdge* newEdge = NULL;
    
    if ( originating ) {
        oldNode = end;
        end = newNode;
    } else {
        oldNode = start;
        start = newNode;
        
        // we need to move startv if this edge's start node changed
        SGGeod gStart = SGGeodesy::direct( start->GetPosition(), GetHeading(true), 0.1 );
        vStart = cgalPoly_Point( gStart.getLongitudeDeg(), gStart.getLatitudeDeg() );
    }

    oldNode->DelEdge( originating, this );
    newNode->AddEdge( !originating, this );

    // create side constraints.
    GenerateSideConstraints();
    
    // now create the new edge with start == new end, end = oldEnd;
    if ( originating ) {
        newEdge = new tgIntersectionEdge( newNode, oldNode, width, zorder, type, debugDataset );
    } else {
        newEdge = new tgIntersectionEdge( oldNode, newNode, width, zorder, type, debugDataset );
    }
    
    return newEdge;
}
 
bool tgIntersectionEdge::VerifyIntersectionLocation( bool originating, const tgConstraint& bisector, const edgeArrPoint& verify_pos )
{
    // find the intersections of bisector, and the correct constraints.
    edgeArrPoint intersectionLocation;
    ConstraintPos_e rightConsType, leftConsType;
    bool verified = false;
    
    if ( originating ) {
        rightConsType = BOT_RIGHT_CONSTRAINT;
        leftConsType  = BOT_LEFT_CONSTRAINT;        
    } else {
        rightConsType = TOP_RIGHT_CONSTRAINT;
        leftConsType  = TOP_LEFT_CONSTRAINT;
    }
    
    if (!verified) {
        for ( unsigned int i=0; i<constraints[rightConsType].size(); i++ ) {
            //tgShapefile::FromConstraint( constraints[rightConsType][i], debugDataset.c_str(), debugLineLayer.c_str() );
            if ( constraints[rightConsType][i].Intersect( bisector, intersectionLocation ) ) {
                //tgShapefile::FromEdgeArrPoint( intersectionLocation, debugDataset.c_str(), debugPointLayer.c_str(), "intersection" );
                if ( verify_pos == intersectionLocation ) {
                    verified = true;
                }
            }
        }
    }
    
    if (!verified) {
        for ( unsigned int i=0; i<constraints[leftConsType].size(); i++ ) {
            //tgShapefile::FromConstraint( constraints[leftConsType][i], debugDataset.c_str(), debugLineLayer.c_str() );
            if ( constraints[leftConsType][i].Intersect( bisector, intersectionLocation ) ) {
                //tgShapefile::FromEdgeArrPoint( intersectionLocation, debugDataset.c_str(), debugPointLayer.c_str(), "intersection" );
                if ( verify_pos == intersectionLocation ) {
                    verified = true;
                }
            }
        }
    }

    return verified;
}
 
void tgIntersectionEdge::AddDebugPoint( const SGGeod& pos, const char* desc )
{
    // tgShapefile::FromGeod( pos, debugDataset.c_str(), debugPointLayer.c_str(), desc );    
}
 
void tgIntersectionEdge::AddDebugPoint( const edgeArrPoint& pos, const char* desc )
{
    // tgShapefile::FromEdgeArrPoint( pos, debugDataset.c_str(), debugPointLayer.c_str(), desc );    
}
 
 bool tgIntersectionEdge::IntersectWithBisector( bool originating, bool right, const tgConstraint& bisector, edgeArrPoint& end_intersect, double& end_dist, edgeArrPoint& side_intersect, double& side_dist )
{
    // find the intersections of bisector, and the correct constraints.
    ConstraintPos_e endConsType, sideConsType;
    edgeArrPoint intersectionLocation;
    double intersectionDistance;
    
    if ( originating ) {
        if ( right ) {
            endConsType  = TOP_RIGHT_CONSTRAINT;
            sideConsType = RIGHT_SIDE_CONSTRAINT;
        } else {
            endConsType  = TOP_LEFT_CONSTRAINT;
            sideConsType = LEFT_SIDE_CONSTRAINT;
        }
    } else {
        if ( right ) {
            endConsType  = BOT_LEFT_CONSTRAINT;
            sideConsType = LEFT_SIDE_CONSTRAINT;
        } else {
            endConsType  = BOT_RIGHT_CONSTRAINT;
            sideConsType = RIGHT_SIDE_CONSTRAINT;
        }
    }
    
    // find the instersection points and distances from the bisector ray origin.
    end_dist = std::numeric_limits< double >::infinity();
    for ( unsigned int i=0; i<constraints[endConsType].size(); i++ ) {
        if ( constraints[endConsType][i].Intersect( bisector, intersectionLocation ) ) {
            intersectionDistance = CGAL::to_double( CGAL::squared_distance( bisector.getStart(), intersectionLocation ) );
            if ( intersectionDistance < end_dist ) {
                end_intersect = intersectionLocation;
                end_dist = intersectionDistance;
            }
        }
    }
    
    side_dist = std::numeric_limits< double >::infinity();
    for ( unsigned int i=0; i<constraints[sideConsType].size(); i++ ) {
        if ( constraints[sideConsType][i].Intersect( bisector, intersectionLocation ) ) {
            intersectionDistance = CGAL::to_double( CGAL::squared_distance( bisector.getStart(), intersectionLocation ) );
            if ( intersectionDistance < side_dist ) {
                side_intersect = intersectionLocation;
                side_dist = intersectionDistance;
            }
        }
    }
    
    return true;
}
 
tgRectangle tgIntersectionEdge::GetBoundingBox( void ) const
{
    SGGeod min, max;

    double minx =  SG_MIN2( start->GetPosition().getLongitudeDeg(), end->GetPosition().getLongitudeDeg() );
    double miny =  SG_MIN2( start->GetPosition().getLatitudeDeg(),  end->GetPosition().getLatitudeDeg() );
    double maxx =  SG_MAX2( start->GetPosition().getLongitudeDeg(), end->GetPosition().getLongitudeDeg() );
    double maxy =  SG_MAX2( start->GetPosition().getLatitudeDeg(),  end->GetPosition().getLatitudeDeg() );

    min = SGGeod::fromDeg( minx, miny );
    max = SGGeod::fromDeg( maxx, maxy );

    return tgRectangle( min, max );
}
    
tgLine tgIntersectionEdge::ToLine( void ) const
{            
    return tgLine( start->GetPosition(), end->GetPosition() );
}

CGAL::Vector_2<edgeArrKernel> tgIntersectionEdge::ToVector( void ) const {
    return CGAL::Vector_2<edgeArrKernel>( start->GetPosition2(), end->GetPosition2() );
}

void tgIntersectionEdge::SetLeftConstraint( bool originating, const std::list<edgeArrPoint>& cons )
{
    if ( originating ) {
        constrain_msbl.clear();
        
        // if we are originating, push to front of bottom left constraint
        for ( std::list<edgeArrPoint>::const_iterator it = cons.begin(); it != cons.end(); it ++ ) {
            constrain_msbl.push_back( (*it) );
        }
    } else {
        constrain_mstr.clear();

        // otherwise, push to front of top right constraint
        for ( std::list<edgeArrPoint>::const_iterator it = cons.begin(); it != cons.end(); it ++ ) {
            constrain_mstr.push_back( (*it) );
        }
    }
}

void tgIntersectionEdge::SetRightConstraint( bool originating, const std::list<edgeArrPoint>& cons )
{
    if ( originating ) {
        constrain_msbr.clear();
    
        // if we are originating, push to back of bottom right constraint
        for ( std::list<edgeArrPoint>::const_iterator it = cons.begin(); it != cons.end(); it ++ ) {
            constrain_msbr.push_back( (*it) );
        }
    } else {
        constrain_mstl.clear();

        // otherwise, push to back of top left constraint
        for ( std::list<edgeArrPoint>::const_iterator it = cons.begin(); it != cons.end(); it ++ ) {
            constrain_mstl.push_back( (*it) );
        }
    }
}

void tgIntersectionEdge::SetLeftProjectList( bool originating, const std::list<edgeArrPoint>& pl )
{
    if ( originating ) {
        projectlist_msbl.clear();
            
        // if we are originating, push to front of bottom left constraint
        for ( std::list<edgeArrPoint>::const_iterator it = pl.begin(); it != pl.end(); it ++ ) {
            projectlist_msbl.push_back( (*it) );
        }
    } else {
        projectlist_mstr.clear();
        
        // otherwise, push to front of top right constraint
        for ( std::list<edgeArrPoint>::const_iterator it = pl.begin(); it != pl.end(); it ++ ) {
            projectlist_mstr.push_back( (*it) );
        }
    }
}

void tgIntersectionEdge::SetRightProjectList( bool originating, const std::list<edgeArrPoint>& pl )
{
    if ( originating ) {
        projectlist_msbr.clear();
        
        // if we are originating, push to back of bottom right constraint
        for ( std::list<edgeArrPoint>::const_iterator it = pl.begin(); it != pl.end(); it ++ ) {
            projectlist_msbr.push_back( (*it) );
        }
    } else {
        projectlist_mstl.clear();
        
        // otherwise, push to back of top left constraint
        for ( std::list<edgeArrPoint>::const_iterator it = pl.begin(); it != pl.end(); it ++ ) {
            projectlist_mstl.push_back( (*it) );
        }
    }
}

void tgIntersectionEdge::ApplyConstraint( bool apply )
{
    unsigned int i;
    char cons_desc[32];
    
    if ( constrain_msbr.size() >= 2 ) {
        if ( apply ) {
            // add constraints to bottom right - replace what's there
            constraints[ BOT_RIGHT_CONSTRAINT ].clear();
            for ( i=0; i<constrain_msbr.size() - 2; i++ ) {
                sprintf( cons_desc, "%06ld_msbr_%02d", id, i );
                tgConstraint cons = tgConstraint::fromSegment( constrain_msbr[i], constrain_msbr[i+1], 0, cons_desc );
                AddConstraint( BOT_RIGHT_CONSTRAINT, cons );
            }
            sprintf( cons_desc, "%06ld_msbr_%02d", id, i );
            tgConstraint cons = tgConstraint::fromRay( constrain_msbr[i], constrain_msbr[i+1], 0, cons_desc );
            AddConstraint( BOT_RIGHT_CONSTRAINT, cons );
        } else {
            constrain_msbr.clear();
        }
    }
    
    if ( constrain_mstr.size() >= 2 ) {
        if ( apply ) {
            // add constraints to top right - replace what's there
            constraints[ TOP_RIGHT_CONSTRAINT ].clear();
            for ( i=0; i<constrain_mstr.size() - 2; i++ ) {
                sprintf( cons_desc, "%06ld_mstr_%02d", id, i );
                tgConstraint cons = tgConstraint::fromSegment( constrain_mstr[i], constrain_mstr[i+1], 0, cons_desc );
                AddConstraint( TOP_RIGHT_CONSTRAINT, cons );
            }
            sprintf( cons_desc, "%06ld_mstr_%02d", id, i );
            tgConstraint cons = tgConstraint::fromRay( constrain_mstr[i], constrain_mstr[i+1], 0, cons_desc );
            AddConstraint( TOP_RIGHT_CONSTRAINT, cons );
        } else {
            constrain_mstr.clear();
        }
    }

    if ( constrain_msbl.size() >= 2 ) {
        if ( apply ) {
            // add constraints to bottom left - replace what's there
            constraints[ BOT_LEFT_CONSTRAINT ].clear();
            for ( i=0; i<constrain_msbl.size() - 2; i++ ) {
                sprintf( cons_desc, "%06ld_msbl_%02d", id, i );
                tgConstraint cons = tgConstraint::fromSegment( constrain_msbl[i], constrain_msbl[i+1], 0, cons_desc );
                AddConstraint( BOT_LEFT_CONSTRAINT, cons );
            }
            sprintf( cons_desc, "%06ld_msbl_%02d", id, i );
            tgConstraint cons = tgConstraint::fromRay( constrain_msbl[i], constrain_msbl[i+1], 0, cons_desc );
            AddConstraint( BOT_LEFT_CONSTRAINT, cons );
        } else {
            constrain_msbr.clear();
        }
    }
    
    if ( constrain_mstl.size() >= 2 ) {
        if ( apply ) {
            // add constraints to tpp left - replace what's there
            constraints[ TOP_LEFT_CONSTRAINT ].clear();
            for ( i=0; i<constrain_mstl.size() - 2; i++ ) {
                sprintf( cons_desc, "%06ld_mstl_%02d", id, i );
                tgConstraint cons = tgConstraint::fromSegment( constrain_mstl[i], constrain_mstl[i+1], 0, cons_desc );
                AddConstraint( TOP_LEFT_CONSTRAINT, cons );
            }
            sprintf( cons_desc, "%06ld_mstl_%02d", id, i );
            tgConstraint cons = tgConstraint::fromRay( constrain_mstl[i], constrain_mstl[i+1], 0, cons_desc );
            AddConstraint( TOP_LEFT_CONSTRAINT, cons );
        } else {
            constrain_mstr.clear();
        }
    }
}

edgeArrPoint tgIntersectionEdge::GetStart( bool originating ) const {
    if ( originating ) {
        return start->GetPosition2();
    } else {
        return end->GetPosition2();
    }
}

bool tgIntersectionEdge::Verify( unsigned long int f )  
{ 
    bool pass = true;
    
    if ( f & FLAGS_INTERSECTED_BOTTOM_CONSTRAINTS ) {
        if ( (flags & FLAGS_INTERSECTED_BOTTOM_CONSTRAINTS) == 0 ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::Verify : edge " << id << " never got bottom contraint intersection with side");

            //tgSegment seg( start->GetPosition(), end->GetPosition() );
            //tgShapefile::FromSegment( seg, true, datasrc, "no_bottom_intersections", description );
            
            pass = false;
        }
    }
    
    if ( f & FLAGS_INTERSECTED_TOP_CONSTRAINTS ) {
        if ( (flags & FLAGS_INTERSECTED_TOP_CONSTRAINTS) == 0 ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::Verify : edge " << id << " never got top contraint intersection with side");                

            //tgSegment seg( start->GetPosition(), end->GetPosition() );
            //tgShapefile::FromSegment( seg, true, datasrc, "no_top_intersections", description );
            
            pass = false;
        }
    }

    if ( f & FLAGS_TEXTURED ) {
        if ( (flags & FLAGS_TEXTURED) == 0 ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::Verify : edge " << id << " never got textured");
            
            //tgSegment seg( start->GetPosition(), end->GetPosition() );
            //tgShapefile::FromSegment( seg, true, datasrc, "not_textured", description );
            
            pass = false;
            
            //poly.SetVertexAttributeInt(TG_VA_CONSTANT, 0, 0);   
        }
    }
    
    return pass; 
}

tgPolygonSet tgIntersectionEdge::GetPoly(const char* prefix)
{    
    return tgPolygonSet( poly, meta );
}

double tgIntersectionEdge::Texture( bool originating, double v_end, tgIntersectionGeneratorTexInfoCb texInfoCb, double ratio )
{
    std::string material = "unset";
    double      texAtlasStartU, texAtlasEndU;
    double      texAtlasStartV, texAtlasEndV;
    double      v_start;
    double      v_dist;
    double      heading;

#if DEBUG_TEXTURE        
    static int  textured_idx = 0;
#endif
        
    if ( start->IsCap() ) {
        texInfoCb( type, true, material, texAtlasStartU, texAtlasEndU, texAtlasStartV, texAtlasEndV, v_dist );
        
        if ( originating ) {
            heading = SGGeodesy::courseDeg( start->GetPosition(), end->GetPosition() );
            meta.setTextureRef( texRefBotLeft, width, 0.5, heading );
        } else {
            heading = SGGeodesy::courseDeg( end->GetPosition(), start->GetPosition() );
            meta.setTextureRef( texRefTopRight, width, 0.5, heading );        
        }
        meta.setMaterial( material );
        meta.setTextureMethod( tgPolygonSetMeta::TEX_1X1_ATLAS );
        meta.setTextureLimits( texAtlasStartU, texAtlasStartV, texAtlasEndU, texAtlasEndV );
        // poly.SetVertexAttributeInt(TG_VA_CONSTANT, 0, 1);

#if DEBUG_TEXTURE        
        // DEBUG : add an arrow with v_start, v_end
        tgSegment seg( start->GetPosition(), end->GetPosition() );
        char from_to[128];
        sprintf( from_to, "id %ld textured start cap %d from %lf, to %lf", id, textured_idx++, v_start, v_end );
        tgShapefile::FromSegment( seg, true, "./", "Texture", from_to );
#endif
        
    } else if ( end->IsCap() ) {
        texInfoCb( type, true, material, texAtlasStartU, texAtlasEndU, texAtlasStartV, texAtlasEndV, v_dist );
        
        if ( originating ) {
            heading = SGGeodesy::courseDeg( start->GetPosition(), end->GetPosition() );
            meta.setTextureRef( texRefBotLeft, width, 0.5, heading );
        } else {
            heading = SGGeodesy::courseDeg( end->GetPosition(), start->GetPosition() );
            meta.setTextureRef( texRefTopRight, width, 0.5, heading );        
        }
        meta.setMaterial( material );
        meta.setTextureMethod( tgPolygonSetMeta::TEX_1X1_ATLAS );
        meta.setTextureLimits( texAtlasStartU, texAtlasStartV, texAtlasEndU, texAtlasEndV );
//      poly.setVertexAttributeInt(TG_VA_CONSTANT, 0, 1);
        
#if DEBUG_TEXTURE
        // DEBUG : add an arrow with v_start, v_end
        tgSegment seg( start->GetPosition(), end->GetPosition() );
        char from_to[128];
        sprintf( from_to, "id %ld textured end start cap %d from %lf, to %lf", id, textured_idx++, v_start, v_end );
        tgShapefile::FromSegment( seg, true, "./", "Texture", from_to );
#endif
        
    } else {
        texInfoCb( type, false, material, texAtlasStartU, texAtlasEndU, texAtlasStartV, texAtlasEndV, v_dist );
        v_dist *= ratio;
        
        double dist = SGGeodesy::distanceM( start->GetPosition(), end->GetPosition() );
        v_start = fmod( v_end, 1.0 );
        v_end   = v_start + (dist/v_dist);
        
        if ( originating ) {
            SG_LOG( SG_GENERAL, LOG_TEXTURE, "tgIntersectionEdge::Texture : edge " << id << " originating : v_start=" << v_start << " v_end= " << v_end << " dist= " << dist << " v_dist= " << v_dist );
            heading = SGGeodesy::courseDeg( start->GetPosition(), end->GetPosition() );
            meta.setTextureRef( texRefBotLeft, width, dist, heading );
        } else {
            SG_LOG( SG_GENERAL, LOG_TEXTURE, "tgIntersectionEdge::Texture : edge " << id << " NOT originating : v_start=" << v_start << " v_end= " << v_end << " dist= " << dist << " v_dist= " << v_dist );
            heading = SGGeodesy::courseDeg( end->GetPosition(), start->GetPosition() );
            meta.setTextureRef( texRefTopRight, width, dist, heading );        
        }    

        meta.setMaterial( material );
        meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_TPS_CLIPU, -1.0, 0.0, 1.0, 0.0 );
        meta.setTextureLimits( texAtlasStartU, v_start, texAtlasEndU, v_end );
//      meta.setVertexAttributeInt(TG_VA_CONSTANT, 0, 0);
        
#if DEBUG_TEXTURE        
        // DEBUG : add an arrow with v_start, v_end
        tgSegment seg( start->GetPosition(), end->GetPosition() );
        char from_to[128];
        sprintf( from_to, "id %ld textured %d from %lf, to %lf", id, textured_idx++, v_start, v_end );
        tgShapefile::FromSegment( seg, true, "./", "Texture", from_to );
#endif        
    }
    
    flags |= FLAGS_TEXTURED;

    return v_end;
}

tgIntersectionEdgeInfo::tgIntersectionEdgeInfo( bool orig, tgIntersectionEdge* e ) 
{    
    edge              = e;
    originating       = orig;
    
    if ( originating ) {
        heading         = SGGeodesy::courseDeg( edge->start->GetPosition(), edge->end->GetPosition() );
        geodesy_heading = SGGeodesy::courseDeg( edge->start->GetPosition(), edge->end->GetPosition() );
    } else {
        heading         = SGGeodesy::courseDeg( edge->end->GetPosition(), edge->start->GetPosition() );
        geodesy_heading = SGGeodesy::courseDeg( edge->end->GetPosition(), edge->start->GetPosition() );
    }
}

double tgIntersectionEdgeInfo::Texture( double vEnd, tgIntersectionGeneratorTexInfoCb texInfoCb, double ratio ) {
    return edge->Texture( originating, vEnd, texInfoCb, ratio );
}

void tgIntersectionEdgeInfo::TextureStartCap( tgIntersectionGeneratorTexInfoCb texInfoCb ) {
    edge->Texture( originating, 0.0f, texInfoCb, 1.0f );
}

void tgIntersectionEdgeInfo::TextureEndCap( tgIntersectionGeneratorTexInfoCb texInfoCb ) {
    edge->Texture( originating, 0.0f, texInfoCb, 1.0f );
}

bool tgIntersectionEdgeInfo::IsTextured( void ) const {
    return edge->IsTextured(); 
}

bool tgIntersectionEdgeInfo::IsStartCap(void) const
{
    return (edge->start->Degree() == 1);
}

bool tgIntersectionEdgeInfo::IsEndCap(void) const
{
    return (edge->end->Degree() == 1);
}

// if pos is right side, check botright, rightside and topright
// if pos is left siide, check botleft leftside and topleft
void tgIntersectionEdge::AddConstraint( ConstraintPos_e pos, tgConstraint cons )
{
    // add the constraint to this edges's debug dir
    cons.setId( id );
    
    constraints[pos].push_back( cons );
}

void tgIntersectionEdge::Generate( void )
{
    std::vector<cgalPoly_Point> nodes;
    bool                        valid = true;

    // dump the face on which the start vertex lies
    // offset the start location a bit to find our face
    poly.clear();
    
    // Create the arrangement
    edgeArrangement     arr;
    edgeArrVertexHandle hStart;
        
    char desc[128];
    SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdge::Generate : " << id );
    
    for ( unsigned int pos=0; pos<NUM_CONSTRAINTS; pos++ ) {
        for ( unsigned int c=0; c< constraints[pos].size(); c++ ) {

            if ( id == 818 ) {
                constraints[pos][c].toShapefile( debugDataset.c_str(), debugConsLineLayer.c_str() );
                sprintf( desc, "cons_%s_start", constraints[pos][c].getDescription().c_str() );
                tgShapefile::FromEdgeArrPoint( constraints[pos][c].getStart(), debugDataset.c_str(), debugConsPointLayer.c_str(), desc );
                sprintf( desc, "cons_%s_end", constraints[pos][c].getDescription().c_str() );
                tgShapefile::FromEdgeArrPoint( constraints[pos][c].getEnd(), debugDataset.c_str(), debugConsPointLayer.c_str(), desc );
        
                std::cout << "inserting cons " << constraints[pos][c].getDescription() << std::endl;
                CGAL::insert( arr, constraints[pos][c].getCurve() );
                std::cout << "done" << std::endl;
            } else {
                CGAL::insert( arr, constraints[pos][c].getCurve() );                
            }
        }
    }
    
    
    // dump the arrangement ( bounded edges only )
    CGAL_precondition( arr.is_valid () );
    
    hStart = CGAL::insert_point( arr, vStart );
    if ( hStart->is_isolated() ) {
        edgeArrFaceHandle           faceh = hStart->face();
        
        if ( faceh->has_outer_ccb() ) {
            edgeArrCcbHECirc        ccb = faceh->outer_ccb();
            edgeArrCcbHECirc        cur = ccb;
            edgeArrHEConstHandle    he;
        
            do
            {
                he = cur;

                // ignore inner antenna
                if ( he->face() != he->twin()->face() ) {
                    if ( !he->source()->is_at_open_boundary() ) 
                    {
                        if ( !he->target()->is_at_open_boundary() ) {
                            nodes.push_back( he->source()->point() );
                        } else {
                            SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdgeInfo::Generate : edge " << id << " HAS he target on open boundary ");
                            valid = false;
                        }
                    } else {
                        SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdgeInfo::Generate : edge " << id << " HAS he source on open boundary ");                            
                        valid = false;
                    }
                } else {
                    SG_LOG( SG_GENERAL, SG_ALERT, "tgIntersectionEdgeInfo::Generate : edge " << id << " Ignoring Antenna edge ");
                }
            
                ++cur;
            } while (cur != ccb);
        }
    }
    
    if( valid ) {
        poly = cgalPoly_Polygon( nodes.begin(), nodes.end() );
    }
}

void tgIntersectionEdge::DumpArrangement( OGRLayer* skeleton_lid, OGRLayer* constraints_lid, OGRLayer* startv_lid, OGRLayer* poly_lid )
{
    char description[256];

    // dump the line
    if ( skeleton_lid ) {
        sprintf( description, "%06ld_skeleton", id );
        tgShapefile::FromSegment( skeleton_lid, tgSegment(start->GetPosition(), end->GetPosition()), true, description );
    }
    
    // dump start vertex
    if ( startv_lid ) {
        sprintf( description, "%06ld_start_v", id );
        SGGeod gStart = SGGeod::fromDeg( CGAL::to_double(vStart.x()), CGAL::to_double(vStart.y()) );
        tgShapefile::FromGeod( startv_lid, gStart, description );    
    }

    // dump the constraints
    if ( constraints_lid ) {
        for ( unsigned int pos=0; pos<NUM_CONSTRAINTS; pos++ ) {
            for ( unsigned int c=0; c<constraints[pos].size(); c++ ) {
                constraints[pos][c].toShapefile( constraints_lid );
            }
        }
    }
    
    // dump the poly
//    if ( poly_lid ) {
//        sprintf( description, "%06ld_poly", id );
//        poly.toShapefile( poly_lid, description );
//    }

}

tgRay tgIntersectionEdgeInfo::GetDirectionRay(void) const
{
    SGGeod s, e;
    
    if ( originating ) {
        s = edge->start->GetPosition();
        e = edge->end->GetPosition();
    } else {
        s = edge->end->GetPosition();
        e = edge->start->GetPosition();            
    }

    return tgRay( s, e );
}