#include <stdlib.h>

#include "global.hxx"
#include "beznode.hxx"
#include "linearfeature.hxx"

double LinearFeature::GetCapDist( unsigned int type )
{
    double cap_dist;
    
    switch( type )
    {                        
        case RWY_CENTERLINE:
            cap_dist = 200 * SG_FEET_TO_METER * 0.125;
            break;
                        
        default:
            cap_dist = 10 * SG_FEET_TO_METER * 0.125;
            break;
    }
    
    return cap_dist;
}

void LinearFeature::ConvertContour( BezContour* src, bool closed )
{
    BezNode*  curNode;
    BezNode*  nextNode;

    SGGeod    curLoc;
    SGGeod    nextLoc;
    SGGeod    cp1;
    SGGeod    cp2;

    int       curve_type = CURVE_LINEAR;
    double    total_dist;
    double    theta1, theta2;
    int       num_segs = BEZIER_DETAIL;

    Marking*  cur_mark = NULL;
    Lighting* cur_light = NULL;

    TG_LOG(SG_GENERAL, SG_DEBUG, " LinearFeature::ConvertContour - Creating a contour with " << src->size() << " nodes");

    // clear anything in the point list
    points.Erase();

    // iterate through each bezier node in the contour
    for (unsigned int i=0; i <= src->size()-1; i++)
    {
        curNode = src->at(i);

        if (i < src->size() - 1)
        {
            nextNode = src->at(i+1);
        }
        else
        {
            // for the last node, next is the first. as all contours are closed
            nextNode = src->at(0);
        }

        ////////////////////////////////////////////////////////////////////////////////////
        // remember the index for the starting stopping of marks on the converted contour

        // are we watching a mark for the end?
        if (cur_mark)
        {
            if (curNode->GetMarking() != cur_mark->type)
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking has changed from " << cur_mark->type << " to " << curNode->GetMarking() << " save mark from " << cur_mark->start_idx << " to " << points.GetSize() );

                // marking has ended, or changed
                cur_mark->end_idx = points.GetSize();
                marks.push_back(cur_mark);
                cur_mark = NULL;
            }
            else
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Continue Marking from " << cur_mark->start_idx << " with type " << cur_mark->type );
            }
        }

        // should we start a new mark?
        if (cur_mark == NULL)
        {
            if (curNode->GetMarking())
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Start Marking from " << points.GetSize() << " with type " << curNode->GetMarking() );

                // we aren't watching a mark, and this node has one
                cur_mark = new Marking;
                cur_mark->type = curNode->GetMarking();
                cur_mark->start_idx = points.GetSize();
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////////////
        // remember the index for the starting stopping of lights on the converted contour

        // are we watching a mark for the end?
        if (cur_light)
        {
            if (curNode->GetLighting() != cur_light->type)
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Lighting has changed from " << cur_light->type << " to " << curNode->GetLighting() << " save light from " << cur_light->start_idx << " to " << points.GetSize() );

                // lighting has ended, or changed : add final light
                cur_light->end_idx = points.GetSize();
                lights.push_back(cur_light);
                cur_light = NULL;
            }
            else
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Continue Lighting from " << cur_light->start_idx << " with type " << cur_light->type );
            }
        }

        // should we start a new light?
        if (cur_light == NULL)
        {
            if (curNode->GetLighting())
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Start Lighting from " << points.GetSize() << " with type " << curNode->GetLighting() );

                // we aren't watching a mark, and this node has one
                cur_light = new Lighting;
                cur_light->type = curNode->GetLighting();
                cur_light->start_idx = points.GetSize();
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////

        // now determine how we will iterate from current node to next node
        if( curNode->HasNextCp() )
        {
            // next curve is cubic or quadratic
            if( nextNode->HasPrevCp() )
            {
                // curve is cubic : need both control points
                curve_type = CURVE_CUBIC;
                cp1 = curNode->GetNextCp();
                cp2 = nextNode->GetPrevCp();
                total_dist = CubicDistance( curNode->GetLoc(), cp1, cp2, nextNode->GetLoc() );
                theta1 = SGMiscd::rad2deg( CalculateTheta( curNode->GetLoc(), cp1, nextNode->GetLoc()) );
                theta2 = SGMiscd::rad2deg( CalculateTheta( curNode->GetLoc(), cp2, nextNode->GetLoc()) );
            }
            else
            {
                // curve is quadratic using current nodes cp as the cp
                curve_type = CURVE_QUADRATIC;
                cp1 = curNode->GetNextCp();
                total_dist = QuadraticDistance( curNode->GetLoc(), cp1, nextNode->GetLoc() );
                theta1 = SGMiscd::rad2deg( CalculateTheta( curNode->GetLoc(), cp1, nextNode->GetLoc()) );
            }
        }
        else
        {
            // next curve is quadratic or linear
            if( nextNode->HasPrevCp() )
            {
                // curve is quadratic using next nodes cp as the cp
                curve_type = CURVE_QUADRATIC;
                cp1 = nextNode->GetPrevCp();
                total_dist = QuadraticDistance( curNode->GetLoc(), cp1, nextNode->GetLoc() );
                theta1 = SGMiscd::rad2deg( CalculateTheta( curNode->GetLoc(), cp1, nextNode->GetLoc()) );
            }
            else
            {
                // curve is linear
                curve_type = CURVE_LINEAR;
                total_dist = LinearDistance( curNode->GetLoc(), nextNode->GetLoc() );
            }
        }

        // One more test - some people are using bezier curves to draw straight lines - this can cause a bit of havoc...
        // Sometimes, the control point lies just beyond the final point.  We try to make a 'hook' at the end, which makes some really bad polys
        // Just convert the entire segment to linear
        // this can be detected in quadratic curves (current issue in LFKJ) when the contol point lies within the line generated from point 1 to point 2
        // theat close to 180 at the control point to the cur node and next node
        if ( curve_type == CURVE_QUADRATIC )
        {
            if ( (abs(theta1 - 180.0) < 5.0 ) || (abs(theta1) < 5.0 ) || (isnan(theta1)) )
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Quadtratic curve with cp in line : convert to linear: " << description << ": theta is " << theta1 );
                curve_type = CURVE_LINEAR;
            }
            else
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Quadtratic curve withOUT cp in line : keep quadtratic: " << description << ": theta is " << theta1 );
            }
        }

        if ( curve_type == CURVE_CUBIC )
        {
            if ( (abs(theta1 - 180.0) < 5.0 ) || (abs(theta1) < 5.0 ) || (isnan(theta1)) )
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Cubic curve with cp1 in line : " << description << ": theta is " << theta1 );

                if ( (abs(theta2 - 180.0) < 5.0 ) || (abs(theta2) < 5.0 ) || (isnan(theta2)) )
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "\n               and cp2 in line : " << description << ": theta is " << theta2 << " CONVERTING TO LINEAR" );

                    curve_type = CURVE_LINEAR;
                }
                else
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "\n               BUT cp2 NOT in line : " << description << ": theta is " << theta2 );
                }
            }
            else
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Cubic curve withOUT cp1 in line : keep quadtratic: " << description << ": theta is " << theta1 );

                if ( (abs(theta2 - 180.0) < 5.0 ) || (abs(theta2) < 5.0 ) || (isnan(theta2)) )
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "\n               BUT cp2 IS in line : " << description << ": theta is " << theta2 );
                }
                else
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "\n               AND cp2 NOT in line : " << description << ": theta is " << theta2 );
                }
            }
        }

        if (total_dist < 4.0f)
        {
            if (curve_type != CURVE_LINEAR)
            {
                // If total distance is < 4 meters, then we need to modify num Segments so that each segment >= 1/2 meter
                num_segs = ((int)total_dist + 1) * 2;
                TG_LOG(SG_GENERAL, SG_DEBUG, "Segment from " << curNode->GetLoc() << " to " << nextNode->GetLoc() );
                TG_LOG(SG_GENERAL, SG_DEBUG, "        Distance is " << total_dist << " ( < 4.0) so num_segs is " << num_segs );
            }
            else
            {
                num_segs = 1;
            }
        }
        else if (total_dist > 800.0f)
        {
            // If total distance is > 800 meters, then we need to modify num Segments so that each segment <= 100 meters
            num_segs = total_dist / 100.0f + 1;
            TG_LOG(SG_GENERAL, SG_DEBUG, "Segment from " << curNode->GetLoc() << " to " << nextNode->GetLoc() );
            TG_LOG(SG_GENERAL, SG_DEBUG, "        Distance is " << total_dist << " ( > 100.0) so num_segs is " << num_segs );
        }
        else
        {
            if (curve_type != CURVE_LINEAR)
            {
                num_segs = 8;
                TG_LOG(SG_GENERAL, SG_DEBUG, "Segment from " << curNode->GetLoc() << " to " << nextNode->GetLoc() );
                TG_LOG(SG_GENERAL, SG_DEBUG, "        Distance is " << total_dist << " (OK) so num_segs is " << num_segs );
            }
            else
            {
                num_segs = 1;
            }
        }

        // if only one segment, revert to linear
        if (num_segs == 1)
        {
            curve_type = CURVE_LINEAR;
        }

        // initialize current location
        curLoc = curNode->GetLoc();
        if (curve_type != CURVE_LINEAR)
        {
            for (int p=0; p<num_segs; p++)
            {
                // calculate next location
                if (curve_type == CURVE_QUADRATIC)
                {
                    nextLoc = CalculateQuadraticLocation( curNode->GetLoc(), cp1, nextNode->GetLoc(), (1.0f/num_segs) * (p+1) );
                }
                else
                {
                    nextLoc = CalculateCubicLocation( curNode->GetLoc(), cp1, cp2, nextNode->GetLoc(), (1.0f/num_segs) * (p+1) );
                }

                // add the feature vertex
                points.AddNode(curLoc);

                if (p==0)
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "adding Curve Anchor node (type " << curve_type << ") at " << curLoc );
                }
                else
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "   add bezier node (type  " << curve_type << ") at " << curLoc );
                }

                // now set set prev and cur locations for the next iteration
                curLoc = nextLoc;
            }
        }
        else
        {
            // calculate linear distance to determine how many segments we want
            if (num_segs > 1)
            {
                for (int p=0; p<num_segs; p++)
                {
                    // calculate next location
                    nextLoc = CalculateLinearLocation( curNode->GetLoc(), nextNode->GetLoc(), (1.0f/num_segs) * (p+1) );

                    // add the feature vertex
                    points.AddNode(curLoc);

                    if (p==0)
                    {
                        TG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear anchor node at " << curLoc );
                    }
                    else
                    {
                        TG_LOG(SG_GENERAL, SG_DEBUG, "   add linear node at " << curLoc );
                    }

                    // now set set prev and cur locations for the next iteration
                    curLoc = nextLoc;
                }
            }
            else
            {
                nextLoc = nextNode->GetLoc();

                // just add the one vertex - dist is small
                points.AddNode(curLoc);

                TG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear Anchor node at " << curLoc );

                curLoc = nextLoc;
            }
        }
    }

    if (closed)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Closed COntour : adding last node at " << curLoc );

        // need to add the markings for last segment
        points.AddNode(curLoc);
    }

    // check for marking that goes all the way to the end...
    if (cur_mark)
    {
       TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking from " << cur_mark->start_idx << " with type " << cur_mark->type << " ends at the end of the contour: " << points.GetSize() );

       cur_mark->end_idx = points.GetSize()-1;
       marks.push_back(cur_mark);
       cur_mark = NULL;
    }

    // check for lighting that goes all the way to the end...
    if (cur_light)
    {
       TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Lighting from " << cur_light->start_idx << " with type " << cur_light->type << " ends at the end of the contour: " << points.GetSize() );

       cur_light->end_idx = points.GetSize()-1;
       lights.push_back(cur_light);
       cur_light = NULL;
    }
}

LinearFeature::~LinearFeature()
{
    for (unsigned int i=0; i<marks.size(); i++)
    {
        delete marks[i];
    }

    for (unsigned int i=0; i<lights.size(); i++)
    {
        delete lights[i];
    }
}

double LinearFeature::AddMarkingPoly( const SGGeod& prev_inner, const SGGeod& prev_outer, const SGGeod& cur_outer, const SGGeod& cur_inner, std::string material, double width, double v_dist, double heading, double atlas_start, double atlas_end, double v_start, double v_end )
{
    SGGeod prev_mp = midpoint( prev_outer, prev_inner );
    SGGeod cur_mp  = midpoint( cur_outer,  cur_inner  );
    double az2, dist;
    tgPolygon poly;
    
    SGGeodesy::inverse( prev_mp, cur_mp, heading, az2, dist );
    
    v_start = fmod( v_end, 1.0 );
    v_end   = v_start + (dist/v_dist);
    
    TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::Finish: Create TCs for mark.  dist is " << dist << " v_start is " << v_start << " v_end is " << v_end );
    
    poly.Erase();
    poly.AddNode( 0, prev_inner );
    poly.AddNode( 0, prev_outer );
    poly.AddNode( 0, cur_outer  );
    poly.AddNode( 0, cur_inner  );
    poly = tgPolygon::Snap( poly, gSnap );
    
    poly.SetMaterial( material );
    poly.SetTexParams( prev_inner, width, v_dist, heading );
    
    poly.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, 0.0, 1.0, 0.0 );
    poly.SetTexLimits( atlas_start, v_start, atlas_end, v_end );
    
    marking_polys.push_back(poly);

    return v_end;
}

int LinearFeature::Finish( bool closed, double def_width )
{
    tgPolygon   poly;
    SGGeod      prev_inner, prev_outer;
    SGGeod      cur_inner,  cur_outer;
    double      heading;
    double      dist;
    double      az2;
    double      v_dist = 10.0;
    double      v_start = 0.0f;
    double      v_end = 0.0f;
    double      width = 0;
    double      atlas_start = 0.0, atlas_end = 0.0;
    std::string material;
    double      cur_light_dist = 0.0f;
    double      light_delta = 0;
    bool        markStarted;

    #define     ATLAS_SINGLE_WIDTH  (1*0.007812500)
    #define     ATLAS_DOUBLE_WIDTH  (2*0.007812500)
    #define     ATLAS_TRIPLE_WIDTH  (3*0.007812500)
    #define     ATLAS_QUAD_WIDTH    (4*0.007812500)
    
    #define     ATLAS_BUFFER0       (0.000000000)
    #define     ATLAS_BUFFER4       (0.000976563)
    #define     ATLAS_BUFFER8       (0.001953125)
    #define     ATLAS_BUFFER16      (0.003906250)
    #define     ATLAS_BUFFER32      (0.007812500)
    
    #define     ATLAS_BUFFER        ATLAS_BUFFER0

    #define     RW_TEX0_START       (ATLAS_BUFFER)
    #define     RW_TEX0_END         (RW_TEX0_START+ATLAS_QUAD_WIDTH)
    #define     RW_TEX1_START       (RW_TEX0_END+2*ATLAS_BUFFER)
    #define     RW_TEX1_END         (RW_TEX1_START+ATLAS_SINGLE_WIDTH)
    #define     RW_TEX2_START       (RW_TEX1_END+2*ATLAS_BUFFER)
    #define     RW_TEX2_END         (RW_TEX2_START+ATLAS_SINGLE_WIDTH)
        
    
    
    // create the inner and outer boundaries to generate polys
    // this generates 2 point lists for the contours, and remembers
    // the start stop points for markings and lights
    ConvertContour( &contour, closed );

    // now generate the supoerpoly and texparams lists for markings
    for (unsigned int i=0; i<marks.size(); i++)
    {
        markStarted = false;
        
        // which material for this mark?
        switch( marks[i]->type )
        {            
            case LF_NONE:
                break;
                
            // single width lines
            case LF_SOLID_YELLOW: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 0*ATLAS_SINGLE_WIDTH;
                atlas_end   = 1*ATLAS_SINGLE_WIDTH;
                break;

            case LF_BROKEN_YELLOW: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 1*ATLAS_SINGLE_WIDTH;
                atlas_end   = 2*ATLAS_SINGLE_WIDTH;
                break;

            case LF_SINGLE_LANE_QUEUE: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 2*ATLAS_SINGLE_WIDTH;
                atlas_end   = 3*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_B_SOLID_YELLOW: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 3*ATLAS_SINGLE_WIDTH;
                atlas_end   = 4*ATLAS_SINGLE_WIDTH;
                break;

            case LF_B_BROKEN_YELLOW: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 4*ATLAS_SINGLE_WIDTH;
                atlas_end   = 5*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_B_SINGLE_LANE_QUEUE: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 5*ATLAS_SINGLE_WIDTH;
                atlas_end   = 6*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_SOLID_WHITE: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 6*ATLAS_SINGLE_WIDTH;
                atlas_end   = 7*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_BROKEN_WHITE: // good
                material = "taxi_markings";
                width = 0.5f;
                atlas_start = 7*ATLAS_SINGLE_WIDTH;
                atlas_end   = 8*ATLAS_SINGLE_WIDTH;
                break;

            case LF_SOLID_DBL_YELLOW:   // good
                material = "taxi_markings";
                width = 1.0f;
                atlas_start = 8*ATLAS_SINGLE_WIDTH;
                atlas_end =  10*ATLAS_SINGLE_WIDTH;
                break;

            case LF_DOUBLE_LANE_QUEUE: // good
                material = "taxi_markings";
                width = 1.0f;
                atlas_start = 10*ATLAS_SINGLE_WIDTH;
                atlas_end   = 12*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_OTHER_HOLD: // good
                material = "taxi_markings";
                width = 1.0f;
                atlas_start = 12*ATLAS_SINGLE_WIDTH;
                atlas_end   = 14*ATLAS_SINGLE_WIDTH;
                break;

            case LF_B_SOLID_DBL_YELLOW: // good
                material = "taxi_markings";
                width = 1.0f;
                atlas_start = 14*ATLAS_SINGLE_WIDTH;
                atlas_end   = 16*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_B_DOUBLE_LANE_QUEUE:
                material = "taxi_markings";
                width = 1.0f;
                atlas_start = 16*ATLAS_SINGLE_WIDTH;
                atlas_end   = 18*ATLAS_SINGLE_WIDTH;
                break;

            case LF_B_OTHER_HOLD:
                material = "taxi_markings";
                width = 1.0f;
                atlas_start = 18*ATLAS_SINGLE_WIDTH;
                atlas_end   = 20*ATLAS_SINGLE_WIDTH;
                break;                

            case LF_RUNWAY_HOLD:
                material = "taxi_markings";
                width = 2.0f;
                atlas_start = 20*ATLAS_SINGLE_WIDTH;
                atlas_end   = 24*ATLAS_SINGLE_WIDTH;
                break;

            case LF_B_RUNWAY_HOLD:
                material = "taxi_markings";
                width = 2.0f;
                atlas_start = 24*ATLAS_SINGLE_WIDTH;
                atlas_end   = 28*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_ILS_HOLD:
                material = "taxi_markings";
                width = 2.0f;
                atlas_start = 28*ATLAS_SINGLE_WIDTH;
                atlas_end   = 32*ATLAS_SINGLE_WIDTH;
                break;

            case LF_B_ILS_HOLD:
                material = "taxi_markings";
                width = 2.0f;
                atlas_start = 32*ATLAS_SINGLE_WIDTH;
                atlas_end =   36*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_SAFETYZONE_CENTERLINE:
                material = "taxi_markings";
                width = 2.0f;
                atlas_start = 36*ATLAS_SINGLE_WIDTH;
                atlas_end =   40*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_B_SAFETYZONE_CENTERLINE:
                material = "taxi_markings";
                width = 2.00f;
                atlas_start = 40*ATLAS_SINGLE_WIDTH;
                atlas_end   = 44*ATLAS_SINGLE_WIDTH;
                break;
                
            case LF_CHECKERBOARD_WHITE:
                material = "taxi_markings";
                width = 1.0f;
                atlas_start = 44*ATLAS_SINGLE_WIDTH;
                atlas_end   = 48*ATLAS_SINGLE_WIDTH;
                break;    

            case RWY_BORDER:
                material = "rwy_markings";
                width = 0.9144; // 36 inches
                atlas_start = RW_TEX1_START;
                atlas_end   = RW_TEX1_END;
                break;
                
            case RWY_DISP_TAIL:
                material = "rwy_markings";
                width = 0.4572; // 18 inches
                atlas_start = RW_TEX1_START;
                atlas_end   = RW_TEX1_END;
                break;
                
            case RWY_CENTERLINE:
                material = "rwy_markings";
                width  = 0.9144; // 36 inches
                v_dist = 200 * SG_FEET_TO_METER;
                atlas_start = RW_TEX2_START;
                atlas_end   = RW_TEX2_END;
                
                break;
                
            case RWY_THRESH:
                material = "rwy_markings";
                width = 1.7526; // 5.75 ft
                atlas_start = RW_TEX0_START;
                atlas_end   = RW_TEX0_END;
                
                v_end = (double)rand()/(double)RAND_MAX;
                break;

            case RWY_TZONE:
                material = "rwy_markings";
                width = 1.2192; // 48 inches ft
                atlas_start = RW_TEX0_START;
                atlas_end   = RW_TEX0_END;
                
                v_end = (double)rand()/(double)RAND_MAX;
                break;

            case RWY_AIM:
                material = "rwy_markings";
                width = 6.096; // 20 ft
                atlas_start = RW_TEX0_START;
                atlas_end   = RW_TEX0_END;
                
                v_end = (double)rand()/(double)RAND_MAX;
                break;
                              
            
            default:
                TG_LOG(SG_GENERAL, SG_ALERT, "LinearFeature::Finish: unknown marking " << marks[i]->type );
                exit(1);
        }
        
        for (unsigned int j = marks[i]->start_idx; j <= marks[i]->end_idx; j++)
        {
            // for each point on the PointsList, generate a quad from
            // start to next, offset by 2 distnaces from the edge

            if (j == marks[i]->start_idx)
            {
                // first point on the mark - offset heading is 90deg
                cur_outer = OffsetPointFirst( points.GetNode(j), points.GetNode(j+1), offset-width/2.0f );
                cur_inner = OffsetPointFirst( points.GetNode(j), points.GetNode(j+1), offset+width/2.0f );
            }
            else if (j == marks[i]->end_idx)
            {
                // last point on the mark - offset heading is 90deg
                cur_outer = OffsetPointLast( points.GetNode(j-1), points.GetNode(j), offset-width/2.0f );
                cur_inner = OffsetPointLast( points.GetNode(j-1), points.GetNode(j), offset+width/2.0f );
            }
            else
            {
                cur_outer = OffsetPointMiddle( points.GetNode(j-1), points.GetNode(j), points.GetNode(j+1), offset-width/2.0f );
                cur_inner = OffsetPointMiddle( points.GetNode(j-1), points.GetNode(j), points.GetNode(j+1), offset+width/2.0f );
            }

            
            // Linear feature texturing
            //
            // u is clipped based on the texture atlas.
            // The atlas contains vertical stripes, so only u is looked up.
            // v sarts at 0.0 at the beginning of a mark, and the end v is calculated.  
            // The continuing marks start v is based on the previous poly's end v 
            //
            // Diagramed below:
            //
            //
            // poly 1
            // length  = 48.75
            // v_dist (v=0,1) = 10.0
            // v_start = 0.000
            // v_end   = 4.875
            
            // 00.00   10.00   20.00   30.00   40.00   48.75
            // 0       1       2       3       4       4.875
            
            // poly 2
            // length  = 24.93
            // v_dist  = 10.0
            // v_start = 0.875 : fmod( prev v_end, 1.0 )
            // v_end   = 0.875 + 2.493 = 3.368
            
            // 08.75   10.00   20.00   30.00   33.68 
            // 0.875   1       2       3       3.368       
            
            if ( markStarted )
            {
                v_end = AddMarkingPoly( prev_inner, prev_outer, cur_outer, cur_inner, material, width, v_dist, heading, atlas_start, atlas_end, v_start, v_end );
            } else {
                markStarted = true;
            }

            prev_outer = cur_outer;
            prev_inner = cur_inner;
        }
    }

    // now generate the superpoly list for lights with constant distance between lights (depending on feature type)
    tglightcontour_list light_contours;
    tgLightContour      cur_light_contour;
    tgLightContour      alt_light_contour;

    for (unsigned int i=0; i<lights.size(); i++)
    {
        markStarted = false;
        cur_light_dist = 0.0f;
        bool directional_light = lights[i]->IsDirectional();
        bool alternate = false;

        // which material for this light
        switch( lights[i]->type )
        {
            case LF_BIDIR_GREEN:
                cur_light_contour.SetType( "RWY_GREEN_TAXIWAY_LIGHTS" );
                light_delta = 10.0f;
                break;

            case LF_OMNIDIR_BLUE:
                cur_light_contour.SetType( "RWY_BLUE_TAXIWAY_LIGHTS" );
                light_delta = 10.0f;
                break;

            case LF_UNIDIR_CLOSE_AMBER:
                cur_light_contour.SetType( "RWY_YELLOW_LIGHTS" );
                light_delta = 2.0f;
                break;

            case LF_UNIDIR_CLOSE_AMBER_PULSE:
                cur_light_contour.SetType( "RWY_YELLOW_PULSE_LIGHTS" );
                light_delta = 2.0f;
                break;

            case LF_BIDIR_GREEN_AMBER:
                cur_light_contour.SetType( "RWY_GREEN_TAXIWAY_LIGHTS" );
                alt_light_contour.SetType( "RWY_YELLOW_LIGHTS" );
                light_delta = 10.0f;
                alternate = true;
                break;

            case LF_OMNIDIR_RED:
                cur_light_contour.SetType( "RWY_RED_LIGHTS" );
                light_delta = 10.0f;
                break;
        }

        for (unsigned int j = lights[i]->start_idx; j <= lights[i]->end_idx; j++)
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::Finish: calculating offsets for light " << i << " whose start idx is " << lights[i]->start_idx << " and end idx is " << lights[i]->end_idx << " cur idx is " << j );
            // for each point on the PointsList, offset by 2 distnaces from the edge, and add a point to the superpoly contour
            if (j == lights[i]->start_idx)
            {
                // first point on the light - offset heading is 90deg
                cur_outer = OffsetPointFirst( points.GetNode(j), points.GetNode(j+1), offset );
            }
            else if (j == lights[i]->end_idx)
            {
                // last point on the mark - offset heading is 90deg
                cur_outer = OffsetPointLast( points.GetNode(j-1), points.GetNode(j), offset );
            }
            else
            {
                cur_outer = OffsetPointMiddle( points.GetNode(j-1), points.GetNode(j), points.GetNode(j+1), offset );
            }

            if ( markStarted )
            {
                SGGeod tmp;
                bool   switch_poly = true;

                // calculate the heading and distance from prev to cur
                SGGeodesy::inverse( prev_outer, cur_outer, heading, az2, dist );

                while (cur_light_dist < dist)
                {
                    if (cur_light_dist == 0.0f)
                    {
                        tmp = prev_outer;
                    }
                    else
                    {
                        // calculate the position of the next light
                        tmp = SGGeodesy::direct( prev_outer, heading, cur_light_dist );
                    }

                    SGVec3f vec;
                    if ( !directional_light )
                    {
                        // calculate the omnidirectional normal
                        vec = normalize(SGVec3f::fromGeod(tmp));
                    } else
                    {
                        // calculate the directional normal
                        double heading_vec = SGMiscd::normalizePeriodic( 0, 360, heading + 90.0 );
                        SGVec3f cart1 = SGVec3f::fromGeod(tmp);
                        SGVec3f cart2 = SGVec3f::fromGeod( SGGeodesy::direct( tmp, heading_vec, 10 ) );
                        vec = normalize(cart2 - cart1);
                    }

                    if (!alternate)
                    {
                        cur_light_contour.AddLight( tmp, vec );
                    }
                    else
                    {
                        if (switch_poly)
                        {
                            cur_light_contour.AddLight( tmp, vec );
                        }
                        else
                        {
                            alt_light_contour.AddLight( tmp, vec );
                        }
                        switch_poly = !switch_poly;
                    }

                    // update current light distance
                    cur_light_dist += light_delta;
                }

                // start next segment at the correct distance
                cur_light_dist = cur_light_dist - dist;
            } else {
                markStarted = true;
            }

            prev_outer = cur_outer;
        }

        // if there were lights generated - create the superpoly
        if (cur_light_contour.ContourSize())
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature::Finish: Adding light contour with " << cur_light_contour.ContourSize() << " lights" );
            cur_light_contour.SetFlag("");
            lighting_polys.push_back(cur_light_contour);
        }

        // create the superpoly for the alternating light color
        if (alt_light_contour.ContourSize())
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature::Finish: Adding light contour with " << cur_light_contour.ContourSize() << " lights" );
            alt_light_contour.SetFlag("");
            lighting_polys.push_back(cur_light_contour);
        }
        else
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature::Finish: No points for linear feature " << description << " light index " << i );
        }
    }

    return 1;
}

void LinearFeature::GetPolys( tgpolygon_list& polys )
{
    for ( unsigned int i = 0; i < marking_polys.size(); i++)
    {
        polys.push_back( marking_polys[i] );
    }    
}

void LinearFeature::GetLights( tglightcontour_list& lights )
{
    for ( unsigned i = 0; i < lighting_polys.size(); i++)
    {
        lights.push_back( lighting_polys[i] );
    }
}