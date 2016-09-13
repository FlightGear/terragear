#include <stdlib.h>

#include <terragear/tg_shapefile.hxx>


#include "global.hxx"
#include "beznode.hxx"
#include "linearfeature.hxx"
#include "airport.hxx"

void LinearFeature::ConvertContour( BezContour* src, bool closed )
{
    BezNode*  curNode;
    BezNode*  nextNode;

    SGGeod    curLoc;
    SGGeod    nextLoc;
    SGGeod    cp1;
    SGGeod    cp2;
    unsigned int last_node;
    
    int       curve_type = CURVE_LINEAR;
    double    total_dist;
    double    theta1, theta2;
    int       num_segs = BEZIER_DETAIL;
    
    Marking*  cur_mark = NULL;
    Lighting* cur_light = NULL;

    TG_LOG(SG_GENERAL, SG_DEBUG, " LinearFeature::ConvertContour - Creating a contour with " << src->size() << " nodes");

    // clear anything in the point list
    points.clear();
    
    if ( closed ) {
        last_node = src->size()-1;
    } else {
        last_node = src->size()-2;
    }
    
    // iterate through each bezier node in the contour
    for (unsigned int i=0; i <= last_node; i++)
    {
        curNode = src->at(i);

        if (i < last_node)
        {
            nextNode = src->at(i+1);
        }
        else if ( closed )
        {
            // for the last node, next is the first. as all contours are closed
            nextNode = src->at(0);
        } 
        else
        {
            // last node is actuall 1 past this...
            nextNode = src->at(i+1);
        }
        
        ////////////////////////////////////////////////////////////////////////////////////
        // remember the index for the starting stopping of marks on the converted contour

        // are we watching a mark for the end?
        if (cur_mark)
        {
            if (curNode->GetMarking() != cur_mark->type)
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking has changed from " << cur_mark->type << " to " << curNode->GetMarking() << " save mark from " << cur_mark->start_idx << " to " << points.size() );

                // marking has ended, or changed
                cur_mark->end_idx = points.size();
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
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Start Marking from " << points.size() << " with type " << curNode->GetMarking() );

                // we aren't watching a mark, and this node has one
                cur_mark = new Marking;
                cur_mark->type = curNode->GetMarking();
                cur_mark->start_idx = points.size();
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////
        // remember the index for the starting stopping of lights on the converted contour

        // are we watching a mark for the end?
        if (cur_light)
        {
            if (curNode->GetLighting() != cur_light->type)
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Lighting has changed from " << cur_light->type << " to " << curNode->GetLighting() << " save light from " << cur_light->start_idx << " to " << points.size() );

                // lighting has ended, or changed : add final light
                cur_light->end_idx = points.size();
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
                TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Start Lighting from " << points.size() << " with type " << curNode->GetLighting() );

                // we aren't watching a mark, and this node has one
                cur_light = new Lighting;
                cur_light->type = curNode->GetLighting();
                cur_light->start_idx = points.size();
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
            if ( (std::abs(theta1 - 180.0) < 5.0 ) || (std::abs(theta1) < 5.0 ) || (std::isnan(theta1)) )
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
            if ( (std::abs(theta1 - 180.0) < 5.0 ) || (std::abs(theta1) < 5.0 ) || (std::isnan(theta1)) )
            {
                TG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Cubic curve with cp1 in line : " << description << ": theta is " << theta1 );

                if ( (std::abs(theta2 - 180.0) < 5.0 ) || (std::abs(theta2) < 5.0 ) || (std::isnan(theta2)) )
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

                if ( (std::abs(theta2 - 180.0) < 5.0 ) || (std::abs(theta2) < 5.0 ) || (std::isnan(theta2)) )
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
                points.push_back( cgalPoly_Point( curLoc.getLongitudeDeg(), curLoc.getLatitudeDeg() ) );

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
                    points.push_back( cgalPoly_Point( curLoc.getLongitudeDeg(), curLoc.getLatitudeDeg() ) );

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
                points.push_back( cgalPoly_Point( curLoc.getLongitudeDeg(), curLoc.getLatitudeDeg() ) );

                TG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear Anchor node at " << curLoc );

                curLoc = nextLoc;
            }
        }
    }

    // need to add the markings for last segment
    TG_LOG(SG_GENERAL, SG_DEBUG, "Closed COntour : adding last node at " << curLoc );
    points.push_back( cgalPoly_Point( curLoc.getLongitudeDeg(), curLoc.getLatitudeDeg() ) );

    // check for marking that goes all the way to the end...
    if (cur_mark)
    {
       TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking from " << cur_mark->start_idx << " with type " << cur_mark->type << " ends at the end of the contour: " << points.size() );

       cur_mark->end_idx = points.size()-1;
       marks.push_back(cur_mark);
       cur_mark = NULL;
    }

    // check for lighting that goes all the way to the end...
    if (cur_light)
    {
       TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Lighting from " << cur_light->start_idx << " with type " << cur_light->type << " ends at the end of the contour: " << points.size() );

       cur_light->end_idx = points.size()-1;
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

double LinearFeature::GetWidth( unsigned int type )
{
    double width = 0.0f;
    
    switch( type )
    {            
        case LF_NONE:
            break;
            
            // single width lines
        case LF_SOLID_YELLOW: // good
        case LF_BROKEN_YELLOW: // good
        case LF_SINGLE_LANE_QUEUE: // good
        case LF_SOLID_WHITE: // good
        case LF_BROKEN_WHITE: // good
            width = 0.30f;  // 0.15 is too narrow - looks like OSG drops them
            break;
            
        case LF_B_SOLID_YELLOW: // good
        case LF_B_BROKEN_YELLOW: // good
        case LF_B_SINGLE_LANE_QUEUE: // good
            width = 0.45f;
            break;
            
        case LF_SOLID_DBL_YELLOW:   // good
        case LF_DOUBLE_LANE_QUEUE: // good
            width = 0.45f;
            break;
            
        case LF_OTHER_HOLD: // good
            width = 0.45f;
            break;
            
        case LF_B_SOLID_DBL_YELLOW: // good
        case LF_B_DOUBLE_LANE_QUEUE:
        case LF_B_OTHER_HOLD:
            width = 0.75f;
            break;                

        case LF_CHECKERBOARD_WHITE:
            width = 1.2f;
            break;
            
        case LF_RUNWAY_HOLD:
            width = 2.1f;
            break;
            
        case LF_B_RUNWAY_HOLD:
            width = 2.4f;
            break;
            
        case LF_ILS_HOLD:
            width = 2.4f;
            break;
            
        case LF_B_ILS_HOLD:
            width = 2.7f;
            break;
            
        case LF_SAFETYZONE_CENTERLINE:
            width = 0.75f;
            break;
            
        case LF_B_SAFETYZONE_CENTERLINE:
            width = 1.05f;
            break;
            
        case RWY_BORDER:
            width = 0.9144; // 36 inches
            break;
            
        case RWY_DISP_TAIL:
            width = 0.4572; // 18 inches
            break;
            
        case RWY_CENTERLINE:
            width  = 0.9144; // 36 inches            
            break;
            
        case RWY_THRESH:
            width = 1.7526; // 5.75 ft
            break;
            
        case RWY_TZONE:
            width = 1.2192; // 48 inches ft
            break;
            
        case RWY_AIM:
            width = 6.096; // 20 ft
            break;
                        
        default:
            TG_LOG(SG_GENERAL, SG_ALERT, "LinearFeature::Finish: unknown marking " << type );
            exit(1);
    }

    return width;
}

// TODO: make this a static function so I can register it as a callback
int LinearFeature::GetTextureInfo( unsigned int type, bool cap, std::string& material, double& atlas_startu, double& atlas_endu, double& atlas_startv, double& atlas_endv, double& v_dist )
{
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
    #define     RW_TEX1_START       (RW_TEX0_END)
    #define     RW_TEX1_END         (RW_TEX1_START+ATLAS_SINGLE_WIDTH)
    #define     RW_TEX2_START       (RW_TEX1_END)
    #define     RW_TEX2_END         (RW_TEX2_START+ATLAS_SINGLE_WIDTH)
        
    #define     pix4                (4/4096)
    
#define ATLAS_WIDTH                 (1024.0l);
    
#define TEST 0
    //cap = false;
    
    switch( type )
    {            
        case LF_NONE:
            break;
            
        case LF_SOLID_YELLOW:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 4.0l/ATLAS_WIDTH;
                atlas_endu   = 12.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 4.0l/ATLAS_WIDTH;
                atlas_endu   = 12.0l/ATLAS_WIDTH;
                
                v_dist = 2.0;   // 0.15 m wide x 2 m long
            }
            break;
            
        case LF_BROKEN_YELLOW:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 20.0l/ATLAS_WIDTH;
                atlas_endu   = 28.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 20.0l/ATLAS_WIDTH;
                atlas_endu   = 28.0l/ATLAS_WIDTH;

                v_dist = 2.0;   // 0.15 m wide x 2 m long
            }
            break;
            
        case LF_SINGLE_LANE_QUEUE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 36.0l/ATLAS_WIDTH;
                atlas_endu   = 44.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 36.0l/ATLAS_WIDTH;
                atlas_endu   = 44.0l/ATLAS_WIDTH;

                v_dist = 12.0;   // 0.15 m wide x 12 m long
            }
            break;
            
        case LF_B_SOLID_YELLOW:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 116.0l/ATLAS_WIDTH;
                atlas_endu   = 140.0l/ATLAS_WIDTH;
                
                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 52.0l/ATLAS_WIDTH;
                atlas_endu   = 76.0l/ATLAS_WIDTH;
                
                v_dist = 2.0;   // 0.45 m wide x 2 m long
            }
            break;
            
        case LF_B_BROKEN_YELLOW:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 116.0l/ATLAS_WIDTH;
                atlas_endu   = 140.0l/ATLAS_WIDTH;
                
                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 84.0l/ATLAS_WIDTH;
                atlas_endu   = 108.0l/ATLAS_WIDTH;
                
                v_dist = 2.0;   // 0.45 m wide x 2 m long
            }
            break;
            
        case LF_B_SINGLE_LANE_QUEUE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 116.0l/ATLAS_WIDTH;
                atlas_endu   = 140.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 116.0l/ATLAS_WIDTH;
                atlas_endu   = 140.0l/ATLAS_WIDTH;

                v_dist = 12.0;   // 0.45 m wide x 12 m long
            }
            break;
            
        case LF_SOLID_WHITE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 148.0l/ATLAS_WIDTH;
                atlas_endu   = 156.0l/ATLAS_WIDTH;
                
                atlas_startv = 0.125l;
                atlas_endv   = 0.25l;
            } else {
                atlas_startu = 148.0l/ATLAS_WIDTH;
                atlas_endu   = 156.0l/ATLAS_WIDTH;

                v_dist = 2.0;   // 0.15 m wide x 2 m long
            }
            break;
            
        case LF_BROKEN_WHITE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 164.0l/ATLAS_WIDTH;
                atlas_endu   = 172.0l/ATLAS_WIDTH;

                atlas_startv = 0.125l;
                atlas_endv   = 0.25l;
            } else {
                atlas_startu = 164.0l/ATLAS_WIDTH;
                atlas_endu   = 172.0l/ATLAS_WIDTH;

                v_dist = 2.0;   // 0.15 m wide x 2 m long
            }
            break;
            
        case LF_SOLID_DBL_YELLOW:   // good
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 204.0l/ATLAS_WIDTH;
                atlas_endu   = 228.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 204.0l/ATLAS_WIDTH;
                atlas_endu   = 228.0l/ATLAS_WIDTH;

                v_dist = 2.0;   // 0.45 m wide x 2 m long
            }
            break;
            
        case LF_DOUBLE_LANE_QUEUE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 236.0l/ATLAS_WIDTH;
                atlas_endu   = 260.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 236.0l/ATLAS_WIDTH;
                atlas_endu   = 260.0l/ATLAS_WIDTH;

                v_dist = 12.0;   // 0.45 m wide x 12 m long
            }
            break;
            
        case LF_OTHER_HOLD:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 268.0l/ATLAS_WIDTH;
                atlas_endu   = 292.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 268.0l/ATLAS_WIDTH;
                atlas_endu   = 292.0l/ATLAS_WIDTH;

                v_dist = 6.0;   // 1 m wide x 6 m long
            }
            break;
            
        case LF_B_SOLID_DBL_YELLOW: // good
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 300.0l/ATLAS_WIDTH;
                atlas_endu   = 340.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 300.0l/ATLAS_WIDTH;
                atlas_endu   = 340.0l/ATLAS_WIDTH;

                v_dist = 2.0;   // 0.75 m wide x 2 m long
            }
            break;
            
        case LF_B_DOUBLE_LANE_QUEUE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 348.0l/ATLAS_WIDTH;
                atlas_endu   = 388.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 348.0l/ATLAS_WIDTH;
                atlas_endu   = 388.0l/ATLAS_WIDTH;

                v_dist = 12.0;   // 0.75 m wide x 12 m long
            }
            break;
            
        case LF_B_OTHER_HOLD:
            material = "taxi_markings";
            if ( cap ) {
                // use double lane queue for cap.
                atlas_startu = 348.0l/ATLAS_WIDTH;
                atlas_endu   = 388.0l/ATLAS_WIDTH;
                
                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 396.0l/ATLAS_WIDTH;
                atlas_endu   = 436.0l/ATLAS_WIDTH;

                v_dist = 6.0;   // 1.3 m wide x 6 m long
            }
            break;                
            
        case LF_RUNWAY_HOLD:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 444.0l/ATLAS_WIDTH;
                atlas_endu   = 500.0l/ATLAS_WIDTH;

                atlas_startv = 0.0l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 444.0l/ATLAS_WIDTH;
                atlas_endu   = 500.0l/ATLAS_WIDTH;

                v_dist = 1.8;   // 2.1 m wide x 1.8 m long
            }
            break;
            
        case LF_B_RUNWAY_HOLD:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 588.0l/ATLAS_WIDTH;
                atlas_endu   = 660.0l/ATLAS_WIDTH;
                
                atlas_startv = 0.05l;
                atlas_endv   = 0.125l;
            } else {
                atlas_startu = 508.0l/ATLAS_WIDTH;
                atlas_endu   = 580.0l/ATLAS_WIDTH;

                v_dist = 1.8;   // 2.1 m wide x 1.8 m long
            }
            break;
            
        case LF_ILS_HOLD:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 668.0l/ATLAS_WIDTH;
                atlas_endu   = 700.0l/ATLAS_WIDTH;

                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = 668.0l/ATLAS_WIDTH;
                atlas_endu   = 700.0l/ATLAS_WIDTH;

                v_dist = 6.0;   // 2.4 m wide x 3.0 m long
            }
            break;
            
        case LF_B_ILS_HOLD:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 708.0l/ATLAS_WIDTH;
                atlas_endu   = 744.0l/ATLAS_WIDTH;

                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = 708.0l/ATLAS_WIDTH;
                atlas_endu   = 744.0l/ATLAS_WIDTH;

                v_dist = 6.0;   // 2.4 m wide x 3.0 m long ( 2 lengths to preserve aspect ratio in texture atlas )
            }
            break;
            
        case LF_SAFETYZONE_CENTERLINE: // note - these should ALWAYS be black...
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 752.0l/ATLAS_WIDTH;
                atlas_endu   = 800.0l/ATLAS_WIDTH;

                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = 752.0l/ATLAS_WIDTH;
                atlas_endu   = 800.0l/ATLAS_WIDTH;

                v_dist = 3.64;   // 1.2 m wide x 3.64 m long
            }
            break;
            
        case LF_B_SAFETYZONE_CENTERLINE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 588.0l/ATLAS_WIDTH;
                atlas_endu   = 644.0l/ATLAS_WIDTH;
                
                atlas_startv = 0.25f;
                atlas_endv   = 0.375f;
            } else {
                atlas_startu = 808.0l/ATLAS_WIDTH;
                atlas_endu   = 864.0l/ATLAS_WIDTH;

                v_dist = 3.64;   // 1.2 m wide x 3.64 m long
            }
            break;
            
        case LF_CHECKERBOARD_WHITE:
            material = "taxi_markings";
            if ( cap ) {
                atlas_startu = 180.0l/ATLAS_WIDTH;
                atlas_endu   = 196.0l/ATLAS_WIDTH;

                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = 180.0l/ATLAS_WIDTH;
                atlas_endu   = 196.0l/ATLAS_WIDTH;

                v_dist = 3.64;   // 0.6 m wide x 2.6 m long
            }
            break;    
            
        case RWY_BORDER:
            material = "rwy_markings";
            if ( cap ) {
                atlas_startu = 28*ATLAS_SINGLE_WIDTH;
                atlas_endu   = 29*ATLAS_SINGLE_WIDTH;
                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = RW_TEX1_START;
                atlas_endu   = RW_TEX1_END;
                v_dist = 10.0;
            }
            break;
            
        case RWY_DISP_TAIL:
            material = "rwy_markings";
            if ( cap ) {
                atlas_startu = 28*ATLAS_SINGLE_WIDTH;
                atlas_endu   = 29*ATLAS_SINGLE_WIDTH;
                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = RW_TEX1_START;
                atlas_endu   = RW_TEX1_END;
                v_dist = 10.0;
            }
            break;
            
        case RWY_CENTERLINE:
            material = "rwy_markings";
            if ( cap ) {
                atlas_startu = 28*ATLAS_SINGLE_WIDTH;
                atlas_endu   = 29*ATLAS_SINGLE_WIDTH;
                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                v_dist = 200 * SG_FEET_TO_METER;
                atlas_startu = RW_TEX2_START;
                atlas_endu   = RW_TEX2_END;
                v_dist = 10.0;
            }
            break;
            
        case RWY_THRESH:
            material = "rwy_markings";
            if ( cap ) {
                atlas_startu = 28*ATLAS_SINGLE_WIDTH;
                atlas_endu   = 29*ATLAS_SINGLE_WIDTH;
                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = RW_TEX0_START;
                atlas_endu   = RW_TEX0_END;
                v_dist = 10.0;
            }
            break;
            
        case RWY_TZONE:
            material = "rwy_markings";
            if ( cap ) {
                atlas_startu = 28*ATLAS_SINGLE_WIDTH;
                atlas_endu   = 29*ATLAS_SINGLE_WIDTH;
                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = RW_TEX0_START;
                atlas_endu   = RW_TEX0_END;
                v_dist = 10.0;
            }
            break;
            
        case RWY_AIM:
            material = "rwy_markings";
            if ( cap ) {
                atlas_startu = 28*ATLAS_SINGLE_WIDTH;
                atlas_endu   = 29*ATLAS_SINGLE_WIDTH;
                atlas_startv = 0.0f;
                atlas_endv   = 0.125f;
            } else {
                atlas_startu = RW_TEX0_START;
                atlas_endu   = RW_TEX0_END;
                v_dist = 10.0;
            }
            break;

        default:
            TG_LOG(SG_GENERAL, SG_ALERT, "LinearFeature::Finish: unknown marking " << type );
            exit(1);
    }

#if TEST
    material = "lftest";
    atlas_startu = 0;
    atlas_endu   = 1;
#endif
    
    return 0;
}

int LinearFeature::Finish( Airport* ap, bool closed, double def_width )
{
    SGGeod      prev, cur;
    double      heading;
    double      dist;
    double      az2;
    std::string material;
    double      cur_light_dist = 0.0f;
    double      light_delta = 0;
    bool        markStarted = false;        
    
    // create the inner and outer boundaries to generate polys
    // this generates 2 point lists for the contours, and remembers
    // the start stop points for markings and lights
    ConvertContour( &contour, closed );

    // now offset the markings and add them to the intersection generator
    for (unsigned int i=0; i<marks.size(); i++)
    {
        double edge_width = GetWidth( marks[i]->type );
        tgIntersectionGenerator* pig = ap->GetLFG( marks[i]->type );        
        markStarted = false;
        
        for (unsigned int j = marks[i]->start_idx; j <= marks[i]->end_idx; j++)
        {
            TG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::Finish: calculating offsets for mark " << i << " whose start idx is " << marks[i]->start_idx << " and end idx is " << marks[i]->end_idx << " cur idx is " << j );

            if (j == marks[i]->start_idx)
            {
                // first point on the mark - offset heading is 90deg
                cur = OffsetPointFirst( points[j], points[j+1], offset );
            }
            else if (j == marks[i]->end_idx)
            {
                // last point on the mark - offset heading is 90deg
                cur = OffsetPointLast( points[j-1], points[j], offset );
            }
            else
            {
                cur = OffsetPointMiddle( points[j-1], points[j], points[j+1], offset );
            }

            if ( markStarted ) {
                if ( pig ) {
                    pig->Insert( prev, cur, edge_width, 0, marks[i]->type );
                }
            } else {
                markStarted = true;
            }

            prev = cur;
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
        int light_direction = lights[i]->LightDirection();
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
                cur = OffsetPointFirst( points[j], points[j+1], offset );
            }
            else if (j == lights[i]->end_idx)
            {
                // last point on the mark - offset heading is 90deg
                cur = OffsetPointLast( points[j-1], points[j], offset );
            }
            else
            {
                cur = OffsetPointMiddle( points[j-1], points[j], points[j+1], offset );
            }

            if ( markStarted )
            {
                SGGeod tmp;
                bool   switch_poly = true;

                // calculate the heading and distance from prev to cur
                SGGeodesy::inverse( prev, cur, heading, az2, dist );

                while (cur_light_dist < dist)
                {
                    if (cur_light_dist == 0.0f)
                    {
                        tmp = prev;
                    }
                    else
                    {
                        // calculate the position of the next light
                        tmp = SGGeodesy::direct( prev, heading, cur_light_dist );
                    }

                    SGVec3f vec1, vec2;
                    if ( light_direction == 0)
                    {
                        // calculate the omnidirectional normal
                        vec1 = normalize(SGVec3f::fromGeod(tmp));
                    } else if ( light_direction == 1)
                    {
                        // calculate the directional normal. These lights all face to the right
                        double heading_vec = SGMiscd::normalizePeriodic( 0, 360, heading + 90.0 );
                        SGVec3f cart1 = SGVec3f::fromGeod(tmp);
                        SGVec3f cart2 = SGVec3f::fromGeod( SGGeodesy::direct( tmp, heading_vec, 10 ) );
                        vec1 = normalize(cart2 - cart1);
                    } else //( light_direction == 2)
                    {
                        // calculate the directional normals for bidirectional lights
                        SGVec3f cart1 = SGVec3f::fromGeod(tmp);
                        SGVec3f cart2 = SGVec3f::fromGeod( SGGeodesy::direct( tmp, heading, 10 ) );
                        vec1 = normalize(cart2 - cart1);
                        cart2 = SGVec3f::fromGeod( SGGeodesy::direct( tmp, heading, -10 ) );
                        vec2 = normalize(cart2 - cart1);
                    }

                    if (!alternate)
                    {
                        cur_light_contour.AddLight( tmp, vec1 );
                        if ( light_direction == 2)
                        {
                            cur_light_contour.AddLight( tmp, vec2 );
                        }
                    }
                    else
                    {
                        if (switch_poly)
                        {
                            cur_light_contour.AddLight( tmp, vec1 );
                            if ( light_direction == 2)
                            {
                                cur_light_contour.AddLight( tmp, vec2 );
                            }
                        }
                        else
                        {
                            alt_light_contour.AddLight( tmp, vec1 );
                            if ( light_direction == 2)
                            {
                                alt_light_contour.AddLight( tmp, vec2 );
                            }
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

            prev = cur;
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

tgPolygonSetList& LinearFeature::GetPolys( void )
{
    return marking_polys;
}

tgPolygonSetList& LinearFeature::GetCapPolys( void )
{
    return cap_polys;
}

void LinearFeature::GetLights( tglightcontour_list& lights )
{
    for ( unsigned i = 0; i < lighting_polys.size(); i++)
    {
        lights.push_back( lighting_polys[i] );
    }
}
