#include <stdlib.h>

#include "global.hxx"
#include "beznode.hxx"
#include "linearfeature.hxx"

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

    SG_LOG(SG_GENERAL, SG_DEBUG, " LinearFeature::ConvertContour - Creating a contour with " << src->size() << " nodes");

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
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking has changed from " << cur_mark->type << " to " << curNode->GetMarking() << " save mark from " << cur_mark->start_idx << " to " << points.GetSize() );

                // marking has ended, or changed
                cur_mark->end_idx = points.GetSize();
                marks.push_back(cur_mark);
                cur_mark = NULL;
            }
            else
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Continue Marking from " << cur_mark->start_idx << " with type " << cur_mark->type );
            }
        }

        // should we start a new mark?
        if (cur_mark == NULL)
        {
            if (curNode->GetMarking())
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Start Marking from " << points.GetSize() << " with type " << curNode->GetMarking() );

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
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Lighting has changed from " << cur_light->type << " to " << curNode->GetLighting() << " save light from " << cur_light->start_idx << " to " << points.GetSize() );

                // lighting has ended, or changed : add final light
                cur_light->end_idx = points.GetSize();
                lights.push_back(cur_light);
                cur_light = NULL;
            }
            else
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Continue Lighting from " << cur_light->start_idx << " with type " << cur_light->type );
            }
        }

        // should we start a new light?
        if (cur_light == NULL)
        {
            if (curNode->GetLighting())
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Start Lighting from " << points.GetSize() << " with type " << curNode->GetLighting() );

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
                SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Quadtratic curve with cp in line : convert to linear: " << description << ": theta is " << theta1 );
                curve_type = CURVE_LINEAR;
            }
            else
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Quadtratic curve withOUT cp in line : keep quadtratic: " << description << ": theta is " << theta1 );
            }
        }

        if ( curve_type == CURVE_CUBIC )
        {
            if ( (abs(theta1 - 180.0) < 5.0 ) || (abs(theta1) < 5.0 ) || (isnan(theta1)) )
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Cubic curve with cp1 in line : " << description << ": theta is " << theta1 );

                if ( (abs(theta2 - 180.0) < 5.0 ) || (abs(theta2) < 5.0 ) || (isnan(theta2)) )
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "\n               and cp2 in line : " << description << ": theta is " << theta2 << " CONVERTING TO LINEAR" );

                    curve_type = CURVE_LINEAR;
                }
                else
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "\n               BUT cp2 NOT in line : " << description << ": theta is " << theta2 );
                }
            }
            else
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature: Cubic curve withOUT cp1 in line : keep quadtratic: " << description << ": theta is " << theta1 );

                if ( (abs(theta2 - 180.0) < 5.0 ) || (abs(theta2) < 5.0 ) || (isnan(theta2)) )
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "\n               BUT cp2 IS in line : " << description << ": theta is " << theta2 );
                }
                else
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "\n               AND cp2 NOT in line : " << description << ": theta is " << theta2 );
                }
            }
        }

        if (total_dist < 4.0f)
        {
            if (curve_type != CURVE_LINEAR)
            {
                // If total distance is < 4 meters, then we need to modify num Segments so that each segment >= 1/2 meter
                num_segs = ((int)total_dist + 1) * 2;
                SG_LOG(SG_GENERAL, SG_DEBUG, "Segment from " << curNode->GetLoc() << " to " << nextNode->GetLoc() );
                SG_LOG(SG_GENERAL, SG_DEBUG, "        Distance is " << total_dist << " ( < 4.0) so num_segs is " << num_segs );
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
            SG_LOG(SG_GENERAL, SG_DEBUG, "Segment from " << curNode->GetLoc() << " to " << nextNode->GetLoc() );
            SG_LOG(SG_GENERAL, SG_DEBUG, "        Distance is " << total_dist << " ( > 100.0) so num_segs is " << num_segs );
        }
        else
        {
            if (curve_type != CURVE_LINEAR)
            {
                num_segs = 8;
                SG_LOG(SG_GENERAL, SG_DEBUG, "Segment from " << curNode->GetLoc() << " to " << nextNode->GetLoc() );
                SG_LOG(SG_GENERAL, SG_DEBUG, "        Distance is " << total_dist << " (OK) so num_segs is " << num_segs );
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
                    SG_LOG(SG_GENERAL, SG_DEBUG, "adding Curve Anchor node (type " << curve_type << ") at " << curLoc );
                }
                else
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "   add bezier node (type  " << curve_type << ") at " << curLoc );
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
                        SG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear anchor node at " << curLoc );
                    }
                    else
                    {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "   add linear node at " << curLoc );
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

                SG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear Anchor node at " << curLoc );

                curLoc = nextLoc;
            }
        }
    }

    if (closed)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Closed COntour : adding last node at " << curLoc );

        // need to add the markings for last segment
        points.AddNode(curLoc);
    }

    // check for marking that goes all the way to the end...
    if (cur_mark)
    {
       SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking from " << cur_mark->start_idx << " with type " << cur_mark->type << " ends at the end of the contour: " << points.GetSize() );

       cur_mark->end_idx = points.GetSize()-1;
       marks.push_back(cur_mark);
       cur_mark = NULL;
    }

    // check for lighting that goes all the way to the end...
    if (cur_light)
    {
       SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Lighting from " << cur_light->start_idx << " with type " << cur_light->type << " ends at the end of the contour: " << points.GetSize() );

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

int LinearFeature::Finish( bool closed, unsigned int idx )
{
    tgPolygon   poly;
    SGGeod      prev_inner, prev_outer;
    SGGeod      cur_inner,  cur_outer;
    double      heading;
    double      dist;
    double      az2;
    double      last_end_v;
    double      width = 0;
    std::string material;
    double      cur_light_dist = 0.0f;
    double      light_delta = 0;
    bool        markStarted;

    // create the inner and outer boundaries to generate polys
    // this generates 2 point lists for the contours, and remembers
    // the start stop points for markings and lights
    ConvertContour( &contour, closed );

    // now generate the supoerpoly and texparams lists for markings
    for (unsigned int i=0; i<marks.size(); i++)
    {
        markStarted = false;
        last_end_v   = 0.0f;

        // which material for this mark?
        switch( marks[i]->type )
        {
            case LF_NONE:
                break;

            case LF_SOLID_YELLOW:
                material = "lf_sng_solid_yellow";
                width = 0.25f;
                break;

            case LF_BROKEN_YELLOW:
                material = "lf_sng_broken_yellow";
                width = 0.25f;
                break;

            case LF_SOLID_DBL_YELLOW:
                material = "lf_dbl_solid_yellow";
                width = 0.5f;
                break;

            case LF_RUNWAY_HOLD:
                material = "lf_runway_hold";
                width = 1.0f;
                break;

            case LF_OTHER_HOLD:
                material = "lf_other_hold";
                width = 0.5f;
                break;

            case LF_ILS_HOLD:
                material = "lf_ils_hold";
                width = 1.0f;
                break;

            case LF_SAFETYZONE_CENTERLINE:
                material = "lf_safetyzone_centerline";
                width = 0.75f;
                break;

            case LF_SINGLE_LANE_QUEUE:
                material = "lf_sng_lane_queue";
                width = 0.25f;
                break;

            case LF_DOUBLE_LANE_QUEUE:
                material = "lf_dbl_lane_queue";
                width = 0.5f;
                break;

            case LF_B_SOLID_YELLOW:
                material = "lf_dbl_solid_yellow";
                width = 0.25f;
                break;

            case LF_B_BROKEN_YELLOW:
                material = "lf_sng_broken_yellow_border";
                width = 0.25f;
                break;

            case LF_B_SOLID_DBL_YELLOW:
                material = "lf_dbl_solid_yellow_border";
                width = 0.5f;
                break;

            case LF_B_RUNWAY_HOLD:
                material = "lf_runway_hold_border";
                width = 1.0f;
                break;

            case LF_B_OTHER_HOLD:
                material = "lf_other_hold_border";
                width = 0.5f;
                break;

            case LF_B_ILS_HOLD:
                material = "lf_ils_hold_border";
                width = 1.0f;
                break;

            case LF_B_SAFETYZONE_CENTERLINE:
                material = "lf_safetyzone_centerline_border";
                width = 0.75f;
                break;

            case LF_B_SINGLE_LANE_QUEUE:
                material = "lf_sng_lane_queue_border";
                width = 0.25f;
                break;

            case LF_B_DOUBLE_LANE_QUEUE:
                material = "lf_dbl_lane_queue_border";
                width = 0.5f;
                break;

            case LF_SOLID_WHITE:
                material = "lf_sng_solid_white";
                width = 0.25f;
                break;

            case LF_CHECKERBOARD_WHITE:
                material = "lf_checkerboard_white";
                width = 0.5f;
                break;

            case LF_BROKEN_WHITE:
                material = "lf_broken_white";
                width = 0.25f;
                break;

            default:
                SG_LOG(SG_GENERAL, SG_ALERT, "LinearFeature::Finish: unknown marking " << marks[i]->type );
                exit(1);
        }

        for (unsigned int j = marks[i]->start_idx; j <= marks[i]->end_idx; j++)
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::Finish: calculating offsets for mark " << i << " whose start idx is " << marks[i]->start_idx << " and end idx is " << marks[i]->end_idx << " cur idx is " << j );
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

            if ( markStarted )
            {
                SGGeod prev_mp = midpoint( prev_outer, prev_inner );
                SGGeod cur_mp  = midpoint( cur_outer,  cur_inner  );
                SGGeodesy::inverse( prev_mp, cur_mp, heading, az2, dist );

                poly.Erase();
                poly.AddNode( 0, prev_inner );
                poly.AddNode( 0, prev_outer );
                poly.AddNode( 0, cur_outer  );
                poly.AddNode( 0, cur_inner  );
                poly = tgPolygon::Snap( poly, gSnap );

                poly.SetMaterial( material );
                poly.SetTexParams( prev_inner, width, 1.0f, heading );
                poly.SetTexLimits( 0, last_end_v, 1, 1 );
                poly.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, 0.0, 1.0, 0.0 );
                marking_polys.push_back(poly);

                last_end_v = (double)1.0f - (fmod( (double)(dist - last_end_v), (double)1.0f ));
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
            SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::Finish: calculating offsets for light " << i << " whose start idx is " << lights[i]->start_idx << " and end idx is " << lights[i]->end_idx << " cur idx is " << j );
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
            SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature::Finish: Adding light contour with " << cur_light_contour.ContourSize() << " lights" );
            cur_light_contour.SetFlag("");
            lighting_polys.push_back(cur_light_contour);
        }

        // create the superpoly for the alternating light color
        if (alt_light_contour.ContourSize())
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature::Finish: Adding light contour with " << cur_light_contour.ContourSize() << " lights" );
            alt_light_contour.SetFlag("");
            lighting_polys.push_back(cur_light_contour);
        }
        else
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature::Finish: No points for linear feature " << description << " light index " << i );
        }
    }

    return 1;
}

int LinearFeature::BuildBtg(tgpolygon_list& line_polys, tglightcontour_list& lights, tgAccumulator& accum, bool make_shapefiles )
{
    tgPolygon poly;
    SGGeod    min, max, minp, maxp;

    SG_LOG(SG_GENERAL, SG_DEBUG, "\nLinearFeature::BuildBtg: " << description);
    for ( unsigned int i = 0; i < marking_polys.size(); i++)
    {
        // Clipping and triangulation need to copy texparams, and material info...
        marking_polys[i] = accum.Diff( marking_polys[i] );
        line_polys.push_back( marking_polys[i] );

        accum.Add( marking_polys[i] );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::BuildBtg: add " << lighting_polys.size() << " light defs");
    for ( unsigned i = 0; i < lighting_polys.size(); i++)
    {
        lights.push_back( lighting_polys[i] );
    }

    return 1;
}
