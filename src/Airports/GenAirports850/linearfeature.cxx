#include <stdlib.h>

#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/SGVec3.hxx>
#include <simgear/math/SGMisc.hxx>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>

#include <Geometry/poly_support.hxx>

#include "beznode.hxx"
#include "linearfeature.hxx"
#include "math.h"

void LinearFeature::ConvertContour( BezContour* src  )
{
    BezNode*    prevNode;
    BezNode*    curNode;
    BezNode*    nextNode;
        
    Point3D prevLoc;
    Point3D curLoc;
    Point3D nextLoc;
    Point3D cp1;
    Point3D cp2;         

    int curve_type = CURVE_LINEAR;
    Marking* cur_mark = NULL;
    int i;

    SG_LOG(SG_GENERAL, SG_DEBUG, " LinearFeature::ConvertContour - Creating a contour with " << src->size() << " nodes");

    // clear anything in the point list
    points.empty();

    // iterate through each bezier node in the contour
    for (i=0; i <= src->size()-1; i++)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, " LinearFeature::ConvertContour: Handling Node " << i << "\n\n");

        if (i == 0)
        {
            // set prev node to last in the contour, as all contours must be closed
            prevNode = src->at( src->size()-1 );
        }
        else
        {
            // otherwise, it's just the previous index
            prevNode = src->at( i-1 );
        }

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
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking has changed from " << cur_mark->type << " to " << curNode->GetMarking() << " save mark from " << cur_mark->start_idx << " to " << points.size() );

                // marking has ended, or changed
                cur_mark->end_idx = points.size();
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
                SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Start Marking from " << points.size() << " with type " << curNode->GetMarking() );

                // we aren't watching a mark, and this node has one
                cur_mark = new Marking;
                cur_mark->type = curNode->GetMarking();
                cur_mark->start_idx = points.size();
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////


        // determine the type of curve from prev (just to get correct prev location)
        // once we start drawing the curve from cur to next, we can just remember the prev loc
        if (prevNode->HasNextCp())
        {
            // curve from prev is cubic or quadratic 
            if(curNode->HasPrevCp())
            {
                // curve from prev is cubic : calculate the last location on the curve
                prevLoc = CalculateCubicLocation( prevNode->GetLoc(), prevNode->GetNextCp(), curNode->GetPrevCp(), curNode->GetLoc(), (1.0f/BEZIER_DETAIL) * (BEZIER_DETAIL-1) );
            }
            else
            {
                // curve from prev is quadratic : use prev node next cp
                prevLoc = CalculateQuadraticLocation( prevNode->GetLoc(), prevNode->GetNextCp(), curNode->GetLoc(), (1.0f/BEZIER_DETAIL) * (BEZIER_DETAIL-1) );
            }
        }
        else 
        {
            // curve from prev is quadratic or linear
            if( curNode->HasPrevCp() )
            {
                // curve from prev is quadratic : calculate the last location on the curve
                prevLoc = CalculateQuadraticLocation( prevNode->GetLoc(), curNode->GetPrevCp(), curNode->GetLoc(), (1.0f/BEZIER_DETAIL) * (BEZIER_DETAIL-1) );
            }
            else
            {
                // curve from prev is linear : just use prev node location
                prevLoc = prevNode->GetLoc();
            }
        }                    

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
            }
            else
            {
                // curve is quadratic using current nodes cp as the cp
                curve_type = CURVE_QUADRATIC;
                cp1 = curNode->GetNextCp();
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
            }
            else
            {
                // curve is linear
                curve_type = CURVE_LINEAR;
            }
        }

        // initialize current location
        curLoc = curNode->GetLoc();
        if (curve_type != CURVE_LINEAR)
        {
            for (int p=0; p<BEZIER_DETAIL; p++)
            {
                // calculate next location
                if (curve_type == CURVE_QUADRATIC)
                {
                    nextLoc = CalculateQuadraticLocation( curNode->GetLoc(), cp1, nextNode->GetLoc(), (1.0f/BEZIER_DETAIL) * (p+1) );                    
                }
                else
                {
                    nextLoc = CalculateCubicLocation( curNode->GetLoc(), cp1, cp2, nextNode->GetLoc(), (1.0f/BEZIER_DETAIL) * (p+1) );                    
                }

                // add the feature vertex
                points.push_back( curLoc );

                if (p==0)
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "adding Curve Anchor node (type " << curve_type << ") at (" << curLoc.x() << "," << curLoc.y() << ")");
                }
                else
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "   add bezier node (type  " << curve_type << ") at (" << curLoc.x() << "," << curLoc.y() << ")");
                }

                // now set set prev and cur locations for the next iteration
                prevLoc = curLoc;
                curLoc = nextLoc;
            }
        }
        else
        {
            nextLoc = nextNode->GetLoc();

            // just add the one vertex - linear
            points.push_back( curLoc );

            SG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear Anchor node at (" << curLoc.x() << "," << curLoc.y() << ")");
        }
    }

    // check for marking that goes all the way to the end...
   if (cur_mark)
   {
       SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::ConvertContour Marking from " << cur_mark->start_idx << " with type " << cur_mark->type << " ends at the end of the contour: " << points.size() );

       cur_mark->end_idx = points.size()-1;
       marks.push_back(cur_mark);
       cur_mark = NULL;                    
    }
}

Point3D LinearFeature::OffsetPointMiddle( Point3D *prev, Point3D *cur, Point3D *next, double offset_by )
{
    double offset_dir;
    double next_dir;
    double az1, az2;
    double dist;
    double theta;
    double pt_x, pt_y;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find average angle for contour: prev (" << *prev << "), "
                                                                  "cur (" << *cur  << "), "
                                                                 "next (" << *next << ")" );

 
   // first, find if the line turns left or right ar src
    // for this, take the cross product of the vectors from prev to src, and src to next.
    // if the cross product is negetive, we've turned to the left
    // if the cross product is positive, we've turned to the right
    // if the cross product is 0, then we need to use the direction passed in
    SGVec3d dir1 = prev->toSGVec3d() - cur->toSGVec3d();
    dir1 = normalize(dir1);

    SGVec3d dir2 = next->toSGVec3d() - cur->toSGVec3d();
    dir2 = normalize(dir2);

    // Now find the average
    SGVec3d avg = dir1 + dir2;
    avg = normalize(avg);

    // check the turn direction
    SGVec3d cp = cross( dir1, dir2 );
    SG_LOG(SG_GENERAL, SG_ALERT, "\tcross product of dir1: " << dir1 << " and dir2: " << dir2 << " is " << cp );

    // calculate the angle between cur->prev and cur->next
    theta = SGMiscd::rad2deg(CalculateTheta(*prev, *cur, *next));

    if ( abs(theta - 180.0) < 0.1 )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "\nLinearFeature: (theta close to 180) " << description << ": theta is " << theta );

        // find the direction to the next point
        geo_inverse_wgs_84( cur->y(), cur->x(), next->y(), next->x(), &next_dir, &az2, &dist);

        offset_dir = next_dir - 90.0;
        while (offset_dir < 0.0)
        {
            offset_dir += 360.0;
        }

        // straight line blows up math - dist should be exactly as given
        dist = offset_by;        
    }
    else if ( abs(theta) < 0.1 )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "\nLinearFeature: (theta close to 0) " << description << ": theta is " << theta );

        // find the direction to the next point
        geo_inverse_wgs_84( cur->y(), cur->x(), next->y(), next->x(), &next_dir, &az2, &dist);

        offset_dir = next_dir - 90;
        while (offset_dir < 0.0)
        {
            offset_dir += 360.0;
        }

        // straight line blows up math - dist should be exactly as given
        dist = offset_by;        
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "\nLinearFeature: (theta NOT close to 180) " << description << ": theta is " << theta );

        // find the offset angle
        geo_inverse_wgs_84( avg.y(), avg.x(), 0.0f, 0.0f, &offset_dir, &az2, &dist);

        // if we turned right, reverse the heading 
        if (cp.z() < 0.0f)
        {
            offset_dir += 180.0;
        }
        while (offset_dir >= 360.0)
        {
            offset_dir -= 360.0;
        }

        // find the direction to the next point
        geo_inverse_wgs_84( cur->y(), cur->x(), next->y(), next->x(), &next_dir, &az2, &dist);

        // calculate correct distance for the offset point
        dist = (offset_by)/sin(SGMiscd::deg2rad(next_dir-offset_dir));
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "\theading is " << offset_dir << " distance is " << dist );

    // calculate the point from cur
    geo_direct_wgs_84( cur->y(), cur->x(), offset_dir, dist, &pt_y, &pt_x, &az2 );

    SG_LOG(SG_GENERAL, SG_ALERT, "\tpoint is (" << pt_x << "," << pt_y << ")" );

    return Point3D(pt_x, pt_y, 0.0f);
}

Point3D LinearFeature::OffsetPointFirst( Point3D *cur, Point3D *next, double offset_by )
{
    double offset_dir;
    double az1, az2;
    double dist;
    double pt_x, pt_y;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at Start : cur (" << *cur  << "), "
                                                            "next (" << *next << ")" );

    // find the offset angle
    geo_inverse_wgs_84( cur->y(), cur->x(), next->y(), next->x(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << offset_dir << " distance is " << offset_by );

    // calculate the point from cur
    geo_direct_wgs_84( cur->y(), cur->x(), offset_dir, offset_by, &pt_y, &pt_x, &az2 );

    SG_LOG(SG_GENERAL, SG_DEBUG, "\tpoint is (" << pt_x << "," << pt_y << ")" );

    return Point3D(pt_x, pt_y, 0.0f);
}

// TODO: Should Be AddEndMarkingVerticies - and handle offset (used in LinearFeature::Finish only)
Point3D LinearFeature::OffsetPointLast( Point3D *prev, Point3D *cur, double offset_by )
{
    double offset_dir;
    double az1, az2;
    double dist;
    double pt_x, pt_y;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at End   : prev (" << *prev  << "), "
                                                              "cur (" << *cur << ")" );

    // find the offset angle
    geo_inverse_wgs_84( prev->y(), prev->x(), cur->y(), cur->x(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << offset_dir << " distance is " << offset_by );

    // calculate the point from cur
    geo_direct_wgs_84( cur->y(), cur->x(), offset_dir, offset_by, &pt_y, &pt_x, &az2 );

    SG_LOG(SG_GENERAL, SG_DEBUG, "\tpoint is (" << pt_x << "," << pt_y << ")" );

    return Point3D(pt_x, pt_y, 0.0f);
}

Point3D midpoint( Point3D p0, Point3D p1 )
{
    return Point3D( (p0.x() + p1.x()) / 2, (p0.y() + p1.y()) / 2, (p0.z() + p1.z()) / 2 );
}

int LinearFeature::Finish()
{
    TGPolygon   poly;
    TGSuperPoly sp;
    TGTexParams tp;
    Point3D     prev_inner, prev_outer;
    Point3D     cur_inner,  cur_outer;
    double      heading;
    double      dist;
    double      az2;
    double      last_end_v;
    double      width;
    int         i, j;
    string      material;
    int         mat_idx = 0;


    // create the inner and outer boundaries to generate polys
    // this generates 2 point lists for the contours, and remembers 
    // the start stop points for markings and lights
    ConvertContour( &contour );

    // now generate the supoerpoly and texparams lists for markings
    for (i=0; i<marks.size(); i++)
    {
        prev_inner = Point3D(0.0f, 0.0f, 0.0f);
        prev_outer = Point3D(0.0f, 0.0f, 0.0f);

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

        last_end_v   = 0.0f;
        for (j = marks[i]->start_idx; j <= marks[i]->end_idx; j++)
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::Finish: calculating offsets for mark " << i << " whose start idx is " << marks[i]->start_idx << " and end idx is " << marks[i]->end_idx << " cur idx is " << j );
            // for each point on the PointsList, generate a quad from
            // start to next, offset by 2 distnaces from the edge

            if (j == marks[i]->start_idx)
            {
                // first point on the mark - offset heading is 90deg 
                cur_outer = OffsetPointFirst( &points[j], &points[j+1], offset-width/2.0f );
                cur_inner = OffsetPointFirst( &points[j], &points[j+1], offset+width/2.0f );
            }
            else if (j == marks[i]->end_idx)
            {
                // last point on the mark - offset heading is 90deg 
                cur_outer = OffsetPointLast( &points[j-1], &points[j], offset-width/2.0f );
                cur_inner = OffsetPointLast( &points[j-1], &points[j], offset+width/2.0f );
            }
            else
            {
                cur_outer = OffsetPointMiddle( &points[j-1], &points[j], &points[j+1], offset-width/2.0f );
                cur_inner = OffsetPointMiddle( &points[j-1], &points[j], &points[j+1], offset+width/2.0f );
            }

            if ( (prev_inner.x() != 0.0f) && (prev_inner.y() != 0.0f) )
            {
                Point3D prev_mp = midpoint( prev_outer, prev_inner );
                Point3D cur_mp  = midpoint( cur_outer,  cur_inner  );
                geo_inverse_wgs_84( prev_mp.y(), prev_mp.x(), cur_mp.y(), cur_mp.x(), &heading, &az2, &dist);

                poly.erase();
                poly.add_node( 0, prev_inner );
                poly.add_node( 0, prev_outer );
                poly.add_node( 0, cur_outer );
                poly.add_node( 0, cur_inner );

                sp.erase();
                sp.set_poly( poly );
                sp.set_material( material );
                sp.set_flag("lf");
                marking_polys.push_back(sp);

                tp = TGTexParams( prev_inner, width, 1.0f, heading );
                tp.set_minv(last_end_v);
                marking_tps.push_back(tp);

                last_end_v = 1.0f - (fmod( (dist - last_end_v), 1.0f ));
            }

            prev_outer = cur_outer;
            prev_inner = cur_inner;
        }
    }

    // now generate the supoerpoly list for lights
    for (i=0; i<lights.size(); i++)
    {
        // which material for this light
        switch( lights[i]->type )
        {
            case LF_BIDIR_GREEN:
                break;

            case LF_OMNIDIR_BLUE:
                break;

            case LF_UNIDIR_CLOSE_AMBER:
                break;

            case LF_UNIDIR_CLOSE_AMBER_PULSE:
                break;

            case LF_BIDIR_GREEN_AMBER:
                break;

            case LF_OMNIDIR_RED:
                break;
        }
    }
}

int LinearFeature::BuildBtg(float alt_m, superpoly_list* line_polys, texparams_list* line_tps, TGPolygon* line_accum, superpoly_list* lights )
{
    TGPolygon poly; 
    TGPolygon clipped;
    TGPolygon split;
    int i;

    for (i=0; i<marking_polys.size(); i++)
    {
        poly = marking_polys[i].get_poly();
        clipped = tgPolygonDiff( poly, *line_accum );

        SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: clipped poly has " << clipped.contours() << " contours");

        TGPolygon split   = tgPolygonSplitLongEdges( clipped, 400.0 );
        SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: split poly has " << split.contours() << " contours");

        marking_polys[i].set_poly( split );
        line_polys->push_back( marking_polys[i] );

        *line_accum = tgPolygonUnion( poly, *line_accum );
        line_tps->push_back( marking_tps[i] );
    }

    return 1;
}
