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

    SG_LOG(SG_GENERAL, SG_DEBUG, "Creating a contour with " << src->size() << " nodes");

    // clear anything in the point list
    points.empty();

    // iterate through each bezier node in the contour
    for (i=0; i <= src->size()-1; i++)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "\nHandling Node " << i << "\n\n");

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
                // amrking has ended, or changed
                cur_mark->end_idx = points.size();
                marks.push_back(cur_mark);
                cur_mark = NULL;
            }
        }
        
        // should we start a new mark?
        if (cur_mark == NULL)
        {
            if (curNode->GetMarking())
            {
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
}


Point3D LinearFeature::OffsetPointMiddle( Point3D *prev, Point3D *cur, Point3D *next, double offset_by )
{
    double offset_dir;
    double next_dir;
    double az1, az2;
    double dist;
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

    // find the offset angle
    geo_inverse_wgs_84( 0.0f, 0.0f, avg.y(), avg.x(), &offset_dir, &az2, &dist);

    // find the direction to the next point
    geo_inverse_wgs_84( cur->y(), cur->x(), next->y(), next->x(), &next_dir, &az2, &dist);

    // calculate correct distance for the offset point
    dist = (offset_by)/sin(SGMiscd::deg2rad(offset_dir-next_dir));

    SG_LOG(SG_GENERAL, SG_DEBUG, "heading is " << offset_dir << " distance is " << dist );

    // calculate the point from cur
    geo_direct_wgs_84( cur->y(), cur->x(), offset_dir, dist, &pt_y, &pt_x, &az2 );

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
    geo_inverse_wgs_84( cur->x(), cur->y(), next->x(), next->y(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "heading is " << offset_dir << " distance is " << offset_by );

    // calculate the point from cur
    geo_direct_wgs_84( cur->y(), cur->x(), offset_dir, offset_by, &pt_y, &pt_x, &az2 );

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
    geo_inverse_wgs_84( prev->x(), prev->y(), cur->x(), cur->y(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "heading is " << offset_dir << " distance is " << offset_by );

    // calculate the point from cur
    geo_direct_wgs_84( cur->y(), cur->x(), offset_dir, offset_by, &pt_y, &pt_x, &az2 );

    return Point3D(pt_x, pt_y, 0.0f);
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
    int         i, j;
    string      material;


    // create the inner and outer boundaries to generate polys
    // this generates 2 point lists for the contours, and remembers 
    // the start stop points for markings
    ConvertContour( &contour );

    // now generate the supoerpoly and texparams list
    for (i=0; i<marks.size(); i++)
    {
        prev_inner = Point3D(0.0f, 0.0f, 0.0f);
        prev_outer = Point3D(0.0f, 0.0f, 0.0f);

        // which material for this mark?
        switch( marks[i]->type )
        {
            case LF_NONE:
            case LF_SOLID_YELLOW:
            case LF_BROKEN_YELLOW:
            case LF_SOLID_DBL_YELLOW:
            case LF_RUNWAY_HOLD:
            case LF_OTHER_HOLD:
            case LF_ILS_HOLD:
            case LF_SAFETYZONE_CENTERLINE:
            case LF_SINGLE_LANE_QUEUE:
            case LF_DOUBLE_LANE_QUEUE:

            case LF_B_SOLID_YELLOW:
            case LF_B_BROKEN_YELLOW:
            case LF_B_SOLID_DBL_YELLOW:
            case LF_B_RUNWAY_HOLD:
            case LF_B_OTHER_HOLD:
            case LF_B_ILS_HOLD:
            case LF_B_SAFETYZONE_CENTERLINE:
            case LF_B_SINGLE_LANE_QUEUE:
            case LF_B_DOUBLE_LANE_QUEUE:

            case LF_SOLID_WHITE:
            case LF_CHECKERBOARD_WHITE:
            case LF_BROKEN_WHITE:

            case LF_BIDIR_GREEN:
            case LF_OMNIDIR_BLUE:
            case LF_UNIDIR_CLOSE_AMBER:
            case LF_UNIDIR_CLOSE_AMBER_PULSE:
            case LF_BIDIR_GREEN_AMBER:
            case LF_OMNIDIR_RED:
                material = "gloff_lf_b_solid_yellow";
                break;

            default:
                SG_LOG(SG_GENERAL, SG_DEBUG, "ClosedPoly::BuildBtg: unknown material " << marks[i]->type );
                exit(1);
        }

        for (j = marks[i]->start_idx; j < marks[i]->end_idx; j++)
        {
            // for each point on the PointsList, generate a quad from
            // start to next, offset by 2 distnaces from the edge

            if (j == 0)
            {
                // first point on the contour - offset heading is 90deg 
                cur_outer = OffsetPointFirst( &points[j], &points[j+1], 0.4 );
                cur_inner = OffsetPointFirst( &points[j], &points[j+1], 0.5 );
            }
            else if (j == points.size()-1)
            {
                // last point on the contour - offset heading is 90deg 
                cur_outer = OffsetPointFirst( &points[j-1], &points[j], 0.4 );
                cur_inner = OffsetPointFirst( &points[j-1], &points[j], 0.5 );
            }
            else
            {
                cur_outer = OffsetPointMiddle( &points[j-1], &points[j], &points[j+1], 0.4 );
                cur_inner = OffsetPointMiddle( &points[j-1], &points[j], &points[j+1], 0.5 );
            }

            if ( (prev_inner.x() != 0.0f) && (prev_inner.y() != 0.0f) )
            {
                geo_inverse_wgs_84( prev_outer.y(), prev_outer.x(), cur_outer.y(), cur_outer.x(), &heading, &az2, &dist);

                poly.erase();
                poly.add_node( 0, prev_outer );
                poly.add_node( 0, prev_inner );
                poly.add_node( 0, cur_inner );
                poly.add_node( 0, cur_outer );

                sp.erase();
                sp.set_poly( poly );
                sp.set_material( material );
                feature_polys.push_back(sp);

                tp = TGTexParams( prev_inner, 0.1, 1.0, heading );
                feature_tps.push_back(tp);
            }

            prev_outer = cur_outer;
            prev_inner = cur_inner;
        }
    }
}

int LinearFeature::BuildBtg(float alt_m, superpoly_list* line_polys, texparams_list* line_tps, TGPolygon* line_accum )
{
    string material;
    int j, k;

#if 0

    // verify the poly has been generated
    if ( pre_tess.contours() )    
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "LinearFeature::BuildBtg: original poly has " << pre_tess.contours() << " contours");
    
        // do this before clipping and generating the base
    	pre_tess = remove_dups( pre_tess );
        pre_tess = reduce_degeneracy( pre_tess );

        for (int c=0; c<pre_tess.contours(); c++)
        {
            for (int pt=0; pt<pre_tess.contour_size(c); pt++)
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: contour " << c << " pt " << pt << ": (" << pre_tess.get_pt(c, pt).x() << "," << pre_tess.get_pt(c, pt).y() << ")" );
            }
        }


        TGSuperPoly sp;
        TGTexParams tp;

        TGPolygon clipped = tgPolygonDiff( pre_tess, *line_accum );
        SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: clipped poly has " << clipped.contours() << " contours");

        TGPolygon split   = tgPolygonSplitLongEdges( clipped, 400.0 );
        SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: split poly has " << split.contours() << " contours");

        sp.erase();
        sp.set_poly( split );
        sp.set_material( material );
        sp.set_flag("taxi");

        line_polys->push_back( sp );
        SG_LOG(SG_GENERAL, SG_DEBUG, "clipped = " << clipped.contours());
        *line_accum = tgPolygonUnion( pre_tess, *line_accum );
        tp = TGTexParams( pre_tess.get_pt(0,0), 0.2 /* TODO poly width */, 1.0 /* TODO poly length */, texture_heading );
        texparams->push_back( tp );
    }
#endif

    return 1;
}
