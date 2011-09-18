#include "beznode.hxx"
#include "linearfeature.hxx"
#include "math.h"

#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/SGVec3.hxx>
#include <simgear/math/SGMisc.hxx>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>

int LinearFeature::Finish( osg::Group* airport )
{
    BezNode*    node;
    int         i, j;
    int         prev_dir = 0;
    double      direction;
    double      dist1, dist2;

    printf("Finish a %s Linear Feature with %d verticies\n", closed ? "closed" : "open", contour.size());
    
    BezNode* prevNode;
    BezNode* curNode;
    BezNode* nextNode;

    Point3D prevLoc;
    Point3D curLoc;
    Point3D nextLoc;

    Point3D  st_cur1;
    Point3D  st_cur2;
    double st_curx, st_cury, az2;

    // create a DrawElement for the stripe triangle strip
    int stripe_idx = 0;
    int cur_marking = 0;

    osg::Vec3dArray* v_stripe = new osg::Vec3dArray;

    for (i=0; i<contour.size(); i++)
    {
        printf("\nHandling Node %d\n\n", i );

        if (i == 0)
        {
            // if closed, set prev node to last in the contour
            if (closed)
            {
                prevNode = contour.at( contour.size()-1 );
            }
            else
            {
                prevNode = NULL;
            }
        }
        else
        {
            prevNode = contour.at( i-1 );
        }

        curNode = contour.at(i);

        if (i<contour.size()-1)
        {
            nextNode = contour.at(i+1);
        }
        else
        {
            // if closed, set next node to the first in the contour
            if (closed)
            {
                nextNode = contour.at(0);
            }
            else
            {
                nextNode = NULL;
            }
        }

        // determine the type of curve from prev (just to get correct prev location)
        // once we start drawing the curve from cur to next, we can just remember the prev loc
        if (prevNode)
        {        
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
        }
        else
        {
            prevLoc = Point3D(0.0f, 0.0f, 0.0f);
        }

        int curve_type = CURVE_LINEAR;
        Point3D cp1;
        Point3D cp2;            

        if ( nextNode)
        {
            // now determine how we will iterate through cur node to next node 
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
        }
        else
        {
            curve_type = CURVE_NONE;
        }

        // initialize current location
        curLoc = curNode->GetLoc();
        switch( curve_type )
        {
            case CURVE_QUADRATIC:
            case CURVE_CUBIC:
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

                    // set the verticies
                    // convert from lat/lon to geodisc
                    // (maybe later) - check some simgear objects...
                    // printf("Added vertex at %lf,%lf\n", 
                    // add the pavement vertex
                    if (p==0)
                    {
                        printf("adding Curve Anchor node (type %d) at (%lf,%lf)\n", curve_type, curLoc.x(), curLoc.y());
                    }
                    else
                    {
                        printf("   add bezier node (type %d) at (%lf,%lf)\n", curve_type, curLoc.x(), curLoc.y());
                    }
    
                    // find the average direction at this vertex...
                    direction = CalcMarkingVerticies( &prevLoc, &curLoc, &nextLoc, -1, &prev_dir, &dist1, &dist2 );

                    printf("got dir for bezier : prev_dir %d, dist1 %lf, dist2 %lf\n", prev_dir, dist1, dist2);

                    // distance can't be constant - it's the hypotenous of a right triangler with a constant offset from the outer rim.              
                    geo_direct_wgs_84( curLoc.x(), curLoc.y(), direction, dist1, &st_curx, &st_cury, &az2 );
                    st_cur1 = Point3D( st_curx, st_cury, 0.0f );
                    v_stripe->push_back( SGPoint3DtoOSGVec3d(st_cur1) );

                    geo_direct_wgs_84( curLoc.x(), curLoc.y(), direction, dist2, &st_curx, &st_cury, &az2 );
                    st_cur2 = Point3D( st_curx, st_cury, 0.0f );
                    v_stripe->push_back( SGPoint3DtoOSGVec3d(st_cur2) );
    
                    printf("node at (%lf,%lf) has stripe verticies (%lf,%lf) and (%lf,%lf)\n", 
                        curLoc.x(), curLoc.y(), 
                        st_cur1.x(), st_cur1.y(),
                        st_cur2.x(), st_cur2.y() 
                    );

                    // now set set prev and cur locations for the next iteration
                    prevLoc = curLoc;
                    curLoc = nextLoc;
                }
                break;

            case CURVE_LINEAR:
                nextLoc = nextNode->GetLoc();

                printf("adding Linear Anchor node at (%lf,%lf)\n", curLoc.x(), curLoc.y());

                // find the average direction at this vertex...
                direction = CalcMarkingVerticies( &prevLoc, &curLoc, &nextLoc, -1, &prev_dir, &dist1, &dist2 );

                printf("got dir for linear : prev_dir %d, dist1 %lf, dist2 %lf\n", prev_dir, dist1, dist2);

                // distance can't be constant - it's the hypotenous of a right triangler with a constant offset from the outer rim.              
                printf("calc direct: curLoc is (%lf,%lf), direction is %lf, dist is %lf\n", curLoc.x(), curLoc.y(), direction, dist1);

                geo_direct_wgs_84( curLoc.x(), curLoc.y(), direction, dist1, &st_curx, &st_cury, &az2 );
                st_cur1 = Point3D( st_curx, st_cury, 0.0f );
                
                v_stripe->push_back( SGPoint3DtoOSGVec3d(st_cur1) );

                geo_direct_wgs_84( curLoc.x(), curLoc.y(), direction, dist2, &st_curx, &st_cury, &az2 );
                st_cur2 = Point3D( st_curx, st_cury, 0.0f );
                v_stripe->push_back( SGPoint3DtoOSGVec3d(st_cur2) );

                printf("node at (%lf,%lf) has stripe verticies (%lf,%lf) and (%lf,%lf)\n", 
                    curLoc.x(), curLoc.y(), 
                    st_cur1.x(), st_cur1.y(),
                    st_cur2.x(), st_cur2.y() 
                );
                break;

            case CURVE_NONE:
                nextLoc = Point3D(0.0f, 0.0f, 0.0f);

                // we need to add the last verticies based on cur and prev position.
                // find the average direction at this vertex...
                direction = CalcMarkingVerticies( &prevLoc, &curLoc, &nextLoc, -1, &prev_dir, &dist1, &dist2 );

                printf("got dir for term points : prev_dir %lf, dist1 %lf, dist2 %lf\n", prev_dir, dist1, dist2);

                // distance can't be constant - it's the hypotenous of a right triangler with a constant offset from the outer rim.              
                geo_direct_wgs_84( curLoc.x(), curLoc.y(), direction, dist1, &st_curx, &st_cury, &az2 );
                st_cur1 = Point3D( st_curx, st_cury, 0.0f );
                v_stripe->push_back( SGPoint3DtoOSGVec3d(st_cur1) );

                geo_direct_wgs_84( curLoc.x(), curLoc.y(), direction, dist2, &st_curx, &st_cury, &az2 );
                st_cur2 = Point3D( st_curx, st_cury, 0.0f );
                v_stripe->push_back( SGPoint3DtoOSGVec3d(st_cur2) );

                printf("node at (%lf,%lf) has stripe verticies (%lf,%lf) and (%lf,%lf)\n", 
                    curLoc.x(), curLoc.y(), 
                    st_cur1.x(), st_cur1.y(),
                    st_cur2.x(), st_cur2.y() 
                );
                break;
        }
    }

    printf("End feature %d \n", v_stripe->size() );
    airport->addChild(FinishMarking( v_stripe ) );
}

int LinearFeature::BuildBtg( float alt_m, superpoly_list* rwy_polys, texparams_list* texparams, TGPolygon* accum, TGPolygon* apt_base, TGPolygon* apt_clearing )
{
    string material;

    return 1;
}


// this should be in the class
// TODO: Should Be AddMidMarkingVerticies - and handle offset (used in LinearFeature::Finish only)
double CalcMiddleMarkingVerticies( Point3D *prev, Point3D *cur, Point3D *next, int wind, int *prev_dir, double *dist1, double *dist2 )
{
    int    turn_direction;
    bool   reverse_dir = false;
    double offset_dir;
    double next_dir;
    double az1, az2;
    double dist;

    printf("Find averate angle for mark: prev (%lf,%lf), cur(%lf,%lf), next(%lf,%lf)\n", prev->x(), prev->y(), cur->x(), cur->y(), next->x(), next->y());

    // first, find if the line turns left or right ar src
    // for this, take the cross product of the vectors from prev to src, and src to next.
    // if the cross product is negetive, we've turned to the left
    // if the cross product is positive, we've turned to the right
    // if the cross product is 0, then we need to use the direction passed in
    SGVec3d dir1 = cur->toSGVec3d() - prev->toSGVec3d();
    dir1 = normalize(dir1);

    printf("normalized dir1 is (%lf,%lf)\n", dir1.x(), dir1.y());

    SGVec3d dir2 = next->toSGVec3d() - cur->toSGVec3d();
    dir2 = normalize(dir2);

    printf("normalized dir2 is (%lf,%lf)\n", dir2.x(), dir2.y());

    SGVec3d cp = cross(dir1,dir2);
    if (cp.z() < 0.0)
    {
        turn_direction = -1;
    }
    else if (cp.z() > 0.0)
    {
        turn_direction = 1;
    }
    else
    {
        turn_direction = 0;
    }

    printf("turn direction is %d\n", turn_direction);

    // Now find the average
    SGVec3d avg = -dir1 + dir2;

    printf("avg is (%lf,%lf)\n", avg.x(), avg.y());

    // now determine which way the direction needs to point
    if ((wind == -1) && (turn_direction == 1))
    {
        reverse_dir = true;
    }
    if ((wind == 1) && (turn_direction == -1))
    {
        reverse_dir = true;
    }

    printf("reverse dir is %d\n", reverse_dir);

    // find the offset angle
    geo_inverse_wgs_84( 0.0f, 0.0f, avg.x(), avg.y(), &offset_dir, &az2, &dist);
    if (reverse_dir)
    {
        offset_dir += 180;
        while (offset_dir >= 360.0)
        {
            offset_dir -= 360.0;
        }
    }
    printf("offset direction is %lf\n", offset_dir);

    // find the direction to the next point
    geo_inverse_wgs_84( cur->x(), cur->y(), next->x(), next->y(), &next_dir, &az2, &dist);
    printf("next direction is %lf\n", next_dir);

    // calculate correct distance for the offset point
    *dist1 = (-LINE_WIDTH)/sin(SGMiscd::deg2rad(next_dir-offset_dir));
    *dist2 = (LINE_WIDTH)/sin(SGMiscd::deg2rad(next_dir-offset_dir));

    return offset_dir;
}

// TODO: Should Be AddStartMarkingVerticies - and handle offset (used in LinearFeature::Finish only)
double CalcStartMarkingVerticies( Point3D *cur, Point3D *next, int wind, int *prev_dir, double *dist1, double *dist2 )
{
    double offset_dir;
    double az1, az2;
    double dist;

    printf("Find start angle for mark: cur(%lf,%lf), next(%lf,%lf)\n", cur->x(), cur->y(), next->x(), next->y());

    // find the offset angle
    geo_inverse_wgs_84( cur->x(), cur->y(), next->x(), next->y(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    printf("offset direction is %lf\n", offset_dir);

    // calculate correct distance for the offset point
    *dist1 = (-LINE_WIDTH);
    *dist2 = (LINE_WIDTH);

    return offset_dir;
}

// TODO: Should Be AddEndMarkingVerticies - and handle offset (used in LinearFeature::Finish only)
double CalcEndMarkingVerticies( Point3D *prev, Point3D *cur, int wind, int *prev_dir, double *dist1, double *dist2 )
{
    double offset_dir;
    double az1, az2;
    double dist;

    printf("Find end angle for mark: prev(%lf,%lf), cur(%lf,%lf)\n", prev->x(), prev->y(), cur->x(), cur->y());

    // find the offset angle
    geo_inverse_wgs_84( prev->x(), prev->y(), cur->x(), cur->y(), &offset_dir, &az2, &dist);
    offset_dir -= 90;
    if (offset_dir < 0)
    {
        offset_dir += 360;
    }

    printf("offset direction is %lf\n", offset_dir);

    // calculate correct distance for the offset point
    *dist1 = (-LINE_WIDTH);
    *dist2 = (LINE_WIDTH);

    return offset_dir;
}

// TODO: Should Be AddMarkingVerticies - and handle offset (used in LinearFeature::Finish only)
double CalcMarkingVerticies( Point3D *prev, Point3D *cur, Point3D *next, int wind, int *prev_dir, double *dist1, double *dist2 )
{
    double offset_dir;
    
    // first, we need to see if we want to average the directions (we have both prev, and next)
    if ( (prev->x() == 0.0f) && (prev->y() == 0.0f) )
    {
        // direction is 90 degrees less than from current to next
        offset_dir = CalcStartMarkingVerticies( cur, next, wind, prev_dir, dist1, dist2 );
        printf("got dist 1: %lf, dist2: %lf\n", *dist1, *dist2);
    }
    else if ( (next->x() == 0.0f) && (next->y() == 0.0f) )
    {
        // direction is 90 degrees less than from current to next
        offset_dir = CalcEndMarkingVerticies( prev, cur, wind, prev_dir, dist1, dist2 );
        printf("got dist 1: %lf, dist2: %lf\n", *dist1, *dist2);
    }
    else
    {
        offset_dir = CalcMiddleMarkingVerticies( prev, cur, next, wind, prev_dir, dist1, dist2 );
        printf("got dist 1: %lf, dist2: %lf\n", *dist1, *dist2);
    }

    printf("return from CalcMarkingVerticies: dirst1: %lf, dist2: %lf\n", *dist1, *dist2);

    return offset_dir;
}

// need to refactor this stuff.
osg::Geode* FinishMarking( osg::Vec3dArray* verticies )
{
    osg::Geometry* stripe = NULL;
    osg::Geode* geode_stripe = NULL;
    osg::StateSet* ss_stripe = NULL;
    osg::PolygonMode* polymode = NULL;
    osg::Vec4Array* col_stripe = NULL;
    int j;

    // set up polygon offset
    osg::PolygonOffset* polyoffset = new osg::PolygonOffset;
    polyoffset->setFactor(-1.00f);
    polyoffset->setUnits(-1.00f);

    polymode = new osg::PolygonMode;
    polymode->setMode(  osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );

    col_stripe = new osg::Vec4Array;
    col_stripe->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));

    // Create a new stripe geometry
    stripe = new osg::Geometry;
    stripe->setColorArray(col_stripe);
    stripe->setColorBinding(osg::Geometry::BIND_OVERALL);

    ss_stripe = new osg::StateSet();
    ss_stripe->setAttributeAndModes(polyoffset,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
#if WIREFRAME
    ss_stripe->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
#endif
    geode_stripe = new osg::Geode();
    geode_stripe->setStateSet(ss_stripe);

    stripe->setVertexArray(verticies);

    // create the index array for the stripe       
    osg::DrawElementsUShort& drawElements = *(new osg::DrawElementsUShort(GL_TRIANGLE_STRIP,verticies->size()));

    for (j=0; j<verticies->size(); j++)
    {
        drawElements[j] = j;
    }
            
    for (int k=0; k<j; k++)
    {
        printf("index %d is %d\n", k, drawElements[k]);
    }

    stripe->addPrimitiveSet(&drawElements);

    geode_stripe->addDrawable(stripe);
   
    return geode_stripe;
}

// TODO: Add this into Linear Feature
osg::Vec3dArray* StartMarking()
{
    osg::Vec3dArray* v_marking = new osg::Vec3dArray;

    return v_marking;
}

// TODO: move this into LinearFeature
osg::Vec3dArray* CheckMarking(int cur_marking, int new_marking, osg::Vec3dArray* v_marking, osg::Group* airport)
{
    // check if we need to finish a marking
    printf("Check Marking : current is %d, new is %d\n", cur_marking, new_marking);

    if ( (v_marking != NULL) && (cur_marking != new_marking) )
    {
        printf("End current marking %d and start new marking %d : mark poly has %d verticies\n", cur_marking, new_marking, v_marking->size() );
        for (int k=0; k<v_marking->size(); k++)
        {
            printf("vertex %d is (%lf, %lf)\n", k, v_marking->at(k).x(), v_marking->at(k).y());
        }

        // END THE CURRENT MARKING
        airport->addChild(FinishMarking( v_marking ) );
        v_marking = NULL;    
    }
            
    // Start recording a new marking
    if ( (v_marking == NULL) && (new_marking != 0) )
    {
        // start a new marking 
        printf("Start new marking %d\n", new_marking);
    
        v_marking = StartMarking();
    }

    return v_marking;    
}
