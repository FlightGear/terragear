#include <stdlib.h>
#include <list>

#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include <osg/Geode>
#include <osg/PolygonMode>
#include <osgUtil/Tessellator>

#include <Geometry/poly_support.hxx>

#include "beznode.hxx"
#include "convex_hull.hxx"
#include "closedpoly.hxx"


ClosedPoly::ClosedPoly( int st, float s, float th, char* desc )
{
    surface_type = st;
    smoothness   = s;
    texture_heading = th;
    if ( desc )
    {
        description = desc;
    }
    else
    {
        description = "none";
    }
        
    boundary = NULL;
    cur_contour = NULL;
    cur_feature = NULL;
}

void ClosedPoly::AddNode( BezNode* node )
{
    // if this is the first node of the contour - create a new contour
    if (!cur_contour)
    {
        cur_contour = new BezContour;
    }
    cur_contour->push_back( node );


    SG_LOG(SG_GENERAL, SG_DEBUG, "CLOSEDPOLY::ADDNODE : (" << node->GetLoc().x() << "," << node->GetLoc().y() << ")");


    // TODO: Create ONE linear feature for each contour.  
    // Parse the polys in linear feature
    // if recording a linear feature on the pavement, add this node
    // to it as well
    // TODO: just doing marking now, need lighting as well
    if (!cur_feature)
    {
        string feature_desc = description + ":";
        if (boundary)
        {
            feature_desc += "hole";
        }
        else
        {
            feature_desc += "boundary";
        }

        SG_LOG(SG_GENERAL, SG_DEBUG, "   Adding node (" << node->GetLoc().x() << "," << node->GetLoc().y() << ") to current linear feature " << cur_feature);
        cur_feature = new LinearFeature(feature_desc, 1.0f );
    } 
    cur_feature->AddNode( node );
}

void ClosedPoly::CreateConvexHull( void )
{
    TGPolygon  convexHull;
    point_list nodes;
    Point3D    p;
    int        i;

    if (boundary->size() > 2){
        for (i=0; i<boundary->size(); i++)
        {

            p = boundary->at(i)->GetLoc();
            nodes.push_back( p );
        }
        convexHull = convex_hull( nodes );
        hull = convexHull.get_contour(0);
    } else
        SG_LOG(SG_GENERAL, SG_ALERT, "Boundary size too small: " << boundary->size() << ". Ignoring..." );
}

int ClosedPoly::CloseCurContour()
{
    SG_LOG(SG_GENERAL, SG_DEBUG, "Close Contour");

    // if we are recording a pavement marking - it must be closed - 
    // add the first node of the poly
    if (cur_feature)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "We still have an active linear feature - add the first node to close it");
        cur_feature->Finish();

        features.push_back(cur_feature);
        cur_feature = NULL;        
    }

    // add the contour to the poly - first one is the outer boundary
    // subsequent contours are holes
    if ( boundary == NULL )
    {
        boundary = cur_contour;

        // generate the convex hull from the bezcontour node locations
        CreateConvexHull();

        cur_contour = NULL;
    }
    else
    {
        holes.push_back( cur_contour );
        cur_contour = NULL;
    }
}

void ClosedPoly::ConvertContour( BezContour* src, point_list *dst )
{
    BezNode*    prevNode;
    BezNode*    curNode;
    BezNode*    nextNode;
        
    Point3D prevLoc;
    Point3D curLoc;
    Point3D nextLoc;
    Point3D cp1;
    Point3D cp2;    

    int start, end, inc;        

    int curve_type = CURVE_LINEAR;
    int i;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Creating a contour with " << src->size() << " nodes");

    // clear anything in this point list
    dst->empty();

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

                // add the pavement vertex
                // convert from lat/lon to geo
                // (maybe later) - check some simgear objects...
                dst->push_back( curLoc );
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
            dst->push_back( curLoc );
            SG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear Anchor node at (" << curLoc.x() << "," << curLoc.y() << ")");
        }
    }

    // Add the first point again?
    // dst->push_back( src->at(0)->GetLoc() );
}

void ExpandPoint( Point3D *prev, Point3D *cur, Point3D *next, double expand_by, double *heading, double *offset )
{
    int    turn_direction;
    bool   reverse_dir = false;
    double offset_dir;
    double next_dir;
    double az1, az2;
    double dist;

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
    geo_inverse_wgs_84( avg.y(), avg.x(), 0.0f, 0.0f, &offset_dir, &az2, &dist);

    // find the direction to the next point
    geo_inverse_wgs_84( cur->y(), cur->x(), next->y(), next->x(), &next_dir, &az2, &dist);

    // calculate correct distance for the offset point
    *offset  = (expand_by)/sin(SGMiscd::deg2rad(offset_dir-next_dir));
    *heading = offset_dir;

    SG_LOG(SG_GENERAL, SG_DEBUG, "heading is " << *heading << " distance is " << *offset );
}

void ClosedPoly::ExpandContour( point_list& src, TGPolygon& dst, double dist )
{
    point_list expanded_boundary;
    Point3D prevPoint, curPoint, nextPoint;
    double theta;
    double expand_by;
    double expanded_x, expanded_y;
    Point3D expanded_point;

    double h1, h2, h3;
    double o1, o2, o3;
    double az2;
    int i;
        
    // iterate through each bezier node in the contour
    for (i=0; i<src.size(); i++)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "\nExpanding point " << i << "\n\n");

        if (i == 0)
        {
            // set prev node to last in the contour, as all contours must be closed
            prevPoint = src.at( src.size()-1 );
        }
        else
        {
            // otherwise, it's just the last index
            prevPoint = src.at( i-1 );
        }

        curPoint = src.at(i);

        if (i<src.size()-1)
        {
            nextPoint = src.at(i+1);
        }
        else
        {
            // for the last node, next is the first. as all contours are closed
            nextPoint = src.at(0);
        }

        // calculate the angle between cur->prev and cur->next
        theta = SGMiscd::rad2deg(CalculateTheta(prevPoint, curPoint, nextPoint));

        if ( theta < 90.0 )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "\nClosed POLY case 1 (theta < 90) " << description << ": theta is " << theta );

            // calculate the expanded point heading and offset from current point
            ExpandPoint( &prevPoint, &curPoint, &nextPoint, dist, &h1, &o1 );

            if (o1 > dist*2.0)
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "\ntheta is " << theta << " distance is " << o1 << " CLAMPING to " << dist*2 );
                o1 = dist*2;

                if (o1 > 100)
                {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "\nClosed POLY " << description << ": theta is " << theta << " distance is " << o1 << " WHAT HAPPENED " );
                    exit(1);
                }
            }

            geo_direct_wgs_84( curPoint.y(), curPoint.x(), h1, o1, &expanded_y, &expanded_x, &az2 );
            expanded_point = Point3D( expanded_x, expanded_y, 0.0f );
            expanded_boundary.push_back( expanded_point );
        }
        else if ( abs(theta - 180.0) < 0.1 )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "\nClosed POLY case 2 (theta close to 180) " << description << ": theta is " << theta );

            // calculate the expanded point heading and offset from current point
            ExpandPoint( &prevPoint, &curPoint, &nextPoint, dist, &h1, &o1 );

            // straight line blows up math - dist should be exactly as given
            o1 = dist;

            geo_direct_wgs_84( curPoint.y(), curPoint.x(), h1, o1, &expanded_y, &expanded_x, &az2 );
            expanded_point = Point3D( expanded_x, expanded_y, 0.0f );
            expanded_boundary.push_back( expanded_point );
        }
        else
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "\nClosed POLY case 3 (fall through) " << description << ": theta is " << theta );

            // calculate the expanded point heading and offset from current point
            ExpandPoint( &prevPoint, &curPoint, &nextPoint, dist, &h1, &o1 );

            if (o1 > dist*2)
            {
                SG_LOG(SG_GENERAL, SG_DEBUG, "\ntheta is " << theta << " distance is " << o1 << " WHAT HAPPENED " );
                exit(1);
            }

            geo_direct_wgs_84( curPoint.y(), curPoint.x(), h1, o1, &expanded_y, &expanded_x, &az2 );
            expanded_point = Point3D( expanded_x, expanded_y, 0.0f );
            expanded_boundary.push_back( expanded_point );
        }
    }

    dst.add_contour( expanded_boundary, 9 );
}

osg::DrawArrays* ClosedPoly::CreateOsgPrimitive( point_list contour, osg::Vec3Array* vpave )
{
    int i;
    Point3D p;
    osg::DrawArrays* primitive;
    int start_vert = vpave->size();

    for (i=0; i<contour.size(); i++)
    {
        p = contour[i];
        vpave->push_back( osg::Vec3d(p.x(), p.y(), 0.0) );
    }

    primitive = new osg::DrawArrays(osg::PrimitiveSet::POLYGON, start_vert, vpave->size()-start_vert);    

    return primitive;
}

// finish the poly - convert to TGPolygon, and tesselate
int ClosedPoly::Finish()
{
    BezContour*         src_contour;
    point_list          dst_contour;
    BezNode*            node;

    // error handling
    if (boundary == NULL)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "no boundary");
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Converting a poly with " << holes.size() << " holes");
    
    if (boundary != NULL)
    {
        // create the boundary
        ConvertContour( boundary, &dst_contour );
    
        // and add it to the geometry 
        pre_tess.add_contour( dst_contour, 0 );

        // The convert the hole contours
        for (int i=0; i<holes.size(); i++)
        {
            dst_contour.clear();
            ConvertContour( holes[i], &dst_contour );
            pre_tess.add_contour( dst_contour, 1 );
        }
    }

    // save memory by deleting unneeded resources
    delete boundary;
    boundary = NULL;

    // The convert the hole contours
    holes.clear();
}

int ClosedPoly::BuildOsg( osg::Group* airport )
{
    BezContour*         contour;
    BezNode*            node;
    osg::Geode*         geode_pave; // has the stateset, and geometry for this poly
    osg::Geometry*      pavement;
    osg::StateSet*      ss_pave;    // just a simple stateset for now
    osg::PolygonMode*   polymode;
    osg::Vec4Array*     col_pave;   // just grey for now...
    osg::Vec3Array*     v_pave;     // ALL verticies (boundary and holes)
    osg::DrawArrays*    primitive;  // an index array for boundary / hole
    int                 num_verts;  // number of verticies in a contour
    int                 i;          // ?

    if ( pre_tess.contours() )
    {
        // Setup a polygonmode for wireframe debuging
        polymode = new osg::PolygonMode;
        polymode->setMode(  osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );

        // Create a drawable for the pavement poly (boundary and holes)
        pavement = new osg::Geometry;

        // make pavement grey : do this when reading the poly
        col_pave = new osg::Vec4Array;
        col_pave->push_back(osg::Vec4(0.5f,0.5f,0.5f,1.0f));
        pavement->setColorArray(col_pave);
        pavement->setColorBinding(osg::Geometry::BIND_OVERALL);

        // set up pavement stateset
        geode_pave = new osg::Geode();
        ss_pave = new osg::StateSet();
    
#if WIREFRAME
        ss_pave->setAttribute( polymode );
#endif
        geode_pave->setStateSet(ss_pave);

        v_pave = new osg::Vec3Array;
        pavement->setVertexArray(v_pave);

        // create the boundary
    	SG_LOG(SG_GENERAL, SG_DEBUG, "  Adding pavement boundary");
        primitive = CreateOsgPrimitive( pre_tess.get_contour( 0 ), v_pave );
        if (hull.size() > 0)
        {
        	SG_LOG(SG_GENERAL, SG_DEBUG, "  boundary has  " << hull.size() << "verts" );
        }

        // add the hole contours
        for (i=1; i<pre_tess.contours(); i++)
        {
            primitive = CreateOsgPrimitive( pre_tess.get_contour( i ), v_pave );
            pavement->addPrimitiveSet(primitive);
        }
        
        osgUtil::Tessellator *tess = new osgUtil::Tessellator;
        tess->setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
        tess->retessellatePolygons(*pavement);

        geode_pave->addDrawable(pavement);
        airport->addChild(geode_pave);

#if 0   // debug ExpandPoly
        TGPolygon base;
        ExpandPoly( hull, base, 20 );
        primitive = CreateOsgPrimitive( base.get_contour( 0 ), v_pave );
        pavement->addPrimitiveSet(primitive);
#endif

        geode_pave->addDrawable(pavement);
        airport->addChild(geode_pave);

        // add the pavement markings
#if 0
        printf("ClosedPoly::Finish - Finish %d linear features\n", features.size());
        for (int feat=0; feat<features.size(); feat++)
        {
            features.at(feat)->Finish(airport);
        }
#endif
    }
}

int ClosedPoly::BuildBtg( float alt_m, superpoly_list* rwy_polys, texparams_list* texparams, TGPolygon* accum, TGPolygon* apt_base, TGPolygon* apt_clearing )
{
    TGPolygon base, safe_base;
    string material;
    int j, k;

    switch( surface_type )
    {
        case 1:
            material = "pa_tiedown";
            break;

        case 2:
            material = "pc_tiedown";
            break;

        case 3:
            material = "grass_rwy";
            break;
// TODO Differentiate more here:
        case 4:
        case 5:
        case 12:
        case 13:
        case 14:
        case 15:
            material = "grass_rwy";
            break;

        default:
            SG_LOG(SG_GENERAL, SG_ALERT, "ClosedPoly::BuildBtg: unknown surface type " << surface_type );
            exit(1);
    }

    // verify the poly has been generated
    if ( pre_tess.contours() )    
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: original poly has " << pre_tess.contours() << " contours");
    
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

        TGPolygon clipped = tgPolygonDiff( pre_tess, *accum );
        SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: clipped poly has " << clipped.contours() << " contours");

        TGPolygon split   = tgPolygonSplitLongEdges( clipped, 400.0 );
        SG_LOG(SG_GENERAL, SG_DEBUG, "BuildBtg: split poly has " << split.contours() << " contours");

        sp.erase();
        sp.set_poly( split );
        sp.set_material( material );
        //sp.set_flag("taxi");

        rwy_polys->push_back( sp );
        SG_LOG(SG_GENERAL, SG_DEBUG, "clipped = " << clipped.contours());
        *accum = tgPolygonUnion( pre_tess, *accum );
        tp = TGTexParams( pre_tess.get_pt(0,0), 5.0, 5.0, texture_heading );
        texparams->push_back( tp );

        ExpandContour( hull, base, 20.0 );
        ExpandContour( hull, safe_base, 50.0 );
        
        // add this to the airport clearing
        *apt_clearing = tgPolygonUnion( safe_base, *apt_clearing);

        // and add the clearing to the base
        *apt_base = tgPolygonUnion( base, *apt_base );
    }

    // clean up to save ram : we're done here...


    return 1;
}
