#include <stdio.h>
#include <stdlib.h>

#include <simgear/debug/logstream.hxx>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_shapefile.hxx>

#include "global.hxx"
#include "beznode.hxx"
#include "closedpoly.hxx"

#define NO_BEZIER       (0)

static void stringPurifier( std::string& s )
{
    for ( std::string::iterator it = s.begin(), itEnd = s.end(); it!=itEnd; ++it) {
        if ( static_cast<unsigned int>(*it) < 32 || static_cast<unsigned int>(*it) > 127 ) {
            (*it) = ' ';
        }
    }
}

ClosedPoly::ClosedPoly( char* desc ) :
    is_pavement(false),
    is_border(false),
    has_feature(false),
    surface_type(0),
    smoothness(0.0),
    texture_heading(0.0),
    description(std::string(desc ? desc : "none"))
{
    is_border = true;

    stringPurifier(description);

    boundary.clear();
    cur_contour.clear();
}

ClosedPoly::ClosedPoly( int st, float s, float th, char* desc ) :
    ClosedPoly( desc )
{
    surface_type = st;
    smoothness   = s;
    texture_heading = th;

    is_pavement = (surface_type != 15) ? true : false;	// wrong??
    is_border = false;
    has_feature = true;
}

ClosedPoly::~ClosedPoly()
{
    TG_LOG( SG_GENERAL, SG_DEBUG, "Deleting ClosedPoly " << description );
}

void ClosedPoly::AddNode( std::shared_ptr<BezNode> node )
{
    cur_contour.push_back( node );

    TG_LOG(SG_GENERAL, SG_DEBUG, "CLOSEDPOLY::ADDNODE : " << node->GetLoc() );

    // For pavement polys, add a linear feature for each contour
    if (has_feature)
    {
        if (!cur_feature)
        {
            std::string feature_desc = description + " - ";
            if (boundary.size() == 0)
            {
                feature_desc += "hole";
            }
            else
            {
                feature_desc += "boundary";
            }

            TG_LOG(SG_GENERAL, SG_DEBUG, "   Adding node " << node->GetLoc() << " to current linear feature " << cur_feature);
            cur_feature = std::make_shared<LinearFeature>(feature_desc, 1.0f);
        }
        cur_feature->AddNode( node );
    }
}

void ClosedPoly::CloseCurContour()
{
    TG_LOG(SG_GENERAL, SG_DEBUG, "Close Contour");

    // if we are recording a pavement marking - it must be closed -
    // add the first node of the poly
    if (cur_feature)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "We still have an active linear feature - add the first node to close it");
        cur_feature->Finish(true, features.size() );

        features.push_back(cur_feature);
        cur_feature = NULL;
    }

    // add the contour to the poly - first one is the outer boundary
    // subsequent contours are holes
    if ( boundary.size() == 0 )
    {
        boundary = cur_contour;

        // generate the convex hull from the bezcontour node locations
        // CreateConvexHull();

        cur_contour.clear();
    }
    else
    {
        holes.push_back( cur_contour );
        cur_contour.clear();
    }
}

void ClosedPoly::ConvertContour( const BezContour& src, tgContour& dst )
{
    std::shared_ptr<BezNode>    curNode;
    std::shared_ptr<BezNode>    nextNode;

    SGGeod curLoc;
    SGGeod nextLoc;
    SGGeod cp1;
    SGGeod cp2;

    int       curve_type = CURVE_LINEAR;
    double    total_dist;
    int       num_segs = BEZIER_DETAIL;

    TG_LOG(SG_GENERAL, SG_DEBUG, "Creating a contour with " << src.size() << " nodes");

    // clear anything in this point list
    dst.Erase();

    // iterate through each bezier node in the contour
    for (unsigned int i = 0; i < src.size(); ++i)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "\nHandling Node " << i << "\n\n");

        curNode = src.at(i);
        if (i < src.size() - 1)
        {
            nextNode = src.at(i + 1);
        }
        else
        {
            // for the last node, next is the first node, as all contours are closed
            nextNode = src.at(0);
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
                total_dist = CubicDistance( curNode->GetLoc(), cp1, cp2, nextNode->GetLoc() );
            }
            else
            {
                // curve is quadratic using current nodes cp as the cp
                curve_type = CURVE_QUADRATIC;
                cp1 = curNode->GetNextCp();
                total_dist = QuadraticDistance( curNode->GetLoc(), cp1, nextNode->GetLoc() );
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
            }
            else
            {
                // curve is linear
                curve_type = CURVE_LINEAR;
                total_dist = LinearDistance( curNode->GetLoc(), nextNode->GetLoc() );
            }
        }

        if (total_dist < 8.0f)
        {
            if (curve_type != CURVE_LINEAR)
            {
                // If total distance is < 4 meters, then we need to modify num Segments so that each segment >= 2 meters
                num_segs = ((int)total_dist + 1);
                TG_LOG(SG_GENERAL, SG_DEBUG, "Segment from " << curNode->GetLoc() << " to " << nextNode->GetLoc() );
                TG_LOG(SG_GENERAL, SG_DEBUG, "        Distance is " << total_dist << " ( < 16.0) so num_segs is " << num_segs );
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
                // make sure linear segments don't got over 100m
                num_segs = total_dist / 100.0f + 1;
            }
        }

#if NO_BEZIER
        num_segs = 1;
#endif

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

                // add the pavement vertex
                // convert from lat/lon to geo
                // (maybe later) - check some simgear objects...
                dst.AddNode( curLoc );

                if (p==0)
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "adding Curve Anchor node (type " << curve_type << ") at " << curLoc );
                }
                else
                {
                    TG_LOG(SG_GENERAL, SG_DEBUG, "   add bezier node (type  " << curve_type << ") at " << curLoc );
                }

                // now set set cur location for the next iteration
                curLoc = nextLoc;
            }
        }
        else
        {
            if (num_segs > 1)
            {
                for (int p=0; p<num_segs; p++)
                {
                    // calculate next location
                    nextLoc = CalculateLinearLocation( curNode->GetLoc(), nextNode->GetLoc(), (1.0f/num_segs) * (p+1) );

                    // add the feature vertex
                    dst.AddNode( curLoc );

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
                dst.AddNode( curLoc );

                TG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear Anchor node at " << curLoc );

                curLoc = nextLoc;
            }
        }
    }
}

// finish the poly - convert to TGPolygon, and tesselate
void ClosedPoly::Finish()
{
    tgContour          dst_contour;

    // error handling
    if (boundary.size() == 0)
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "no boundary");
    }

    TG_LOG(SG_GENERAL, SG_DEBUG, "Converting a poly with " << holes.size() << " holes");

    if (boundary.size() != 0)
    {
        // create the boundary
        ConvertContour( boundary, dst_contour );
        dst_contour.SetHole( false );

        // and add it to the geometry
        pre_tess.AddContour( dst_contour );

        // Then convert the hole contours
        for (unsigned int i=0; i<holes.size(); i++)
        {
            ConvertContour( holes[i], dst_contour );
            dst_contour.SetHole( true );

            pre_tess.AddContour( dst_contour );
        }

        pre_tess = tgPolygon::Snap( pre_tess, gSnap );
        pre_tess = tgPolygon::RemoveDups( pre_tess );
    }

    // save memory by deleting unneeded resources
    boundary.clear();

    // and the hole contours
    holes.clear();
}

std::string ClosedPoly::GetMaterial( int surface )
{
    std::string material;

    switch( surface ) {
        case 1:
            material = "pa_tiedown"; // Asphalt
            break;

        case 2:
            material = "pc_tiedown"; // Concrete
            break;

        case 3:
            material = "Grass"; // Grass / Turf
            break;

        case 4:
            material = "Dirt"; // Dirt
            break;

        case 5:
            material = "Gravel"; // Gravel
            break;

        case 12:
            material = "lakebed_taxiway"; // Dry lakebed
            break;

        case 13:
            material = "Lake"; // Water
            break;

        case 14:
            material = "SnowCover"; // Snow
            break;

        case 15:
            break;

        default:
            TG_LOG(SG_GENERAL, SG_ALERT, "ClosedPoly::BuildBtg: unknown surface type " << surface_type );
            exit(1);
    }

    return material;
}

int ClosedPoly::BuildBtg( tgpolygon_list& rwy_polys, tgcontour_list& slivers, tgpolygon_list& apt_base_polys, tgpolygon_list& apt_clearing_polys, tgAccumulator& accum, std::string& shapefile_name )
{
    if ( is_pavement && pre_tess.Contours() )
    {
        tgPolygon base, safe_base;

        BuildBtg( rwy_polys, slivers, accum, shapefile_name );

        base = tgPolygon::Expand( pre_tess, 20.0 );
        safe_base = tgPolygon::Expand( pre_tess, 50.0);

        // add this to the airport clearing
        apt_clearing_polys.push_back( safe_base );

        // and add the clearing to the base
        apt_base_polys.push_back( base );
    }

    // clean up to save ram : we're done here...
    return 1;
}

int ClosedPoly::BuildBtg( tgpolygon_list& rwy_polys, tgcontour_list& slivers, tgAccumulator& accum, std::string& shapefile_name )
{
    if ( is_pavement && pre_tess.Contours() )
    {
        char layer[128];
    
        if(  shapefile_name.size() ) {
            sprintf( layer, "%s_preclip", shapefile_name.c_str() );
            tgShapefile::FromPolygon( pre_tess, "./airport_dbg", layer, std::string("preclip") );
            
            pre_tess.Tesselate();
            sprintf( layer, "%s_preclip_tris", shapefile_name.c_str() );
            tgShapefile::FromTriangles( pre_tess, "./airport_dbg", layer, std::string("preclip") );

            //accum.ToShapefiles( "./airport_dbg", "accum", true );
        }

        tgPolygon clipped = accum.Diff( pre_tess );
        if ( clipped.Contours() ) {
            if(  shapefile_name.size() ) {
                sprintf( layer, "%s_postclip", shapefile_name.c_str() );
                tgShapefile::FromPolygon( clipped, "./airport_dbg", layer, std::string("postclip") );
            }

            // tgPolygon::RemoveSlivers( clipped, slivers );

            if(  shapefile_name.size() ) {
                clipped.Tesselate();
                sprintf( layer, "%s_postclip_tris", shapefile_name.c_str() );
                tgShapefile::FromTriangles( clipped, "./airport_dbg", layer, std::string("postclip") );                
            }
            
            clipped.SetMaterial( GetMaterial( surface_type ) );
            clipped.SetTexParams( clipped.GetNode(0,0), 5.0, 5.0, texture_heading );
            clipped.SetTexLimits( 0.0, 0.0, 1.0, 1.0 );
            clipped.SetTexMethod( TG_TEX_BY_TPS_NOCLIP );

            rwy_polys.push_back( clipped );
            
            accum.Add( pre_tess );                
        }
    }

    return 1;
}


// Just used for user defined border - add a little bit, as some modelers made the border exactly on the edges
// - resulting in no base, which we can't handle
int ClosedPoly::BuildBtg( tgPolygon& apt_base, tgPolygon& apt_clearing, std::string& shapefile_name )
{
    tgPolygon base, safe_base;

    // verify the poly has been generated, and the contour isn't a pavement
    if ( is_border && pre_tess.Contours() )
    {
        base = tgPolygon::Expand( pre_tess, 2.0);
        safe_base = tgPolygon::Expand( pre_tess, 5.0);

        // add this to the airport clearing
        apt_clearing = tgPolygon::Union( safe_base, apt_clearing);

        // and add the clearing to the base
        apt_base = tgPolygon::Union( base, apt_base );
    }

    return 1;
}