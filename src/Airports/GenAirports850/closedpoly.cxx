#include <stdlib.h>

#include <simgear/debug/logstream.hxx>

#include <terragear/tg_shapefile.hxx>

#include "global.hxx"
#include "beznode.hxx"
#include "closedpoly.hxx"
#include "airport.hxx"

#define NO_BEZIER       (0)

static void stringPurifier( std::string& s )
{
    for ( std::string::iterator it = s.begin(), itEnd = s.end(); it!=itEnd; ++it) {
        if ( static_cast<unsigned int>(*it) < 32 || static_cast<unsigned int>(*it) > 127 ) {
            (*it) = ' ';
        }
    }
}

ClosedPoly::ClosedPoly( char* desc )
{
    is_pavement = false;
    is_border   = true;
    has_feature = false;

    if ( desc )
    {
        description = desc;
        stringPurifier(description);
    }
    else
    {
        description = "none";
    }

    boundary = NULL;
    cur_contour = NULL;
    cur_feature = NULL;
}

ClosedPoly::ClosedPoly( int st, float s, float th, char* desc )
{
    surface_type = st;
    smoothness   = s;
    texture_heading = th;

    is_pavement = (surface_type != 15) ? true : false;	// wrong??
    is_border   = false;
    has_feature = true;

    if ( desc )
    {
        description = desc;
        stringPurifier(description);
    }
    else
    {
        description = "none";
    }

    boundary = NULL;
    cur_contour = NULL;
    cur_feature = NULL;
}

ClosedPoly::~ClosedPoly()
{
    TG_LOG( SG_GENERAL, SG_DEBUG, "Deleting ClosedPoly " << description );
}

void ClosedPoly::AddNode( BezNode* node )
{
    // if this is the first node of the contour - create a new contour
    if (!cur_contour)
    {
        cur_contour = new BezContour;
    }
    cur_contour->push_back( node );

    TG_LOG(SG_GENERAL, SG_DEBUG, "CLOSEDPOLY::ADDNODE : " << node->GetLoc() );

    // For pavement polys, add a linear feature for each contour
    if (has_feature)
    {
        if (!cur_feature)
        {
            std::string feature_desc = description + " - ";
            if (boundary)
            {
                feature_desc += "hole";
            }
            else
            {
                feature_desc += "boundary";
            }

            TG_LOG(SG_GENERAL, SG_DEBUG, "   Adding node " << node->GetLoc() << " to current linear feature " << cur_feature);

            // todo : how can we offset the feature on the pavement? - send it to intersection generator?
            cur_feature = new LinearFeature(feature_desc, 1.0f );
        }
        cur_feature->AddNode( node );
    }
}

void ClosedPoly::CloseCurContour( Airport* ap )
{
    TG_LOG(SG_GENERAL, SG_DEBUG, "Close Contour");

    // if we are recording a pavement marking - it must be closed -
    // add the first node of the poly
    if (cur_feature)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "We still have an active linear feature - add the first node to close it");
        cur_feature->Finish(ap, true);

        features.push_back(cur_feature);
        cur_feature = NULL;
    }

    // add the contour to the poly - first one is the outer boundary
    // subsequent contours are holes
    if ( boundary == NULL )
    {
        boundary = cur_contour;

        // generate the convex hull from the bezcontour node locations
        // CreateConvexHull();

        cur_contour = NULL;
    }
    else
    {
        holes.push_back( cur_contour );
        cur_contour = NULL;
    }
}

cgalPoly_Polygon ClosedPoly::ConvertContour( BezContour* src )
{
    BezNode*    curNode;
    BezNode*    nextNode;

    SGGeod curLoc;
    SGGeod nextLoc;
    SGGeod cp1;
    SGGeod cp2;

    std::vector<cgalPoly_Point> dst_points;
    
    int       curve_type = CURVE_LINEAR;
    double    total_dist;
    int       num_segs = BEZIER_DETAIL;

    TG_LOG(SG_GENERAL, SG_DEBUG, "Creating a contour with " << src->size() << " nodes");

    // clear anything in this point list
    // dst.clear();

    // iterate through each bezier node in the contour
    for (unsigned int i = 0; i <= src->size()-1; i++)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "\nHandling Node " << i << "\n\n");

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
                dst_points.push_back( cgalPoly_Point(curLoc.getLongitudeDeg(), curLoc.getLatitudeDeg()) );

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
                    dst_points.push_back( cgalPoly_Point(curLoc.getLongitudeDeg(), curLoc.getLatitudeDeg()) );

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
                dst_points.push_back( cgalPoly_Point(curLoc.getLongitudeDeg(), curLoc.getLatitudeDeg()) );

                TG_LOG(SG_GENERAL, SG_DEBUG, "adding Linear Anchor node at " << curLoc );

                curLoc = nextLoc;
            }
        }
    }
    
    return cgalPoly_Polygon( dst_points.begin(), dst_points.end() );
}

// finish the poly - convert to TGPolygon, and tesselate
void ClosedPoly::Finish()
{
    cgalPoly_Polygon                polyBoundary;
    std::vector<cgalPoly_Polygon>   polyHoles;

    // error handling
    if (boundary == NULL)
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "no boundary");
    }

    TG_LOG(SG_GENERAL, SG_DEBUG, "Converting a poly with " << holes.size() << " holes");

    if (boundary != NULL)
    {
        // create the boundary
        polyBoundary = ConvertContour( boundary );

        // Then convert the hole contours
        for (unsigned int i=0; i<holes.size(); i++)
        {
            polyHoles.push_back( ConvertContour( holes[i] ) );
        }

//        pre_tess.Snap(gSnap);
//        pre_tess.RemoveDups();
//        pre_tess.RemoveBadContours();

        // generate a polygon set
        pre_tess = cgalPoly_PolygonWithHoles( polyBoundary, polyHoles.begin(), polyHoles.end());        
    }

    // save memory by deleting unneeded resources
    for (unsigned int i=0; i<boundary->size(); i++)
    {
        delete boundary->at(i);
    }
    delete boundary;
    boundary = NULL;

    // and the hole contours
    for (unsigned int i=0; i<holes.size(); i++)
    {
        for (unsigned int j=0; j<holes[i]->size(); j++)
        {
            delete holes[i]->at(j);
        }
    }

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

tgPolygonSetList& ClosedPoly::GetPolys( void )
{
    if ( is_pavement && !pre_tess.is_unbounded() )
    {
        // get texture reference point
        cgalPoly_Polygon ob = pre_tess.outer_boundary();
        cgalPoly_Polygon::Vertex_iterator it = ob.vertices_begin();
        
        tgPolygonSetMeta    meta( tgPolygonSetMeta::META_TEXTURED, "Grass", description.c_str() );    
        meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );        
        meta.setMaterial( GetMaterial( surface_type ) );
        meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_TPS_NOCLIP );
        meta.setTextureRef( *it, 5.0l, 5.0l, texture_heading );
        meta.setTextureLimits( 0.0l, 0.0l, 1.0l, 1.0l );

        pavement_polys.push_back( tgPolygonSet( pre_tess, meta ) );
    }
    
    return pavement_polys;
}

tgPolygonSetList& ClosedPoly::GetFeaturePolys( void )
{   
    for ( unsigned int i = 0; i < features.size(); i++)
    {
        tgPolygonSetList featPolys = features[i]->GetPolys();
        feature_polys.insert( feature_polys.end(), featPolys.begin(), featPolys.end() );
    }
    
    return feature_polys;
}

tgPolygonSetList& ClosedPoly::GetFeatureCapPolys( void )
{
    for ( unsigned int i = 0; i < features.size(); i++)
    {
        tgPolygonSetList featCapPolys = features[i]->GetCapPolys();
        feature_cap_polys.insert( feature_cap_polys.end(), featCapPolys.begin(), featCapPolys.end() );
    }
    
    return feature_cap_polys;
}

void ClosedPoly::GetFeatureLights( tglightcontour_list& lights )
{
    for ( unsigned int i = 0; i < features.size(); i++)
    {
        features[i]->GetLights( lights );
    }    
}

tgPolygonSetList& ClosedPoly::GetInnerBasePolys( void )
{
    char desc[256];
    snprintf( desc, 256, "%s_innerbase", description.c_str() );
    
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, "Grass", desc );    
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );
    
    tgPolygonSet b( pre_tess, meta );
    tgPolygonSet o = b.offset( 20.0l );
    
    inner_base_polys.push_back( o );
    
    return inner_base_polys;
}

tgPolygonSetList& ClosedPoly::GetOuterBasePolys( void )
{
    char desc[256];
    snprintf( desc, 256, "%s_outerbase", description.c_str() );
    
    tgPolygonSetMeta    meta( tgPolygonSetMeta::META_TEXTURED, "Grass", desc );    
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );
    
    tgPolygonSet b( pre_tess, meta );
    tgPolygonSet o = b.offset( 50.0l );
    
    outer_base_polys.push_back( o );
    
    return outer_base_polys;
}

tgPolygonSetList& ClosedPoly::GetInnerBoundaryPolys( void )
{
    char desc[256];
    snprintf( desc, 256, "%s_innerbase", description.c_str() );
    
    tgPolygonSetMeta    meta( tgPolygonSetMeta::META_TEXTURED, "Grass", desc );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );
    
    tgPolygonSet b( pre_tess, meta );
    tgPolygonSet o = b.offset( 20.0l );
    
    inner_boundary_polys.push_back( o );
    
    return inner_boundary_polys;
}

tgPolygonSetList& ClosedPoly::GetOuterBoundaryPolys( void )
{
    char desc[256];
    snprintf( desc, 256, "%s_outerbase", description.c_str() );
    
    tgPolygonSetMeta    meta( tgPolygonSetMeta::META_TEXTURED, "Grass", desc );    
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );
    
    tgPolygonSet b( pre_tess, meta );
    tgPolygonSet o = b.offset( 50.0l );
    
    outer_boundary_polys.push_back( o );
    
    return outer_boundary_polys;
}