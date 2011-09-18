
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>

#include <Geometry/poly_support.hxx>
#include <Polygon/polygon.hxx>

#include "beznode.hxx"
#include "runway.hxx"

extern int nudge;

Runway::Runway(char* definition)
{
    double az2;

    // format:
    // runway   width   surface  shoulder  smoothness centerline lights  edge lighting distance remaining signs
    // 100      46.02   2        1         0.00       1                  2             1 
    //     
    //          runway number  runway end lat runway end long   threshold  overrun  markings  approach lighting
    //          09L            33.63470475     -084.44798671    0.00       120.09   3         7 
    // 
    //          touchdown zone lighting  runway end identifier lights
    //          0                        1                              
    //
    //          runway number  runway end lat runway end long   threshold  overrun  markings  approach lighting
    //          27R             33.63469907   -084.40893004     0.00       120.09   3         6 
    //
    //          touchdown zone lighting  runway end identifier lights
    //          0                        1

    // Parse the line
    // 46.02   2   1 0.00 1 2 1 09L  33.63470475 -084.44798671    0.00  120.09 3  7 0 1 27R  33.63469907 -084.40893004    0.00  120.09 3  6 0 1

    // int fscanf(FILE *stream, const char *format, ...);
    sscanf(definition, "%lf %d %d %lf %d %d %d %s %lf %lf %lf %lf %d %d %d %d %s %lf %lf %lf %lf %d %d %d %d", 
        &width,  &surface, &shoulder, &smoothness, &centerline_lights, &edge_lights, &dist_remain_signs, 
        rwnum[0], &lat[0], &lon[0], &threshold[0], &overrun[0], &marking[0], &approach_lights[0], &tz_lights[0], &reil[0],
        rwnum[1], &lat[1], &lon[1], &threshold[1], &overrun[1], &marking[1], &approach_lights[1], &tz_lights[1], &reil[1]
    );

    // calculate runway heading and length (used a lot)
    geo_inverse_wgs_84( lat[0], lon[0], lat[1], lon[1], &heading, &az2, &length );

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read runway: (" << lon[0] << "," << lat[0] << ") to (" << lon[1] << "," << lat[1] << ") heading: " << heading << " length: " << length << " width: " << width );
} 

int Runway::BuildOsg ( osg::Group* airport )
{
    // calculated distance, and azimuths
    double az1, az2, dist;

    // rectangle verticies
    double v0_lat, v0_lon, v1_lat, v1_lon, v2_lat, v2_lon, v3_lat, v3_lon;

    // Create a node for the runway
    osg::Geode* geode = new osg::Geode();

    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3dArray& v = *(new osg::Vec3dArray(4));
    
    // first, find the runway direction vector
    // static int geo_inverse_wgs_84( double lat1, double lon1, double lat2, double lon2, double *az1, double *az2, double *s )
    geo_inverse_wgs_84( lat[0], lon[0], lat[1], lon[1], &az1, &az2, &dist);

    // now, need to caculate the 4 verticies
    // static int geo_direct_wgs_84( double lat1, double lon1, double az1, double s, double *lat2, double *lon2, double *az2 )
    geo_direct_wgs_84( lat[0], lon[0], az1+90, width/2, &v0_lat, &v0_lon, &az2 );
    geo_direct_wgs_84( v0_lat, v0_lon, az1, dist, &v1_lat, &v1_lon, &az2 );
    geo_direct_wgs_84( v1_lat, v1_lon, az1-90, width, &v2_lat, &v2_lon, &az2 );
    geo_direct_wgs_84( v2_lat, v2_lon, az1+180, dist, &v3_lat, &v3_lon, &az2 );

    // convert from lat/lon to geodisc
    // (maybe later) - check some simgear objects...
    v[0].set(v0_lat, v0_lon, 0.0f);
    v[1].set(v1_lat, v1_lon, 0.0f);
    v[2].set(v2_lat, v2_lon, 0.0f);
    v[3].set(v3_lat, v3_lon, 0.0f);

    geometry->setVertexArray(&v);

    osg::DrawElementsUShort& drawElements = *(new osg::DrawElementsUShort(GL_TRIANGLE_STRIP,4));
    geometry->addPrimitiveSet(&drawElements);
    
    drawElements[0] = 3;
    drawElements[1] = 0;
    drawElements[2] = 2;
    drawElements[3] = 1;

    geode->addDrawable(geometry);

    airport->addChild( geode );

    return 0;
}

TGPolygon Runway::gen_wgs84_area(   Point3D origin,
                                    double length_m,
                                    double displ1, double displ2,
                                    double width_m,
                                    double heading_deg,
                                    double alt_m,
                                    bool   add_mid )
{
    TGPolygon result_list;
    double length_hdg = heading_deg;
    double left_hdg = length_hdg - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    // move to the +l end/center of the runway
    Point3D ref = origin;
    double lon, lat, r;
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        length_m / 2.0 - displ2, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the l,-w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                        -width_m / 2.0, &lat, &lon, &r );
    Point3D p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the l,w corner
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,w point (then we add points in a clockwise direction)

        ref = origin;
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                            width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    // move to the -l end/center of the runway
    ref = origin;
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), length_hdg, 
                        displ1 - length_m/2.0, &lat, &lon, &r );
    ref = Point3D( lon, lat, 0.0 );

    // move to the -l,w corner (then we add points in a clockwise direction)
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                        width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    // move to the -l,-w corner
    geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                        -width_m / 2.0, &lat, &lon, &r );
    p = Point3D( lon, lat, 0.0 );
    result_list.add_node( 0, p );

    if ( add_mid ) {
        // move to the 0,-w point (then we add points in a clockwise direction)

        ref = origin;
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg, 
                            -width_m / 2.0, &lat, &lon, &r );
        p = Point3D( lon, lat, 0.0 );
        result_list.add_node( 0, p );
    }

    return result_list;
}


// generate an area for a runway with expansion specified in meters
// (return result points in degrees)
TGPolygon Runway::gen_runway_area_w_extend( double alt_m, double length_extend, double displ1, double displ2, double width_extend ) 
{
    TGPolygon result_list;
    Point3D origin = GetMidpoint();

    result_list = gen_wgs84_area( origin, length + 2.0*length_extend, displ1, displ2, width + 2.0*width_extend, heading, alt_m, false );

    // display points
    SG_LOG(SG_GENERAL, SG_DEBUG, "Results w/ extend (new way)");
    for ( int i = 0; i < result_list.contour_size( 0 ); ++i ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  " << result_list.get_pt(0, i));
    }

    return result_list;
}


// generate an area for a runway and include midpoints
TGPolygon Runway::gen_runway_w_mid( double alt_m, double length_extend_m, double width_extend_m ) 
{
    TGPolygon result_list;
    Point3D origin = GetMidpoint();

    result_list = gen_wgs84_area( origin, length + 2.0*length_extend_m, 0.0, 0.0, width + 2.0 * width_extend_m, heading, alt_m, true );

    // display points
    SG_LOG(SG_GENERAL, SG_DEBUG, "Results w/ mid (new way)");
    for ( int i = 0; i < result_list.contour_size( 0 ); ++i ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  " << result_list.get_pt(0, i));
    }

    return result_list;
}

#if 0
void Runway::gen_simple_rwy( double alt_m, const string& material, superpoly_list *rwy_polys, texparams_list *texparams, TGPolygon *accum )
{
    Point3D mp, end[2];
    int i;

    // create the runway in two halves, as markings may be different in each direction
    end[0] = GetStart();
    mp = GetMidpoint();
    end[1] = GetEnd();

    for (i=0; i<2; i++)
    {
        gen_simple_half( alt_m, mp, end[i], material, rwy_polys, texparams, accum );
    }
}
#endif

void Runway::gen_simple_rwy( double alt_m, const string& material, superpoly_list *rwy_polys, texparams_list *texparams, TGPolygon *accum )
{
    int j, k;

    TGPolygon runway = gen_runway_w_mid( alt_m, 0.0, 0.0 );

    // runway half "a"
    TGPolygon runway_a;
    runway_a.erase();
    runway_a.add_node( 0, runway.get_pt(0, 0) );
    runway_a.add_node( 0, runway.get_pt(0, 1) );
    runway_a.add_node( 0, runway.get_pt(0, 2) );
    runway_a.add_node( 0, runway.get_pt(0, 5) );

    // runway half "b"
    TGPolygon runway_b;
    runway_b.erase();
    runway_b.add_node( 0, runway.get_pt(0, 5) );
    runway_b.add_node( 0, runway.get_pt(0, 2) );
    runway_b.add_node( 0, runway.get_pt(0, 3) );
    runway_b.add_node( 0, runway.get_pt(0, 4) );
	
    Point3D p;
    SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway pts (a half)");
    for ( j = 0; j < runway_a.contour_size( 0 ); ++j ) 
    {
    	p = runway_a.get_pt(0, j);
    	SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    }
    SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway pts (b half)");
    for ( j = 0; j < runway_b.contour_size( 0 ); ++j ) 
    {
    	p = runway_b.get_pt(0, j);
    	SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    }
	
    // do this before clipping and generating the base
    runway_a = remove_dups( runway_a );
    runway_a = reduce_degeneracy( runway_a );

    runway_b = remove_dups( runway_b );
    runway_b = reduce_degeneracy( runway_b );

    TGSuperPoly sp;
    TGTexParams tp;

    TGPolygon clipped_a = tgPolygonDiff( runway_a, *accum );
    TGPolygon split_a = tgPolygonSplitLongEdges( clipped_a, 400.0 );
    sp.erase();
    sp.set_poly( split_a );
    sp.set_material( material );
    rwy_polys->push_back( sp );
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped_a = " << clipped_a.contours());
    *accum = tgPolygonUnion( runway_a, *accum );
    tp = TGTexParams( runway_a.get_pt(0,0), width, length/2.0, heading );
    texparams->push_back( tp );

    TGPolygon clipped_b = tgPolygonDiff( runway_b, *accum );
    TGPolygon split_b = tgPolygonSplitLongEdges( clipped_b, 400.0 );
    sp.erase();
    sp.set_poly( split_b );
    sp.set_material( material );
    rwy_polys->push_back( sp );
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped_b = " << clipped_b.contours());
    *accum = tgPolygonUnion( runway_b, *accum );
    tp = TGTexParams( runway_b.get_pt(0,2), width, length/2.0, heading+180.0 );
    texparams->push_back( tp );

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped runway pts (a)");
    for ( j = 0; j < clipped_a.contours(); ++j ) 
    {
    	for ( k = 0; k < clipped_a.contour_size( j ); ++k ) 
        {
    	    p = clipped_a.get_pt(j, k);
    	    SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    	}
    }

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped runway pts (b)");
    for ( j = 0; j < clipped_b.contours(); ++j ) 
    {
    	for ( k = 0; k < clipped_b.contour_size( j ); ++k ) 
        {
    	    p = clipped_b.get_pt(j, k);
    	    SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
    	}
    }

#if 0    
    gen_runway_stopway( rwy_info, runway_a, runway_b,
    	    		material,
    	    		rwy_polys, texparams, accum );
#endif
}

#if 0
void Runway::gen_marked_rwy( double alt_m, const string& material, superpoly_list *rwy_polys, texparams_list *texparams, TGPolygon *accum )
{
    Point3D mp, end[2];
    int i;

    // create the runway in two halves, as markings may be different in each direction
    end[0] = GetStart();
    mp = GetMidpoint();
    end[1] = GetEnd();

    for (i=0; i<2; i++)
    {
        // first, create half 'a'
        switch( marking[i] )
        {
            case 0:
            case 1: // Visual
                SG_LOG( SG_GENERAL, SG_ALERT, "Half " << i << ": has Visual marking");
                gen_visual_half( alt_m, mp, end[i], material, rwy_polys, texparams, accum );
                break;

            case 2: // non-precision
                SG_LOG( SG_GENERAL, SG_ALERT, "Half " << i << ": has Non Precision marking");
                gen_non_precision_half( alt_m, mp, end[i], material, rwy_polys, texparams, accum );
                break;

            case 3: // precision
                SG_LOG( SG_GENERAL, SG_ALERT, "Half " << i << ": has Precision marking");
                //gen_precision_half( alt_m, mp, end[i], material, rwy_polys, texparams, accum );
                gen_simple_half( alt_m, mp, end[i], material, rwy_polys, texparams, accum );
                break;

            default: // unknown
                SG_LOG( SG_GENERAL, SG_ALERT, "Half " << i << ": has unknown marking");
                break;
        }
    }
}
#endif

int Runway::BuildBtg( float alt_m, superpoly_list* rwy_polys, texparams_list* texparams, TGPolygon* accum, TGPolygon* apt_base, TGPolygon* apt_clearing )
{
    TGPolygon base, safe_base;
    string material;

    if ( surface == 1 /* Asphalt */ ) 
    {
        material = "pa_tiedown";
    } 
    else if ( surface == 2 /* Concrete */ ) 
    {
        material = "pc_tiedown";
    } 
    else if ( surface == 3 /* Turf/Grass */ ) 
    {
        material = "grass_rwy";
    } 
    else if ( surface == 4 /* Dirt */ || surface == 5 /* Gravel */ ) 
    {
        material = "dirt_rwy";
    } 
    else if ( surface == 12 /* Dry Lakebed */ ) 
    {
        material = "dirt_rwy";
    } 
    else if ( surface == 13 /* Water runway (buoy's?) */ ) 
    {
        // water
    } 
    else 
    {
        SG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << surface);
    	throw sg_exception("unknown runway type!");
    }

    // first, check the surface type - anything but concrete and asphalt are easy
    switch( surface )
    {
        case 1: // asphalt:
        case 2: // concrete
            SG_LOG( SG_GENERAL, SG_ALERT, "Build Runway: asphalt or concrete" << surface);
            gen_simple_rwy( alt_m, material, rwy_polys, texparams, accum );
            break;
    
        case 3: // Grass
        case 4: // Dirt
        case 5: // Gravel
            SG_LOG( SG_GENERAL, SG_ALERT, "Build Runway: Turf, Dirt or Gravel" << surface );
	        gen_simple_rwy( alt_m, material, rwy_polys, texparams, accum );
            break;

        case 12: // dry lakebed
            SG_LOG( SG_GENERAL, SG_ALERT, "Build Runway: Dry Lakebed");
            break;

        case 13: // water
            SG_LOG( SG_GENERAL, SG_ALERT, "Build Runway: Water");
            break;

        case 14: // snow
            SG_LOG( SG_GENERAL, SG_ALERT, "Build Runway: Snow");
            break;

        case 15: // transparent
            SG_LOG( SG_GENERAL, SG_ALERT, "Build Runway: transparent");
            break;

        default: // unknown
            SG_LOG( SG_GENERAL, SG_ALERT, "Build Runway: unknown" << surface);
            break;
    }    

    // generate area around runways
    base      = gen_runway_area_w_extend( 0.0, 20.0, -overrun[0], -overrun[1], 20.0 );

    // also clear a safe area around the runway
    safe_base = gen_runway_area_w_extend( 0.0, 180.0, -overrun[0], -overrun[1], 50.0 );

    // add this to the airport clearing
    *apt_clearing = tgPolygonUnion(safe_base, *apt_clearing);

    // and add the clearing to the base
    *apt_base = tgPolygonUnion( base, *apt_base );
}
