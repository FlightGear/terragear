#include "beznode.hxx"
#include "runway.hxx"
#include "helipad.hxx"
#include "airport.hxx"

#include <list>
#include <map>
#include <string>

#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/misc/texcoord.hxx>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/chop.hxx>

#include <Geometry/poly_support.hxx>
#include <Output/output.hxx>

#include "elevations.hxx"
#include "poly_extra.hxx"

Airport::Airport( int c, char* def)
{
    int   numParams;
    char  d[100];
    char  tmp[16];
    int   x, y;

    code = c;

    numParams = sscanf(def, "%d %d %d %s %ls", &altitude, &x, &y, tmp, d);

    altitude *= SG_FEET_TO_METER;
    icao = tmp;
    description = d;
    boundary = NULL;
}

// TODO: fix OSG - it was nice, but unnecesary...
void Airport::BuildOsg( osg::Group* airport )
{
    // draw for OSG
    int i;

#if 0
    for (i=0; i<runways.size(); i++)
    {
        runways.at(i)->BuildOsg( airport );
    }
#endif

    for (i=0; i<pavements.size(); i++)
    {
    	SG_LOG(SG_GENERAL, SG_DEBUG, " Adding pavement " << i << "to airport");
        pavements.at(i)->BuildOsg(airport);
    }

#if 0
    for (i=0; i<features.size(); i++)
    {
        features.at(i)->Finish(airport);
    }
#endif
}


// TODO: Add to runway object
// calculate texture coordinates for runway section using the provided
// texturing parameters.  Returns a mirror polygon to the runway,
// except each point is the texture coordinate of the corresponding
// point in the original polygon.
static TGPolygon rwy_section_tex_coords( const TGPolygon& in_poly, const TGTexParams& tp, const bool clip_result )
{
    int i, j;
    TGPolygon result;
    result.erase();

    Point3D ref = tp.get_ref();
    double width = tp.get_width();
    double length = tp.get_length();
    double heading = tp.get_heading();
    double minu = tp.get_minu();
    double maxu = tp.get_maxu();
    double minv = tp.get_minv();
    double maxv = tp.get_maxv();
    SG_LOG( SG_GENERAL, SG_DEBUG, "section ref = " << ref );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  width = " << width );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  length = " << length );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  heading = " << heading );
    Point3D p, t;
    double x, y, tx, ty;

    for ( i = 0; i < in_poly.contours(); ++i ) {
    	for ( j = 0; j < in_poly.contour_size( i ); ++j ) {
    	    p = in_poly.get_pt( i, j );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "point = " << p);

    	    //
    	    // 1. Calculate distance and bearing from the center of
    	    // the runway
    	    //

    	    // given alt, lat1, lon1, lat2, lon2, calculate starting
    	    // and ending az1, az2 and distance (s).  Lat, lon, and
    	    // azimuth are in degrees.  distance in meters
    	    double az1, az2, dist;
    	    geo_inverse_wgs_84( 0, ref.y(), ref.x(), p.y(), p.x(),
    				&az1, &az2, &dist );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "basic course = " << az2);

    	    //
    	    // 2. Rotate this back into a coordinate system where Y
    	    // runs the length of the runway and X runs crossways.
    	    //

    	    double course = az2 - heading;
    	    while ( course < -360 ) { course += 360; }
    	    while ( course > 360 ) { course -= 360; }
    	    SG_LOG( SG_GENERAL, SG_DEBUG,
                    "  course = " << course << "  dist = " << dist );

    	    //
    	    // 3. Convert from polar to cartesian coordinates
    	    //
    
    	    x = sin( course * SGD_DEGREES_TO_RADIANS ) * dist;
    	    y = cos( course * SGD_DEGREES_TO_RADIANS ) * dist;
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  x = " << x << " y = " << y);

    	    //
    	    // 4. Map x, y point into texture coordinates
    	    //
    	    double tmp;

            tmp = x / width;
            tx = tmp * (maxu - minu) + minu;
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

            if ( clip_result) {
                if ( tx < 0.0 ) { tx = 0.0; }
                if ( tx > 1.0 ) { tx = 1.0; }
            }

    	    ty = y / length;
            tmp = y / length;
            ty = tmp * (maxv - minv) + minv;
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << ty << ")");

            if ( clip_result ) {
                if ( ty < 0.0 ) { ty = 0.0; }
                if ( ty > 1.0 ) { ty = 1.0; }
            }

    	    t = Point3D( tx, ty, 0 );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ", " << ty << ")");

    	    result.add_node( i, t );
    	}
    }

    return result;
}

// TODO: add to linear feature class 
static TGPolygon linear_feature_tex_coords( const TGPolygon& in_poly, const TGTexParams& tp )
{
    int i, j;
    TGPolygon result;
    result.erase();

    Point3D ref = tp.get_ref();
    double width = tp.get_width();
    double length = tp.get_length();
    double heading = tp.get_heading();
    double minu = tp.get_minu();
    double maxu = tp.get_maxu();
    double minv = tp.get_minv();
    double maxv = tp.get_maxv();
    SG_LOG( SG_GENERAL, SG_DEBUG, "section ref = " << ref );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  width   = " << width );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  length  = " << length );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  heading = " << heading );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  minv    = " << minv );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  maxv    = " << maxv );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  heading = " << heading );

    Point3D p, t;
    double x, y, tx, ty;

    for ( i = 0; i < in_poly.contours(); ++i ) 
    {
    	for ( j = 0; j < in_poly.contour_size( i ); ++j ) 
        {
    	    p = in_poly.get_pt( i, j );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "tex coords for contour " << i << "point " << j << ": " << p );

    	    //
    	    // 1. Calculate distance and bearing from the center of
    	    // the feature
    	    //

    	    // given alt, lat1, lon1, lat2, lon2, calculate starting
    	    // and ending az1, az2 and distance (s).  Lat, lon, and
    	    // azimuth are in degrees.  distance in meters
    	    double az1, az2, dist;
    	    geo_inverse_wgs_84( 0, ref.y(), ref.x(), p.y(), p.x(),
    				&az1, &az2, &dist );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "basic course from ref = " << az2);

    	    //
    	    // 2. Rotate this back into a coordinate system where Y
    	    // runs the length of the runway and X runs crossways.
    	    //

    	    double course = az2 - heading;
    	    while ( course < -360 ) { course += 360; }
    	    while ( course > 360 ) { course -= 360; }
    	    SG_LOG( SG_GENERAL, SG_DEBUG,
                        "  course = " << course << "  dist = " << dist );

    	    //
    	    // 3. Convert from polar to cartesian coordinates
    	    //

    	    x = sin( course * SGD_DEGREES_TO_RADIANS ) * dist;
    	    y = cos( course * SGD_DEGREES_TO_RADIANS ) * dist;
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  x = " << x << " y = " << y);

    	    //
    	    // 4. Map x, y point into texture coordinates
    	    //
    	    double tmp;

            tmp = x / width;
            tx = tmp * (maxu - minu) + minu;

            if ( tx < -1.0 )  { tx = -1.0; }
            if ( tx > 1.0 ) { tx = 1.0; }

    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

            ty = (y/length) + minv;
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << ty << ")");

    	    t = Point3D( tx, ty, 0 );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ", " << ty << ")");

    	    result.add_node( i, t );
    	}
    }

    return result;
}

// TODO : Add somewhere
// Determine node elevations of a point_list based on the provided
// TGAptSurface.  Offset is added to the final elevation
static point_list calc_elevations( TGAptSurface &surf,
                                   const point_list& geod_nodes,
                                   double offset )
{
    point_list result = geod_nodes;
    for ( unsigned int i = 0; i < result.size(); ++i ) {
        double elev = surf.query( result[i].lon(), result[i].lat() );
        result[i].setelev( elev + offset );
    }

    return result;
}

// Determine node elevations of each node of a TGPolygon based on the
// provided TGAptSurface.  Offset is added to the final elevation
static TGPolygon calc_elevations( TGAptSurface &surf,
                                  const TGPolygon& poly,
                                  double offset )
{
    TGPolygon result;
    for ( int i = 0; i < poly.contours(); ++i ) {
        point_list contour = poly.get_contour( i );
        point_list elevated = calc_elevations( surf, contour, offset );

        result.add_contour( elevated, poly.get_hole_flag(i) );
    }

    return result;
}

void Airport::BuildBtg(const string& root, const string_list& elev_src )
{
    TGPolygon apt_base;
    TGPolygon apt_clearing;

    TGPolygon accum;
    accum.erase(); 
    TGPolygon line_accum;
    line_accum.erase();

    // runways
    superpoly_list rwy_polys;
    texparams_list rwy_tps;

    // pavements
    superpoly_list pvmt_polys;
    texparams_list pvmt_tps;

    // linear features
    superpoly_list line_polys;
    texparams_list line_tps;

    int i, j, k;
    char buf[120];  // For debugging output
    Point3D p;

    // parse main airport information
    double apt_lon = 0.0, apt_lat = 0.0;
    int rwy_count = 0;

    // Find the average of all the runway long / lats
    // TODO : Need runway object...    
    for (i=0; i<runways.size(); i++)
    {
        printf("runway %d from (%lf,%lf) to (%lf,%lf) : midpoint is (%lf,%lf)\n", 
            i,
            runways[i]->GetStart().x(),    runways[i]->GetStart().y(),
            runways[i]->GetEnd().x(),      runways[i]->GetEnd().y(),
            runways[i]->GetMidpoint().x(), runways[i]->GetMidpoint().y()
        );

        apt_lon += runways.at(i)->GetMidpoint().x();
        apt_lat += runways.at(i)->GetMidpoint().y();
    }
    apt_lon = apt_lon / (double)runways.size();
    apt_lat = apt_lat / (double)runways.size();

    printf("Average is (%lf,%lf\n", apt_lon, apt_lat);

    SGBucket b( apt_lon, apt_lat );
    SG_LOG(SG_GENERAL, SG_INFO, b.gen_base_path() << "/" << b.gen_index_str());

    superpoly_list rwy_lights; 
    rwy_lights.clear();

    // If we are cutting in the linear features, add them first
    if (pavements.size())
    {
        for ( i=0; i<pavements.size(); i++ )
        {
            AddFeatures( pavements[i]->GetFeatures() );
        }
    }

    // Add the linear features
    if (features.size())
    {
        for ( i=0; i<features.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Build Feature Poly " << i << ": " << features[i]->GetDescription() );

            // cut the linear feature in until we get the geometry right...
            // features[i]->BuildBtg( altitude, &line_polys, &line_tps, &line_accum );
            features[i]->BuildBtg( altitude, &pvmt_polys, &pvmt_tps, &accum, &rwy_lights );
        }
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "no markings");
    }

    // Build runways next
    for (i=0; i<runways.size(); i++ ) 
    {
        if ( runways[i]->IsPrecision() ) 
        {
            if (boundary)
            {
                runways[i]->BuildBtg( altitude, &rwy_polys, &rwy_tps, &rwy_lights, &accum, NULL, NULL );
            }
            else
            {
                runways[i]->BuildBtg( altitude, &rwy_polys, &rwy_tps, &rwy_lights, &accum, &apt_base, &apt_clearing );
            }
	    }
    }

    if (lightobjects.size())
    {
        for ( i=0; i<lightobjects.size(); i++ )
        {
            lightobjects[i]->BuildBtg( altitude, &rwy_lights );
        }
    }

    // Build helipads (use runway poly- and texture list for this)
    if (helipads.size())
    {
        for (i=0; i<helipads.size(); i++ )
        {
            if (boundary)
            {
                helipads[i]->BuildBtg( altitude, &rwy_polys, &rwy_tps, &rwy_lights, &accum, NULL, NULL );
            }
            else
            {
                helipads[i]->BuildBtg( altitude, &rwy_polys, &rwy_tps, &rwy_lights, &accum, &apt_base, &apt_clearing );
            }
        }
    }
    // Build the pavements
    if (pavements.size())
    {
        for ( i=0; i<pavements.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Build Pavement Poly " << i << ": " << pavements[i]->GetDescription());

            if (boundary)
            {
                pavements[i]->BuildBtg( altitude, &pvmt_polys, &pvmt_tps, &accum, NULL, NULL );
            }
            else
            {
                pavements[i]->BuildBtg( altitude, &pvmt_polys, &pvmt_tps, &accum, &apt_base, &apt_clearing );
            }
        }
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "no pavements");
    }

    // build the base and clearing if there's a boundary
    if (boundary)
    {
        boundary->BuildBtg( altitude, &apt_base, &apt_clearing );
    }

    if ( apt_base.total_size() == 0 )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "no airport points generated");
    	return;
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Lookin good");
    }

    TGPolygon filled_base  = tgPolygonStripHoles( apt_base );
    TGPolygon divided_base = tgPolygonSplitLongEdges( filled_base, 200.0 );
    TGPolygon base_poly    = tgPolygonDiff( divided_base, accum );

    // add segments to polygons to remove any possible "T"
    // intersections
    TGTriNodes tmp_nodes;

    // build temporary node list from runways...
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) 
    {
    	TGPolygon poly = rwy_polys[k].get_poly();
    	for ( i = 0; i < poly.contours(); ++i ) 
        {
    	    for ( j = 0; j < poly.contour_size( i ); ++j ) 
            {
        		tmp_nodes.unique_add( poly.get_pt(i, j) );
    	    }
    	}
    }

    // and pavements
    for ( k = 0; k < (int)pvmt_polys.size(); ++k ) 
    {
    	TGPolygon poly = pvmt_polys[k].get_poly();
    	for ( i = 0; i < poly.contours(); ++i ) 
        {
    	    for ( j = 0; j < poly.contour_size( i ); ++j ) 
            {
        		tmp_nodes.unique_add( poly.get_pt(i, j) );
    	    }
    	}
    }

    // and linear features
    for ( k = 0; k < (int)line_polys.size(); ++k ) 
    {
    	TGPolygon poly = line_polys[k].get_poly();
    	for ( i = 0; i < poly.contours(); ++i ) 
        {
    	    for ( j = 0; j < poly.contour_size( i ); ++j ) 
            {
        		tmp_nodes.unique_add( poly.get_pt(i, j) );
    	    }
    	}
    }

    // and the base
    for ( i = 0; i < base_poly.contours(); ++i ) 
    {
    	for ( j = 0; j < base_poly.contour_size( i ); ++j ) 
        {
    	    tmp_nodes.unique_add( base_poly.get_pt(i, j) );
    	}
    }

    // the divided base could contain points not found in base_poly,
    // so we should add them because the skirt needs them.
    for ( i = 0; i < divided_base.contours(); ++i ) 
    {
	    for ( j = 0; j < divided_base.contour_size( i ); ++j ) 
        {
    	    tmp_nodes.unique_add( divided_base.get_pt(i, j) );
    	}
    }

    // second pass : runways
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) 
    {
    	TGPolygon poly = rwy_polys[k].get_poly();
    	poly = add_nodes_to_poly( poly, tmp_nodes );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.total_size());

    	rwy_polys[k].set_poly( poly );
    }

    // second pass : and pavements
    for ( k = 0; k < (int)pvmt_polys.size(); ++k ) 
    {
    	TGPolygon poly = pvmt_polys[k].get_poly();
    	poly = add_nodes_to_poly( poly, tmp_nodes );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.total_size());

    	pvmt_polys[k].set_poly( poly );
    }

    // second pass : and lines
    for ( k = 0; k < (int)line_polys.size(); ++k ) 
    {
    	TGPolygon poly = line_polys[k].get_poly();
    	poly = add_nodes_to_poly( poly, tmp_nodes );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.total_size());

    	line_polys[k].set_poly( poly );
    }


    // One more pass to try to get rid of other yukky stuff
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) 
    {
    	TGPolygon poly = rwy_polys[k].get_poly();

    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size of section " << k << " before =" << poly.total_size());

        poly = remove_dups( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_dups() = " << poly.total_size());
        poly = remove_bad_contours( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_bad() = " << poly.total_size());
        poly = remove_tiny_contours( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_tiny_contours() = " << poly.total_size());

    	rwy_polys[k].set_poly( poly );
    }

    for ( k = 0; k < (int)pvmt_polys.size(); ++k ) 
    {
    	TGPolygon poly = pvmt_polys[k].get_poly();

    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size of section " << k << " before =" << poly.total_size());

        poly = remove_dups( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_dups() = " << poly.total_size());
        poly = remove_bad_contours( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_bad() = " << poly.total_size());
        poly = remove_tiny_contours( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_tiny_contours() = " << poly.total_size());

    	pvmt_polys[k].set_poly( poly );
    }

    for ( k = 0; k < (int)line_polys.size(); ++k ) 
    {
    	TGPolygon poly = line_polys[k].get_poly();

    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size of section " << k << " before =" << poly.total_size());

        poly = remove_dups( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_dups() = " << poly.total_size());
        poly = remove_bad_contours( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_bad() = " << poly.total_size());
        poly = remove_tiny_contours( poly );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_tiny_contours() = " << poly.total_size());

    	line_polys[k].set_poly( poly );
    }


    SG_LOG(SG_GENERAL, SG_DEBUG, "add nodes base ");
    SG_LOG(SG_GENERAL, SG_DEBUG, " before: " << base_poly);
    SG_LOG(SG_GENERAL, SG_DEBUG, " tmp_nodes size = " << tmp_nodes.get_node_list().size());

    base_poly = add_nodes_to_poly( base_poly, tmp_nodes );
    SG_LOG(SG_GENERAL, SG_DEBUG, " after adding tmp_nodes: " << base_poly);

    // write_polygon( base_poly, "base-add" );
    SG_LOG(SG_GENERAL, SG_DEBUG, "remove dups base ");
    base_poly = remove_dups( base_poly );
    SG_LOG(SG_GENERAL, SG_DEBUG, "remove bad contours base");
    base_poly = remove_bad_contours( base_poly );
    SG_LOG(SG_GENERAL, SG_DEBUG, "remove small contours base");
    base_poly = remove_tiny_contours( base_poly );
    // write_polygon( base_poly, "base-fin" );
    SG_LOG(SG_GENERAL, SG_DEBUG, " after clean up: " << base_poly);

    // tesselate the polygons and prepair them for final output
    for ( i = 0; i < (int)rwy_polys.size(); ++i ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating section = " << i << " flag = " << rwy_polys[i].get_flag());

    	TGPolygon poly = rwy_polys[i].get_poly();
	    SG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << poly.contours() << " total points before = " << poly.total_size());
	    TGPolygon tri = polygon_tesselate_alt( poly );
	    SG_LOG(SG_GENERAL, SG_DEBUG, "total size after = " << tri.total_size());

        TGPolygon tc;
        tc = rwy_section_tex_coords( tri, rwy_tps[i], true );

    	rwy_polys[i].set_tris( tri );
	    rwy_polys[i].set_texcoords( tc );
    }

    // tesselate the polygons and prepair them for final output
    for ( i = 0; i < (int)pvmt_polys.size(); ++i ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating section = " << i << " flag = " << pvmt_polys[i].get_flag());

    	TGPolygon poly = pvmt_polys[i].get_poly();
	    SG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << poly.contours() << " total points before = " << poly.total_size());
	    TGPolygon tri = polygon_tesselate_alt( poly );
	    SG_LOG(SG_GENERAL, SG_DEBUG, "total size after = " << tri.total_size());

        TGPolygon tc;
        if (pvmt_polys[i].get_flag() == "lf")
        {
            tc = linear_feature_tex_coords( tri, pvmt_tps[i] );
        }
        else
        {
            tc = rwy_section_tex_coords( tri, pvmt_tps[i], false );
        }

    	pvmt_polys[i].set_tris( tri );
	    pvmt_polys[i].set_texcoords( tc );
    }

    // tesselate the polygons and prepair them for final output
    for ( i = 0; i < (int)line_polys.size(); ++i ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating section = " << i << " flag = " << pvmt_polys[i].get_flag());

    	TGPolygon poly = line_polys[i].get_poly();
	    SG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << poly.contours() << " total points before = " << poly.total_size());
	    TGPolygon tri = polygon_tesselate_alt( poly );
	    SG_LOG(SG_GENERAL, SG_DEBUG, "total size after = " << tri.total_size());

        TGPolygon tc;
        tc = rwy_section_tex_coords( tri, line_tps[i], false );

    	line_polys[i].set_tris( tri );
	    line_polys[i].set_texcoords( tc );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating base");
    TGPolygon base_tris = polygon_tesselate_alt( base_poly );

    //
    // We should now have the runway polygons all generated with their
    // corresponding triangles and texture coordinates, and the
    // surrounding base area.
    //
    // Next we need to create the output lists ... vertices, normals,
    // texture coordinates, and tri-strips
    //

    // traverse the tri list and create ordered node and texture
    // coordinate lists

    TGTriNodes nodes, normals, texcoords;
    nodes.clear();
    normals.clear();
    texcoords.clear();

    group_list pts_v; pts_v.clear();
    group_list pts_n; pts_n.clear();
    string_list pt_materials; pt_materials.clear();

    group_list tris_v; tris_v.clear();
    group_list tris_n; tris_n.clear();
    group_list tris_tc; tris_tc.clear();
    string_list tri_materials; tri_materials.clear();

    group_list strips_v; strips_v.clear();
    group_list strips_n; strips_n.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();

    int index;
    int_list pt_v, tri_v, strip_v;
    int_list pt_n, tri_n, strip_n;
    int_list tri_tc, strip_tc;

    // calculate "the" normal for this airport
    p.setx( base_tris.get_pt(0, 0).x() * SGD_DEGREES_TO_RADIANS );
    p.sety( base_tris.get_pt(0, 0).y() * SGD_DEGREES_TO_RADIANS );
    p.setz( 0 );
    Point3D vnt = sgGeodToCart( p );
    
    // SG_LOG(SG_GENERAL, SG_DEBUG, "geod = " << p);
    // SG_LOG(SG_GENERAL, SG_DEBUG, "cart = " << tmp);

    SGVec3d tmp( vnt.x(), vnt.y(), vnt.z() );
    tmp = normalize(tmp);
    Point3D vn( tmp.x(), tmp.y(), tmp.z() );

    SG_LOG(SG_GENERAL, SG_DEBUG, "found normal for this airport = " << tmp);

    for ( k = 0; k < (int)rwy_polys.size(); ++k ) 
    {
    	SG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
    	// TGPolygon tri_poly = rwy_tris[k];
    	TGPolygon tri_poly = rwy_polys[k].get_tris();
    	TGPolygon tri_txs = rwy_polys[k].get_texcoords();
    	string material = rwy_polys[k].get_material();
    	SG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
    	SG_LOG(SG_GENERAL, SG_DEBUG, "poly size = " << tri_poly.contours());
    	SG_LOG(SG_GENERAL, SG_DEBUG, "texs size = " << tri_txs.contours());
    	for ( i = 0; i < tri_poly.contours(); ++i ) 
        {
    	    tri_v.clear();
    	    tri_n.clear();
    	    tri_tc.clear();
    	    for ( j = 0; j < tri_poly.contour_size(i); ++j ) 
            {
        		p = tri_poly.get_pt( i, j );
        		index = nodes.unique_add( p );
        		tri_v.push_back( index );

        		// use 'the' normal
        		index = normals.unique_add( vn );
        		tri_n.push_back( index );

        		Point3D tc = tri_txs.get_pt( i, j );
        		index = texcoords.unique_add( tc );
        		tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    for ( k = 0; k < (int)pvmt_polys.size(); ++k ) 
    {
    	SG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
    	// TGPolygon tri_poly = rwy_tris[k];
    	TGPolygon tri_poly = pvmt_polys[k].get_tris();
    	TGPolygon tri_txs = pvmt_polys[k].get_texcoords();
    	string material = pvmt_polys[k].get_material();
    	SG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
    	SG_LOG(SG_GENERAL, SG_DEBUG, "poly size = " << tri_poly.contours());
    	SG_LOG(SG_GENERAL, SG_DEBUG, "texs size = " << tri_txs.contours());
    	for ( i = 0; i < tri_poly.contours(); ++i ) 
        {
    	    tri_v.clear();
    	    tri_n.clear();
    	    tri_tc.clear();
    	    for ( j = 0; j < tri_poly.contour_size(i); ++j ) 
            {
        		p = tri_poly.get_pt( i, j );
        		index = nodes.unique_add( p );
        		tri_v.push_back( index );

        		// use 'the' normal
        		index = normals.unique_add( vn );
        		tri_n.push_back( index );

        		Point3D tc = tri_txs.get_pt( i, j );
        		index = texcoords.unique_add( tc );
        		tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    for ( k = 0; k < (int)line_polys.size(); ++k ) 
    {
    	SG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
    	// TGPolygon tri_poly = rwy_tris[k];
    	TGPolygon tri_poly = line_polys[k].get_tris();
    	TGPolygon tri_txs = line_polys[k].get_texcoords();
    	string material = line_polys[k].get_material();
    	SG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
    	SG_LOG(SG_GENERAL, SG_DEBUG, "poly size = " << tri_poly.contours());
    	SG_LOG(SG_GENERAL, SG_DEBUG, "texs size = " << tri_txs.contours());
    	for ( i = 0; i < tri_poly.contours(); ++i ) 
        {
    	    tri_v.clear();
    	    tri_n.clear();
    	    tri_tc.clear();
    	    for ( j = 0; j < tri_poly.contour_size(i); ++j ) 
            {
        		p = tri_poly.get_pt( i, j );
        		index = nodes.unique_add( p );
        		tri_v.push_back( index );

        		// use 'the' normal
        		index = normals.unique_add( vn );
        		tri_n.push_back( index );

        		Point3D tc = tri_txs.get_pt( i, j );
        		index = texcoords.unique_add( tc );
        		tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    // add base points
    std::vector< SGVec2f > base_txs; 
    int_list base_tc;

    for ( i = 0; i < base_tris.contours(); ++i ) 
    {
    	tri_v.clear();
    	tri_n.clear();
    	tri_tc.clear();
    	for ( j = 0; j < base_tris.contour_size(i); ++j ) 
        {
    	    p = base_tris.get_pt( i, j );
    	    index = nodes.unique_add( p );
    	    tri_v.push_back( index );

    	    index = normals.unique_add( vn );
    	    tri_n.push_back( index);
    	}
    	tris_v.push_back( tri_v );
    	tris_n.push_back( tri_n );
    	tri_materials.push_back( "Grass" );

    	std::vector < SGGeod > geodNodes;
    	for ( j = 0; j < nodes.get_node_list().size(); j++ ) 
        {
    	    Point3D node = nodes.get_node_list()[j];
    	    geodNodes.push_back( SGGeod::fromDegM( node.x(), node.y(), node.z() ) );
    	}
	    base_txs.clear();
    	base_txs = sgCalcTexCoords( b, geodNodes, tri_v );

    	base_tc.clear();
    	for ( j = 0; j < (int)base_txs.size(); ++j ) 
        {
    	    SGVec2f tc = base_txs[j];
    	    // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
    	    index = texcoords.simple_add( Point3D( tc.x(), tc.y(), 0 ) );
    	    base_tc.push_back( index );
    	}
    	tris_tc.push_back( base_tc );
    }

    // on rare occasion, one or more of the divided base points can be
    // missed.  Make sure they are all in the node list so we can
    // build a proper skirt.

    for ( i = 0; i < divided_base.contours(); ++i ) 
    {
    	for ( j = 0; j < divided_base.contour_size( i ); ++j ) 
        {
    	    nodes.unique_add( divided_base.get_pt(i, j) );
    	}
    }

    // Now that we have assembled all the airport geometry nodes into
    // a list, calculate an "average" airport elevation based on all
    // the actual airport node points.  This is more useful than
    // calculating an average over the entire airport surface because
    // it avoids biases introduced from the surrounding area if the
    // airport is located in a bowl or on a hill.

    double average = tgAverageElevation( root, elev_src, nodes.get_node_list() );
    // cout << "average airport elevation = " << average << endl;

    // Now build the fitted airport surface ...

    // calculation min/max coordinates of airport area
    SG_LOG(SG_GENERAL, SG_DEBUG, " calculation min/max coordinates of airport area");

    Point3D min_deg(9999.0, 9999.0, 0), max_deg(-9999.0, -9999.0, 0);
    for ( j = 0; j < (int)nodes.get_node_list().size(); ++j ) 
    {
        Point3D p = nodes.get_node_list()[j];
        if ( p.lon() < min_deg.lon() ) 
        {
            min_deg.setlon( p.lon() );
        }
        if ( p.lon() > max_deg.lon() ) 
        {
            max_deg.setlon( p.lon() );
        }
        if ( p.lat() < min_deg.lat() ) 
        {
            min_deg.setlat( p.lat() );
        }
        if ( p.lat() > max_deg.lat() ) 
        {
            max_deg.setlat( p.lat() );
        }
    }

    // extend the min/max coordinates of airport area to cover all
    // lights as well

    SG_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : num rwy lights is " << rwy_lights.size() );

    for ( i = 0; i < (int)rwy_lights.size(); ++i ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : rwy light " << i << "has " << rwy_lights[i].get_poly().get_contour(0).size() << " lights " );

        for ( j = 0; j < (int)rwy_lights[i].get_poly().get_contour(0).size(); ++j )
        {
            Point3D p = rwy_lights[i].get_poly().get_contour(0)[j];
            if ( p.lon() < min_deg.lon() ) 
            {
                min_deg.setlon( p.lon() );
            }
            if ( p.lon() > max_deg.lon() ) 
            {
                max_deg.setlon( p.lon() );
            }
            if ( p.lat() < min_deg.lat() ) 
            {
                min_deg.setlat( p.lat() );
            }
            if ( p.lat() > max_deg.lat() ) 
            {
                max_deg.setlat( p.lat() );
            }
        }
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, " done " );

    // Extend the area a bit so we don't have wierd things on the edges
    double dlon = max_deg.lon() - min_deg.lon();
    double dlat = max_deg.lat() - min_deg.lat();
    min_deg.setlon( min_deg.lon() - 0.01 * dlon );
    max_deg.setlon( max_deg.lon() + 0.01 * dlon );
    min_deg.setlat( min_deg.lat() - 0.01 * dlat );
    max_deg.setlat( max_deg.lat() + 0.01 * dlat );
    SG_LOG(SG_GENERAL, SG_DEBUG, "min = " << min_deg << " max = " << max_deg );

    TGAptSurface apt_surf( root, elev_src, min_deg, max_deg, average );
    SG_LOG(SG_GENERAL, SG_INFO, "Airport surface created");

    // calculate node elevations
    SG_LOG(SG_GENERAL, SG_INFO, "Computing airport node elevations");
    point_list geod_nodes = calc_elevations( apt_surf,
                                             nodes.get_node_list(),
                                             0.0 );
    divided_base = calc_elevations( apt_surf, divided_base, 0.0 );

    SG_LOG(SG_GENERAL, SG_DEBUG, "DIVIDED");
    SG_LOG(SG_GENERAL, SG_DEBUG, divided_base);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with base calc_elevations()");

#if 0 // TODO : along with taxiway sign elevations
    SG_LOG(SG_GENERAL, SG_INFO, "Computing tower node elevations");
    point_list tower_nodes = calc_elevations( apt_surf, towers, 0.0 );
#endif

    // calc windsock elevations:
    SG_LOG(SG_GENERAL, SG_INFO, "Computing windsock node elevations");
    point_list ws_nodes;
    ws_nodes.clear();
    for ( i = 0; i < (int)windsocks.size(); ++i ) 
    {
        p = windsocks[i]->GetLoc();
        ws_nodes.push_back( p );
    }
    point_list windsock_nodes = calc_elevations( apt_surf, ws_nodes, 0.0 );


    SG_LOG(SG_GENERAL, SG_INFO, "Computing beacon node elevations");
    point_list b_nodes;
    b_nodes.clear();
    for ( i = 0; i < (int)beacons.size(); ++i ) 
    {
        p = beacons[i]->GetLoc();
        b_nodes.push_back( p );
    }
    point_list beacon_nodes = calc_elevations( apt_surf, b_nodes, 0.0 );

    // calc taxiway sign elevations:
    SG_LOG(SG_GENERAL, SG_INFO, "Computing taxiway sign node elevations");
    point_list ts_nodes;
    ts_nodes.clear();
    for ( i = 0; i < (int)signs.size(); ++i ) 
    {
        p = signs[i]->GetLoc();
        ts_nodes.push_back( p );
    }
    point_list taxisigns_nodes = calc_elevations( apt_surf, ts_nodes, 0.0 );

    // calc water runway buoys elevations:
    point_list buoy_nodes;
    buoy_nodes.clear();
    for ( i = 0; i < (int)waterrunways.size(); ++i )
    {
        TGPolygon tmp_nodes;
        tmp_nodes.erase();
        tmp_nodes = waterrunways[i]->GetNodes();
        for (j=0; j< tmp_nodes.contour_size( 0 ); ++j )
        {
            buoy_nodes.push_back( tmp_nodes.get_pt( 0, j ) );
        }
    }
    point_list water_buoys_nodes = calc_elevations( apt_surf, buoy_nodes, 0.0 );


    // add base skirt (to hide potential cracks)
    //
    // this has to happen after we've calculated the node elevations
    // but before we convert to wgs84 coordinates

    int uindex, lindex;

    for ( i = 0; i < divided_base.contours(); ++i ) 
    {
	    strip_v.clear();
	    strip_n.clear();
	    strip_tc.clear();

    	// prime the pump ...
	    p = divided_base.get_pt( i, 0 );
	    uindex = nodes.find( p );
	    if ( uindex >= 0 ) 
        {
    	    Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
    	    SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
    	    lindex = nodes.simple_add( lower );
    	    geod_nodes.push_back( lower );
    	    strip_v.push_back( lindex );
    	    strip_v.push_back( uindex );

    	    // use 'the' normal.  We are pushing on two nodes so we
    	    // need to push on two normals.
    	    index = normals.unique_add( vn );
    	    strip_n.push_back( index );
    	    strip_n.push_back( index );
     	} 
        else 
        {
            string message = "Ooops missing node when building skirt (in init)";
            SG_LOG( SG_GENERAL, SG_INFO, message << " " << p );
    	    throw sg_exception( message );
    	}

    	// loop through the list
    	for ( j = 1; j < divided_base.contour_size(i); ++j ) 
        {
    	    p = divided_base.get_pt( i, j );
    	    uindex = nodes.find( p );
    	    if ( uindex >= 0 ) 
            {
        		Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
        		SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
        		lindex = nodes.simple_add( lower );
        		geod_nodes.push_back( lower );
        		strip_v.push_back( lindex );
        		strip_v.push_back( uindex );

        		index = normals.unique_add( vn );
        		strip_n.push_back( index );
        		strip_n.push_back( index );
    	    } 
            else 
            {
                string message = "Ooops missing node when building skirt (in loop)";
                SG_LOG( SG_GENERAL, SG_INFO, message << " " << p );
                throw sg_exception( message );
	        }
	    }

    	// close off the loop
	    p = divided_base.get_pt( i, 0 );
	    uindex = nodes.find( p );
	    if ( uindex >= 0 ) 
        {
    	    Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
    	    SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
    	    lindex = nodes.simple_add( lower );
    	    geod_nodes.push_back( lower );
    	    strip_v.push_back( lindex );
    	    strip_v.push_back( uindex );

    	    index = normals.unique_add( vn );
    	    strip_n.push_back( index );
    	    strip_n.push_back( index );
    	} 
        else 
        {
            string message = "Ooops missing node when building skirt (at end)";
            SG_LOG( SG_GENERAL, SG_INFO, message << " " << p );
            throw sg_exception( message );
    	}

    	strips_v.push_back( strip_v );
    	strips_n.push_back( strip_n );
    	strip_materials.push_back( "Grass" );

    	std::vector < SGGeod > geodNodes;
    	for ( j = 0; j < nodes.get_node_list().size(); j++ ) 
        {
    	    Point3D node = nodes.get_node_list()[j];
    	    geodNodes.push_back( SGGeod::fromDegM( node.x(), node.y(), node.z() ) );
    	}
	    base_txs.clear();
	    base_txs = sgCalcTexCoords( b, geodNodes, strip_v );

	    base_tc.clear();
	    for ( j = 0; j < (int)base_txs.size(); ++j ) 
        {
    	    SGVec2f tc = base_txs[j];
    	    // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
    	    index = texcoords.simple_add( Point3D( tc.x(), tc.y(), 0 ) );
    	    base_tc.push_back( index );
    	}
    	strips_tc.push_back( base_tc );
    }

    // add light points
    superpoly_list tmp_light_list; 
    tmp_light_list.clear();
    typedef map < string, double, less<string> > elev_map_type;
    typedef elev_map_type::const_iterator const_elev_map_iterator;
    elev_map_type elevation_map;

    SG_LOG(SG_GENERAL, SG_INFO, "Computing runway/approach lighting elevations");

    // pass one, calculate raw elevations from Array
    for ( i = 0; i < (int)rwy_lights.size(); ++i ) 
    {
        TGTriNodes light_nodes;
        light_nodes.clear();
        point_list lights_v = rwy_lights[i].get_poly().get_contour(0);
        for ( j = 0; j < (int)lights_v.size(); ++j ) 
        {
            p = lights_v[j];
            index = light_nodes.simple_add( p );
        }

        // calculate light node elevations
        point_list geod_light_nodes = calc_elevations( apt_surf, light_nodes.get_node_list(), 0.0 );
        TGPolygon p;
        p.add_contour( geod_light_nodes, 0 );
        TGSuperPoly s;
        s.set_poly( p );
        tmp_light_list.push_back( s );

        string flag = rwy_lights[i].get_flag();
        if ( flag != (string)"" ) 
        {
            double max = -9999;
            const_elev_map_iterator it = elevation_map.find( flag );
            if ( it != elevation_map.end() ) 
            {
                max = elevation_map[flag];
            }
            for ( j = 0; j < (int)geod_light_nodes.size(); ++j ) 
            {
                if ( geod_light_nodes[j].z() > max ) 
                {
                    max = geod_light_nodes[j].z();
                }
            }
            elevation_map[flag] = max;
            SG_LOG( SG_GENERAL, SG_DEBUG, flag << " max = " << max );
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Done with lighting calc_elevations() num light polys is " << rwy_lights.size() );

    // pass two, for each light group check if we need to lift (based
    // on flag) and do so, then output next structures.
    for ( i = 0; i < (int)rwy_lights.size(); ++i ) 
    {
        // tmp_light_list is a parallel structure to rwy_lights
        point_list geod_light_nodes = tmp_light_list[i].get_poly().get_contour(0);
        
        SG_LOG(SG_GENERAL, SG_INFO, "got a point list with " <<  geod_light_nodes.size() << " points" );

        // this is a little round about, but what we want to calculate the
        // light node elevations as ground + an offset so we do them
        // seperately, then we add them back into nodes to get the index
        // out, but also add them to geod_nodes to maintain consistancy
        // between these two lists.
        point_list light_normals = rwy_lights[i].get_normals().get_contour(0);
        pt_v.clear();
        pt_n.clear();
        for ( j = 0; j < (int)geod_light_nodes.size(); ++j ) 
        {
            p = geod_light_nodes[j];
            index = nodes.simple_add( p );
            pt_v.push_back( index );
            geod_nodes.push_back( p );

            index = normals.unique_add( light_normals[j] );
            pt_n.push_back( index );
        }
        pts_v.push_back( pt_v );
        pts_n.push_back( pt_n );
        pt_materials.push_back( rwy_lights[i].get_material() );
    }

    // calculate wgs84 mapping of nodes
    std::vector< SGVec3d > wgs84_nodes;
    for ( i = 0; i < (int)geod_nodes.size(); ++i ) 
    {
        SGGeod geod = SGGeod::fromDegM( geod_nodes[i].x(), geod_nodes[i].y(), geod_nodes[i].z() );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "geod pt = " << geod_nodes[i] );
        SGVec3d cart = SGVec3d::fromGeod(geod);
        SG_LOG(SG_GENERAL, SG_DEBUG, "  cart pt = " << cart );
    	wgs84_nodes.push_back( cart );
    }
    SGSphered d;
    for ( i = 0; i < wgs84_nodes.size(); ++i ) 
    {
        d.expandBy(wgs84_nodes[ i ]);
    }
    
    SGVec3d gbs_center = d.getCenter();
    double gbs_radius = d.getRadius();
    SG_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
    SG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_n; fans_n.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    string objpath = root + "/AirportObj";
    string name = icao + ".btg";
    
    std::vector< SGVec3f > normals_3f;
    for ( i=0; i < normals.get_node_list().size(); i++ ) 
    {
        Point3D node = normals.get_node_list()[i];
        normals_3f.push_back( node.toSGVec3f() );
    }

    std::vector< SGVec2f > texcoords_2f;
    for ( i=0; i < texcoords.get_node_list().size(); i++ ) 
    {
        Point3D node = texcoords.get_node_list()[i];
        texcoords_2f.push_back( node.toSGVec2f() );
    }

    SGBinObject obj;

    SG_LOG(SG_GENERAL, SG_ALERT, "number of NODES is  " << wgs84_nodes.size() );

    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals_3f );
    obj.set_texcoords( texcoords_2f );
    obj.set_pts_v( pts_v );
    obj.set_pts_n( pts_n );
    obj.set_pt_materials( pt_materials );
    obj.set_tris_v( tris_v );
    obj.set_tris_n( tris_n );
    obj.set_tris_tc( tris_tc ); 
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_n( strips_n );
    obj.set_strips_tc( strips_tc ); 
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_n( fans_n );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );

    bool result;
    result = obj.write_bin( objpath, name, b );
    if ( !result ) 
    {
        throw sg_exception("error writing file. :-(");
    }

    // write out airport object reference
    write_index( objpath, b, name );

#if 0 // TODO : along with taxiway signs
    // write out tower references
    for ( i = 0; i < (int)tower_nodes.size(); ++i ) 
    {
        write_index_shared( objpath, b, tower_nodes[i],
                            "Models/Airport/tower.xml",
                            0.0 );
    }
#endif

    // write out windsock references : TODO - save elevation data in the windsock object
    for ( i = 0; i < (int)windsock_nodes.size(); ++i ) 
    {
    	if ( windsocks[i]->IsLit() ) 
        {
            write_index_shared( objpath, b, windsock_nodes[i],
                                "Models/Airport/windsock_lit.xml",
                                0.0 );
        }
        else
        {
            write_index_shared( objpath, b, windsock_nodes[i],
                                "Models/Airport/windsock.xml",
                                0.0 );
    	} 
    }

    // write out beacon references
    for ( i = 0; i < (int)beacon_nodes.size(); ++i ) 
    {
        write_index_shared( objpath, b, beacon_nodes[i],
                            "Models/Airport/beacon.xml",
                            0.0 );
    }

    // write out taxiway signs references
    for ( i = 0; i < (int)taxisigns_nodes.size(); ++i ) 
    {
        write_object_sign( objpath, b, taxisigns_nodes[i],
                            signs[i]->GetDefinition(),
                            0.0 );
    }

    // write out water buoys
    for ( i = 0; i < (int)water_buoys_nodes.size(); ++i )
    {
    write_index_shared( objpath, b, water_buoys_nodes[i],
                            "Models/Airport/water_rw_buoy.xml",
                            0.0 );
    }


    string holepath = root + "/AirportArea";
    // long int poly_index = poly_index_next();
    // write_boundary( holepath, b, hull, poly_index );
    tgChopNormalPolygon( holepath, "Hole", divided_base, true );
    tgChopNormalPolygon( holepath, "Airport", apt_clearing, false );
}
