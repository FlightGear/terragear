// build.cxx -- routines to build polygon model of an airport from the runway
//              definition
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
//


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <simgear/compiler.h>

#include <stdio.h>
#include <stdlib.h>		// for atoi() atof()
#include <time.h>

#include <list>
#include STL_STRING

#include <plib/sg.h>			// plib include

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/fg_geodesy.hxx>
#include <simgear/misc/texcoord.hxx>

#include <Array/array.hxx>
#include <Geometry/poly_support.hxx>
#include <Geometry/trinodes.hxx>
#include <Polygon/index.hxx>
#include <Polygon/polygon.hxx>
#include <Triangulate/trieles.hxx>

#include "convex_hull.hxx"
#include "output.hxx"
#include "point2d.hxx"
#include "runway.hxx"
#include "scenery_version.hxx"


static const double tgAirportEpsilon = FG_EPSILON / 10.0;


// calculate texture coordinates for a 1/2 runway.  Returns a mirror
// polygon to the runway, except each point is the texture coordinate
// of the corresponding point in the original polygon.
static FGPolygon rwy_calc_tex_coords( const FGRunway& rwy,
				      double hdg_offset,
				      const FGPolygon& in_poly )
{
    FGPolygon result;
    result.erase();
    double length = rwy.length * FEET_TO_METER;
    double width = rwy.width * FEET_TO_METER;

    Point3D center( rwy.lon, rwy.lat, 0 );
    cout << "runway heading = " << rwy.heading << endl;
    double angle = rwy.heading + hdg_offset;
    Point3D p, tp;
    double x, y, tx, ty;

    for ( int i = 0; i < in_poly.contours(); ++i ) {
	for ( int j = 0; j < in_poly.contour_size( i ); ++j ) {
	    p = in_poly.get_pt( i, j );

	    // dist = gc_dist( center, p );
	    // course = gc_course( center, p, dist ) + angle;

	    // given alt, lat1, lon1, lat2, lon2, calculate starting
	    // and ending az1, az2 and distance (s).  Lat, lon, and
	    // azimuth are in degrees.  distance in meters
	    double az1, az2, dist;
	    geo_inverse_wgs_84( 0, center.y(), center.x(), p.y(), p.x(),
				&az1, &az2, &dist );

	    // cout << "basic course = " << az1 << endl;
	    double course = az1 - angle;
	    // cout << "course = " << course << endl;
	    while ( course < -360 ) { course += 360; }
	    while ( course > 360 ) { course -= 360; }
	    // cout << "Dist = " << dist << endl;
	    // cout << "  Course = " << course * 180.0 / FG_PI << endl;

	    x = cos( course * DEG_TO_RAD ) * dist;
	    y = sin( course * DEG_TO_RAD ) * dist;
	    // cout << "  x = " << x << " y = " << y << endl;

	    tx = x / (length / 2.0);
	    tx = ((int)(tx * 100)) / 100.0;
	    if ( tx < -1.0 ) { tx = -1.0; }
	    if ( tx > 1.0 ) { tx = 1.0; }

	    ty = (y + (width / 2.0)) / width;
	    ty = ((int)(ty * 100)) / 100.0;
	    if ( ty < -1.0 ) { ty = -1.0; }
	    if ( ty > 1.0 ) { ty = 1.0; }

	    tp = Point3D( tx, ty, 0 );
	    // cout << "  (" << tx << ", " << ty << ")" << endl;

	    result.add_node( i, tp );
	}
    }

    return result;
}


// Divide segment if there are other existing points on it, return the
// new polygon
void add_intermediate_nodes( int contour, const Point3D& start, 
			     const Point3D& end, const FGTriNodes& tmp_nodes,
			     FGPolygon *result )
{
    bool found_extra = false;
    int extra_index = 0;
    int counter;
    double m, m1, b, b1, y_err, x_err, y_err_min, x_err_min;
    const_point_list_iterator current, last;
    point_list nodes = tmp_nodes.get_node_list();

    Point3D p0 = start;
    Point3D p1 = end;

    cout << "  add_intermediate_nodes()" << endl;
    printf("   %.7f %.7f %.7f <=> %.7f %.7f %.7f\n",
	   p0.x(), p0.y(), p0.z(), p1.x(), p1.y(), p1.z() );

    double xdist = fabs(p0.x() - p1.x());
    double ydist = fabs(p0.y() - p1.y());
    cout << "xdist = " << xdist << "  ydist = " << ydist << endl;
    x_err_min = xdist + 1.0;
    y_err_min = ydist + 1.0;

    if ( xdist > ydist ) {
	cout << "use y = mx + b" << endl;

	// sort these in a sensible order
	Point3D p_min, p_max;
	if ( p0.x() < p1.x() ) {
	    p_min = p0;
	    p_max = p1;
	} else {
	    p_min = p1;
	    p_max = p0;
	}

	m = (p_min.y() - p_max.y()) / (p_min.x() - p_max.x());
	b = p_max.y() - m * p_max.x();

	// if ( temp ) {
	cout << "m = " << m << " b = " << b << endl;
	// }

	current = nodes.begin();
	last = nodes.end();
	counter = 0;
	for ( ; current != last; ++current ) {
	    cout << counter << endl;

	    if ( (current->x() > (p_min.x() + FG_EPSILON)) 
		 && (current->x() < (p_max.x() - FG_EPSILON)) ) {

		printf("found a potential candidate %.7f %.7f %.7f\n",
		       current->x(), current->y(), current->z() );

		y_err = fabs(current->y() - (m * current->x() + b));
		cout << "y_err = " << y_err << endl;

		if ( y_err < tgAirportEpsilon ) {
		    cout << "FOUND EXTRA SEGMENT NODE (Y)" << endl;
		    cout << p_min << " < " << *current << " < "
		         << p_max << endl;
		    found_extra = true;
		    if ( y_err < y_err_min ) {
			extra_index = counter;
			y_err_min = y_err;
		    }
		}
	    }
	    ++counter;
	}
    } else {
	cout << "use x = m1 * y + b1" << endl;

	// sort these in a sensible order
	Point3D p_min, p_max;
	if ( p0.y() < p1.y() ) {
	    p_min = p0;
	    p_min = p1;
	} else {
	    p_min = p1;
	    p_max = p0;
	}

	m1 = (p_min.x() - p_max.x()) / (p_min.y() - p_max.y());
	b1 = p_max.x() - m1 * p_max.y();

	// bool temp = true;
	// if ( temp ) {
	cout << "  m1 = " << m1 << " b1 = " << b1 << endl;
	// }

	// cout << "  should = 0 = " << fabs(p_min.x() - (m1 * p_min.y() + b1)) << endl;;
	// cout << "  should = 0 = " << fabs(p_max.x() - (m1 * p_max.y() + b1)) << endl;;

	current = nodes.begin();
	last = nodes.end();
	counter = 0;
	for ( ; current != last; ++current ) {
	    if ( (current->y() > (p_min.y() + FG_EPSILON)) 
		 && (current->y() < (p_max.y() - FG_EPSILON)) ) {
		
		printf("found a potential candidate %.7f %.7f %.7f\n",
		       current->x(), current->y(), current->z() );

		x_err = fabs(current->x() - (m1 * current->y() + b1));
		cout << "x_err = " << x_err << endl;

		// if ( temp ) {
		cout << "  (" << counter << ") x_err = " << x_err << endl;
		// }

		if ( x_err < tgAirportEpsilon ) {
		    cout << "FOUND EXTRA SEGMENT NODE (X)" << endl;
		    cout << p_min << " < " << *current << " < "
		         << p_max << endl;
		    found_extra = true;
		    if ( x_err < x_err_min ) {
			extra_index = counter;
			x_err_min = x_err;
		    }
		}
	    }
	    ++counter;
	}
    }

    if ( found_extra ) {
	// recurse with two sub segments
	cout << "dividing " << p0 << " " << nodes[extra_index]
	     << " " << p1 << endl;
	add_intermediate_nodes( contour, p0, nodes[extra_index], tmp_nodes, 
				result );

	result->add_node( contour, nodes[extra_index] );

	add_intermediate_nodes( contour, nodes[extra_index], p1, tmp_nodes,
				result );
    } else {
	// this segment does not need to be divided
    }
}


// Search each segment for additional vertex points that may have been
// created elsewhere that lie on the segment and split it there to
// avoid "T" intersections.

static FGPolygon add_nodes_to_poly( const FGPolygon& poly, 
				    const FGTriNodes& tmp_nodes ) {
    FGPolygon result;
    Point3D p0, p1;

    // cout << "add_nodes_to_poly" << endl;

    for ( int i = 0; i < poly.contours(); ++i ) {
	// cout << "contour = " << i << endl;
	for ( int j = 0; j < poly.contour_size(i) - 1; ++j ) {
	    p0 = poly.get_pt( i, j );
	    p1 = poly.get_pt( i, j + 1 );

	    // add start of segment
	    result.add_node( i, p0 );

	    // add intermediate points
	    add_intermediate_nodes( i, p0, p1, tmp_nodes, &result );

	    // end of segment is beginning of next segment
	}
	p0 = poly.get_pt( i, poly.contour_size(i) - 1 );
	p1 = poly.get_pt( i, 0 );

	// add start of segment
	result.add_node( i, p0 );

	// add intermediate points
	add_intermediate_nodes( i, p0, p1, tmp_nodes, &result );

	// end of segment is beginning of next segment
	// 5/9/2000 CLO - this results in duplicating the last point
	// of a contour so I have removed this line.
	// result.add_node( i, p1 );

	// maintain original hole flag setting
	result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    return result;
}


// remove any duplicate nodes
static FGPolygon remove_dups( const FGPolygon &poly ) {
    FGPolygon result;
    result.erase();

    for ( int i = 0; i < poly.contours(); ++i ) {
        Point3D last = poly.get_pt( i, poly.contour_size(i) - 1 );
	bool all_same = true;
	for ( int j = 0; j < poly.contour_size(i); ++j ) {
	    // cout << "  " << i << " " << j << endl;
	    Point3D cur = poly.get_pt( i, j );
	    if ( cur == last ) {
		// skip
		// cout << "skipping a duplicate point" << endl;
	    } else {
		result.add_node( i, cur );
		all_same = false;
		last = cur;
	    }
        }

	// make sure the last point doesn't equal the previous or the first.
        Point3D begin = poly.get_pt( i, 0 );
        Point3D end = poly.get_pt( i, poly.contour_size(i) - 1 );
	if ( begin == end ) {
	    // skip
	    cout << "begin == end!" << endl;
	    // exit(-1);
	}

	if ( !all_same ) {
	    int flag = poly.get_hole_flag( i );
	    result.set_hole_flag( i, flag );
	} else {
	    // too small an area ... add a token point to the contour
	    // to keep other things happy
	    result.add_node( i, begin );
	}
    }

    return result;
}


// remove any degenerate contours
static FGPolygon remove_bad_contours( const FGPolygon &poly ) {
    FGPolygon result;
    result.erase();

    for ( int i = 0; i < poly.contours(); ++i ) {
	point_list contour = poly.get_contour( i );
	if ( contour.size() >= 3 ) {
	    // good
	    int flag = poly.get_hole_flag( i );
	    result.add_contour( contour, flag );
	}
    }

    return result;
}


// fix node elevations
point_list calc_elevations( const string& root, const point_list& geod_nodes ) {
    bool done = false;
    point_list result = geod_nodes;
    int i;
    FGArray array;

    // set all elevations to -9999
    for ( i = 0; i < (int)result.size(); ++i ) {
	result[i].setz( -9999.0 );
    }

    while ( !done ) {
	// find first node with -9999 elevation
	i = 0;
	while ( (result[i].z() > -9000) && (i < (int)result.size()) ) {
	    ++i;
	}

	if ( i < (int)result.size() ) {
	    FGBucket b( result[i].x(), result[i].y() );
	    string base = b.gen_base_path();

	    // try 3 arcsec dems first
	    string dem_path = root + "/DEM-3/" + base 
		+ "/" + b.gen_index_str() + ".dem";
	    cout << "dem_path = " << dem_path << endl;
	
	    if ( ! array.open(dem_path) ) {
		cout << "ERROR: cannot open 3 arcsec file " << dem_path << endl;
		cout << "trying 30 arcsec file" << endl;
		
		// try 30 arcsec dem
		dem_path = root + "/DEM-30/" + base 
		    + "/" + b.gen_index_str() + ".dem";
		cout << "dem_path = " << dem_path << endl;
		if ( ! array.open(dem_path) ) {
		    cout << "ERROR: cannot open 3 arcsec file " 
			 << dem_path << endl;
		}
	    }
	    array.parse( b );

	    // update all the non-updated elevations that are inside
	    // this dem file
	    double elev;
	    done = true;
	    for ( int j = 0; j < (int)result.size(); ++j ) {
		if ( result[j].z() < -9000 ) {
		    done = false;
		    cout << "interpolating for " << result[j] << endl;
		    elev = array.interpolate_altitude( result[j].x() * 3600.0,
						   result[j].y() * 3600.0 );
		    if ( elev > -9000 ) {
			result[j].setz( elev );
		    }
		}
	    }
	    array.close();
	} else {
	    done = true;
	}
    }


    return result;
}


// strip trailing spaces
static void my_chomp( string& str ) {
    cout << "my_chomp()" << endl;
    cout << "'" << str.substr( str.length() - 1, 1 ) << "'" << endl;
    while ( str.substr( str.length() - 1, 1 ) == " " ) {
	str = str.substr( 0, str.length() - 1 );
	cout << "'" << str.substr( str.length() - 1, 1 ) << "'" << endl;
    }
}


// build 3d airport
void build_airport( string airport_raw, string_list& runways_raw,
		    const string& root ) {

    poly_list rwy_polys, rwy_tris, rwy_txs;
    FGPolygon runway, runway_a, runway_b, result, result_a, result_b;
    FGPolygon base;
    point_list apt_pts;
    Point3D p;

    FGPolygon accum;
    accum.erase();

    // parse main airport information
    double apt_lon, apt_lat;
    int elev;

    cout << airport_raw << endl;
    string apt_type = airport_raw.substr(0, 1);
    string apt_code = airport_raw.substr(2, 4); my_chomp( apt_code );
    string apt_lat_str = airport_raw.substr(7, 10);
    apt_lat = atof( apt_lat_str.c_str() );
    string apt_lon_str = airport_raw.substr(18, 11);
    apt_lon = atof( apt_lon_str.c_str() );
    string apt_elev = airport_raw.substr(30, 5);
    elev = atoi( apt_elev.c_str() );
    string apt_use = airport_raw.substr(36, 1);
    string apt_twr = airport_raw.substr(37, 1);
    string apt_bldg = airport_raw.substr(38, 1);
    string apt_name = airport_raw.substr(40);

    /*
    cout << "  type = " << apt_type << endl;
    cout << "  code = " << apt_code << endl;
    cout << "  lat  = " << apt_lat << endl;
    cout << "  lon  = " << apt_lon << endl;
    cout << "  elev = " << apt_elev << " " << elev << endl;
    cout << "  use  = " << apt_use << endl;
    cout << "  twr  = " << apt_twr << endl;
    cout << "  bldg = " << apt_bldg << endl;
    cout << "  name = " << apt_name << endl;
    */

    FGBucket b( apt_lon, apt_lat );
    Point3D center_geod( b.get_center_lon() * DEG_TO_RAD,
			 b.get_center_lat() * DEG_TO_RAD, 0 );
    Point3D gbs_center = fgGeodToCart( center_geod );
    cout << b.gen_base_path() << "/" << b.gen_index_str() << endl;

    // Ignore any seaplane bases
    if ( apt_type == "S" ) {
	return;
    }

    // parse runways and generate the vertex list
    runway_list runways;
    runways.clear();
    string rwy_str;

    for (int i = 0; i < (int)runways_raw.size(); ++i ) {
	rwy_str = runways_raw[i];

	FGRunway rwy;

	cout << rwy_str << endl;
	rwy.rwy_no = rwy_str.substr(2, 4);

	string rwy_lat = rwy_str.substr(6, 10);
	rwy.lat = atof( rwy_lat.c_str() );

	string rwy_lon = rwy_str.substr(17, 11);
	rwy.lon = atof( rwy_lon.c_str() );

	string rwy_hdg = rwy_str.substr(29, 7);
	rwy.heading = atof( rwy_hdg.c_str() );

	string rwy_len = rwy_str.substr(36, 7);
	rwy.length = atoi( rwy_len.c_str() );

	string rwy_width = rwy_str.substr(43, 4);
	rwy.width = atoi( rwy_width.c_str() );

	rwy.surface_flags = rwy_str.substr(47, 4);
	rwy.end1_flags = rwy_str.substr(52, 8);
	rwy.end2_flags = rwy_str.substr(61, 8);

	/*
	cout << "  no    = " << rwy_no << endl;
	cout << "  lat   = " << rwy_lat << " " << lat << endl;
	cout << "  lon   = " << rwy_lon << " " << lon << endl;
	cout << "  hdg   = " << rwy_hdg << " " << hdg << endl;
	cout << "  len   = " << rwy_len << " " << len << endl;
	cout << "  width = " << rwy_width << " " << width << endl;
	cout << "  sfc   = " << rwy_sfc << endl;
	cout << "  end1  = " << rwy_end1 << endl;
	cout << "  end2  = " << rwy_end2 << endl;
	*/

	runways.push_back( rwy );
    }

    for ( int i = 0; i < (int)runways.size(); ++i ) {
	runway = gen_runway_w_mid( runways[i] );

	// runway half "a"
	runway_a.erase();
	runway_a.add_node( 0, runway.get_pt(0, 0) );
	runway_a.add_node( 0, runway.get_pt(0, 1) );
	runway_a.add_node( 0, runway.get_pt(0, 2) );
	runway_a.add_node( 0, runway.get_pt(0, 5) );

	// runway half "b"
	runway_b.erase();
	runway_b.add_node( 0, runway.get_pt(0, 5) );
	runway_b.add_node( 0, runway.get_pt(0, 2) );
	runway_b.add_node( 0, runway.get_pt(0, 3) );
	runway_b.add_node( 0, runway.get_pt(0, 4) );
	
	cout << "raw runway pts (a half)" << endl;
	for ( int j = 0; j < runway_a.contour_size( 0 ); ++j ) {
	    p = runway_a.get_pt(0, j);
	    cout << " point = " << p << endl;
	}
	cout << "raw runway pts (b half)" << endl;
	for ( int j = 0; j < runway_b.contour_size( 0 ); ++j ) {
	    p = runway_b.get_pt(0, j);
	    cout << " point = " << p << endl;
	}
	
	result_a = polygon_diff( runway_a, accum );
	rwy_polys.push_back( result_a );
	cout << "result_a = " << result_a.contours() << endl;
	accum = polygon_union( runway_a, accum );

	result_b = polygon_diff( runway_b, accum );
	rwy_polys.push_back( result_b );
	cout << "result_b = " << result_b.contours() << endl;
	accum = polygon_union( runway_b, accum );

	// after clip, but before removing T intersections
	char tmpa[256], tmpb[256];
	sprintf( tmpa, "a%d", i );
	sprintf( tmpb, "b%d", i );
	write_polygon( result_a, tmpa );
	write_polygon( result_b, tmpb );

	// print runway points
	cout << "clipped runway pts (a)" << endl;
	for ( int j = 0; j < result_a.contours(); ++j ) {
	    for ( int k = 0; k < result_a.contour_size( j ); ++k ) {
		p = result_a.get_pt(j, k);
		cout << " point = " << p << endl;
	    }
	}

	// print runway points
	cout << "clipped runway pts (b)" << endl;
	for ( int j = 0; j < result_b.contours(); ++j ) {
	    for ( int k = 0; k < result_b.contour_size( j ); ++k ) {
		p = result_b.get_pt(j, k);
		cout << " point = " << p << endl;
	    }
	}

	base = gen_runway_area( runways[i], 1.05, 1.5 );

	// add base to apt_pts (for convex hull of airport area)
	for ( int j = 0; j < base.contour_size( 0 ); ++j ) {
	    p = base.get_pt(0, j);
	    // cout << " point = " << p << endl;
	    apt_pts.push_back( p );
	}
    }

    if ( apt_pts.size() == 0 ) {
	cout << "no airport points generated" << endl;
	return;
    }

    // generate convex hull
    FGPolygon hull = convex_hull(apt_pts);
    FGPolygon base_poly = polygon_diff( hull, accum );
    write_polygon( base_poly, "base-raw" );

    // add segments to polygons to remove any possible "T"
    // intersections
    FGTriNodes tmp_nodes;

    // build temporary node list
    for ( int k = 0; k < (int)rwy_polys.size(); ++k ) {
	for ( int i = 0; i < rwy_polys[k].contours(); ++i ) {
	    for ( int j = 0; j < rwy_polys[k].contour_size( i ); ++j ) {
		tmp_nodes.unique_add( rwy_polys[k].get_pt(i, j) );
	    }
	}
    }
    for ( int i = 0; i < base_poly.contours(); ++i ) {
	for ( int j = 0; j < base_poly.contour_size( i ); ++j ) {
	    tmp_nodes.unique_add( base_poly.get_pt(i, j) );
	}
    }

#if 0
    // dump info for debugging purposes
    point_list ttt = tmp_nodes.get_node_list();
    for ( int i = 0; i < (int)ttt.size(); ++i ) {
	char name[256];
	sprintf(name, "p%d", i );
	FILE *fp = fopen( name, "w" );
	fprintf(fp, "%.8f %.8f\n", ttt[i].x(), ttt[i].y());
	fclose(fp);
    }

    for ( int i = 0; i < base_poly.contours(); ++i ) {
	char name[256];
	sprintf(name, "l%d", i );
	FILE *fp = fopen( name, "w" );

	for ( int j = 0; j < base_poly.contour_size( i ) - 1; ++j ) {
	    Point3D p0 = base_poly.get_pt(i, j);
	    Point3D p1 = base_poly.get_pt(i, j + 1);
	    fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	    fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	}
	Point3D p0 = base_poly.get_pt(i, base_poly.contour_size( i ) - 1);
	Point3D p1 = base_poly.get_pt(i, 0);
	fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	fclose(fp);
    }
#endif

    for ( int k = 0; k < (int)rwy_polys.size(); ++k ) {
	cout << "add nodes/remove dups runway = " << k << endl;
	rwy_polys[k] = add_nodes_to_poly( rwy_polys[k], tmp_nodes );

	char tmp[256];
	sprintf( tmp, "r%d", k );
	write_polygon( rwy_polys[k], tmp );

        rwy_polys[k] = remove_dups( rwy_polys[k] );
        rwy_polys[k] = remove_bad_contours( rwy_polys[k] );
    }
    cout << "add nodes/remove dups base " << endl;
    base_poly = add_nodes_to_poly( base_poly, tmp_nodes );
    write_polygon( base_poly, "base-add" );
    base_poly = remove_dups( base_poly );
    base_poly = remove_bad_contours( base_poly );
    write_polygon( base_poly, "base-final" );

    // tesselate the polygons and prepair them for final output

    for ( int i = 0; i < (int)runways.size(); ++i ) {
        cout << "Tesselating runway = " << i << endl;
	FGPolygon tri_a, tri_b, tc_a, tc_b;

	tri_a = polygon_tesselate_alt( rwy_polys[2 * i] );
	tc_a = rwy_calc_tex_coords( runways[i], 0.0, tri_a );
	rwy_tris.push_back( tri_a );
	rwy_txs.push_back( tc_a );

	tri_b = polygon_tesselate_alt( rwy_polys[2 * i + 1] );
	tc_b = rwy_calc_tex_coords( runways[i], 180.0, tri_b );
	rwy_tris.push_back( tri_b );
	rwy_txs.push_back( tc_b );
    }

    cout << "Tesselating base" << endl;
    FGPolygon base_tris = polygon_tesselate_alt( base_poly );

#if 0
    // dump more debugging output
    for ( int i = 0; i < base_strips.contours(); ++i ) {
	char name[256];
	sprintf(name, "s%d", i );
	FILE *fp = fopen( name, "w" );

	for ( int j = 0; j < base_strips.contour_size( i ) - 1; ++j ) {
	    Point3D p0 = base_strips.get_pt(i, j);
	    Point3D p1 = base_strips.get_pt(i, j + 1);
	    fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	    fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	}
	Point3D p0 = base_strips.get_pt(i, base_strips.contour_size( i ) - 1);
	Point3D p1 = base_strips.get_pt(i, 0);
	fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	fclose(fp);
    }
#endif

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

    FGTriNodes nodes, texcoords;
    nodes.clear();
    texcoords.clear();

    group_list tris_v; tris_v.clear();
    group_list tris_tc; tris_tc.clear();
    string_list tri_materials; tri_materials.clear();

    Point3D tc;
    int index;
    int_list tri_v;
    int_list tri_tc;

    for ( int k = 0; k < (int)rwy_tris.size(); ++k ) {
	cout << "tri " << k << endl;
	FGPolygon tri_poly = rwy_tris[k];
	for ( int i = 0; i < tri_poly.contours(); ++i ) {
	    tri_v.clear();
	    tri_tc.clear();
	    for ( int j = 0; j < tri_poly.contour_size(i); ++j ) {
		p = tri_poly.get_pt( i, j );
		index = nodes.unique_add( p );
		tri_v.push_back( index );
		tc = rwy_txs[k].get_pt( i, j );
		index = texcoords.unique_add( tc );
		tri_tc.push_back( index );
	    }
	    tris_v.push_back( tri_v );
	    tris_tc.push_back( tri_tc );
	    tri_materials.push_back( "Concrete" );
	}
    }
    
    // add base points
    point_list base_txs; 
    int_list base_tc;
    for ( int i = 0; i < base_tris.contours(); ++i ) {
	tri_v.clear();
	tri_tc.clear();
	for ( int j = 0; j < base_tris.contour_size(i); ++j ) {
	    p = base_tris.get_pt( i, j );
	    index = nodes.unique_add( p );
	    tri_v.push_back( index );
	}
	tris_v.push_back( tri_v );
	tri_materials.push_back( "Grass" );

	base_txs.clear();
	base_txs = calc_tex_coords( b, nodes.get_node_list(), tri_v );

	base_tc.clear();
	for ( int j = 0; j < (int)base_txs.size(); ++j ) {
	    tc = base_txs[j];
	    // cout << "base_tc = " << tc << endl;
	    index = texcoords.simple_add( tc );
	    base_tc.push_back( index );
	}
	tris_tc.push_back( base_tc );
    }

    // calculate node elevations
    point_list geod_nodes = calc_elevations( root, nodes.get_node_list() );
    cout << "Done with calc_elevations()" << endl;

    // calculate wgs84 mapping of nodes
    point_list wgs84_nodes;
    for ( int i = 0; i < (int)geod_nodes.size(); ++i ) {
	p.setx( geod_nodes[i].x() * DEG_TO_RAD );
	p.sety( geod_nodes[i].y() * DEG_TO_RAD );
	p.setz( geod_nodes[i].z() );
	wgs84_nodes.push_back( fgGeodToCart( p ) );
    }
    double gbs_radius = calc_bounding_radius( gbs_center, wgs84_nodes );
    cout << "Done with wgs84 node mapping" << endl;

    // calculate normal(s) for this airport
    p.setx( base_tris.get_pt(0, 0).x() * DEG_TO_RAD );
    p.sety( base_tris.get_pt(0, 0).y() * DEG_TO_RAD );
    p.setz( 0 );
    Point3D tmp = fgGeodToCart( p );
    // cout << "geod = " << p << endl;
    // cout << "cart = " << tmp << endl;

    sgdVec3 vn;
    sgdSetVec3( vn, tmp.x(), tmp.y(), tmp.z() );
    sgdNormalizeVec3( vn );
    point_list normals;
    normals.clear();
    for ( int i = 0; i < (int)nodes.size(); ++i ) {
	normals.push_back( Point3D( vn[0], vn[1], vn[2] ) );
    }
    cout << "found normal for this airport = " << tmp << endl;

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    // null structures
    group_list strips_v; strips_v.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();

    string objpath = root + "/AirportObj";
    string name = apt_code;

    write_obj( objpath, b, name, gbs_center, gbs_radius, 
	       wgs84_nodes, normals,
	       texcoords.get_node_list(), 
	       tris_v, tris_tc, tri_materials,
	       strips_v, strips_tc, strip_materials, 
	       fans_v, fans_tc, fan_materials );

    write_index( objpath, b, name );

    string holepath = root + "/AirportArea";
    long int poly_index = poly_index_next();
    write_boundary( holepath, b, hull, poly_index );
}
