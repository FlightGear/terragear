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

#include <iostream>
SG_USING_STD(cout);
SG_USING_STD(cerr);
SG_USING_STD(endl);

#include <plib/sg.h>			// plib include

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/texcoord.hxx>

#include <Array/array.hxx>
#include <Geometry/poly_support.hxx>
#include <Geometry/trinodes.hxx>
#include <Output/output.hxx>
#include <Polygon/index.hxx>
#include <Polygon/polygon.hxx>
#include <Polygon/split.hxx>
#include <Polygon/superpoly.hxx>
#include <Triangulate/trieles.hxx>

#include "build.hxx"
#include "convex_hull.hxx"
#include "lights.hxx"
#include "point2d.hxx"
#include "poly_extra.hxx"
#include "runway.hxx"
#include "rwy_common.hxx"
#include "rwy_nonprec.hxx"
#include "rwy_prec.hxx"
#include "rwy_simple.hxx"
#include "rwy_visual.hxx"
#include "taxiway.hxx"
#include "texparams.hxx"


// calculate texture coordinates for runway section using the provided
// texturing parameters.  Returns a mirror polygon to the runway,
// except each point is the texture coordinate of the corresponding
// point in the original polygon.
static FGPolygon rwy_section_tex_coords( const FGPolygon& in_poly,
					 const FGTexParams& tp,
                                         const bool clip_result )
{
    int i, j;
    FGPolygon result;
    result.erase();
    // double length = rwy.length * SG_FEET_TO_METER;
    // double width = rwy.width * SG_FEET_TO_METER;

    Point3D center = tp.get_center();
    Point3D min = tp.get_min();
    Point3D max = tp.get_max();
    double angle = tp.get_angle();
    cout << "section heading = " << angle << endl;
    cout << "center = " << center << endl;
    cout << "min = " << min << endl;
    cout << "max = " << max << endl;
    Point3D p, t;
    double x, y, tx, ty;

    for ( i = 0; i < in_poly.contours(); ++i ) {
	for ( j = 0; j < in_poly.contour_size( i ); ++j ) {
	    p = in_poly.get_pt( i, j );
	    cout << "point = " << p << endl;

	    //
	    // 1. Calculate distance and bearing from the center of
	    // the runway
	    //

	    // given alt, lat1, lon1, lat2, lon2, calculate starting
	    // and ending az1, az2 and distance (s).  Lat, lon, and
	    // azimuth are in degrees.  distance in meters
	    double az1, az2, dist;
	    geo_inverse_wgs_84( 0, center.y(), center.x(), p.y(), p.x(),
				&az1, &az2, &dist );
	    // cout << "basic course = " << az1 << endl;

	    //
	    // 2. Rotate this back into a coordinate system where Y
	    // runs the length of the runway and X runs crossways.
	    //

	    double course = az1 - angle + 90;
	    // cout << "course = " << course << endl;
	    while ( course < -360 ) { course += 360; }
	    while ( course > 360 ) { course -= 360; }
	    // cout << "Dist = " << dist << endl;
	    // cout << "  Course = " << course * 180.0 / SGD_PI << endl;

	    //
	    // 3. Convert from polar to cartesian coordinates
	    //

	    x = cos( course * SGD_DEGREES_TO_RADIANS ) * dist;
	    y = sin( course * SGD_DEGREES_TO_RADIANS ) * dist;
	    cout << "  x = " << x << " y = " << y << endl;
	    cout << "  min = " << min << endl;
	    cout << "  max = " << max << endl;
	    //
	    // 4. Map x, y point into texture coordinates
	    //
	    
	    tx = (x - min.x()) / (max.x() - min.x());
	    tx = ((int)(tx * 100)) / 100.0;
	    cout << "  (" << tx << ")" << endl;

            if ( clip_result ) {
                if ( tx < 0.0 ) { tx = 0.0; }
                if ( tx > 1.0 ) { tx = 1.0; }
            }

	    // ty = (y - min.y()) / (max.y() - min.y());
	    ty = (max.y() - y) / (max.y() - min.y());
	    ty = ((int)(ty * 100)) / 100.0;
	    cout << "  (" << ty << ")" << endl;

            if ( clip_result ) {
                if ( ty < 0.0 ) { ty = 0.0; }
                if ( ty > 1.0 ) { ty = 1.0; }
            }

	    t = Point3D( tx, ty, 0 );
	    cout << "  (" << tx << ", " << ty << ")" << endl;

	    result.add_node( i, t );
	}
    }

    return result;
}


// fix node elevations
point_list calc_elevations( const string& root, const point_list& geod_nodes ) {
    bool done = false;
    point_list result = geod_nodes;
    int i, j;
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
	    SGBucket b( result[i].x(), result[i].y() );
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
	    for ( j = 0; j < (int)result.size(); ++j ) {
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


// build a runway
void build_runway( const FGRunway& rwy_info,
		   superpoly_list *rwy_polys,
		   texparams_list *texparams,
		   FGPolygon *accum,
		   FGPolygon *apt_base )
{
    cout << "surface flags = " << rwy_info.surface_flags << endl;
    string surface_flag = rwy_info.surface_flags.substr(1, 1);
    cout << "surface flag = " << surface_flag << endl;

    string material;
    if ( surface_flag == "A" ) {
        if ( !rwy_info.really_taxiway ) {
            material = "pa_";	// asphalt
        } else {
            material = "pa_taxiway";
        }
    } else if ( surface_flag == "C" ) {
        if ( !rwy_info.really_taxiway ) {
            material = "pc_";	// concrete
        } else {
            if ( rwy_info.width > 150 ) {
                material = "pc_tiedown";
            } else {
                material = "pc_taxiway";
            }
        }
    } else if ( surface_flag == "D" ) {
        material = "dirt_rwy";
    } else if ( surface_flag == "G" ) {
        material = "grass_rwy";
    } else if ( surface_flag == "L" ) {
        if ( rwy_info.really_taxiway ) {
            material = "lakebed_taxiway";
        } else {
            material = "dirt_rwy";
        }
    } else if ( surface_flag == "T" ) {
        material = "grass_rwy";
    } else if ( surface_flag == "W" ) {
        // water ???
    } else {
        cout << "unknown runway type!" << endl;
        exit(-1);
    }


    string type_flag = rwy_info.surface_flags.substr(2, 1);
    cout << "type flag = " << type_flag << endl;

    if ( rwy_info.really_taxiway ) {
	gen_taxiway( rwy_info, material,
                     rwy_polys, texparams, accum );
    } else if ( surface_flag == "D" || surface_flag == "G" ||
	 surface_flag == "T" )
    {
	gen_simple_rwy( rwy_info, material,
			rwy_polys, texparams, accum );
    } else if ( type_flag == "P" ) {
	// precision runway markings
	gen_precision_rwy( rwy_info, material,
			   rwy_polys, texparams, accum );
    } else if ( type_flag == "R" ) {
	// non-precision runway markings
	gen_non_precision_rwy( rwy_info, material,
			       rwy_polys, texparams, accum );
    } else if ( type_flag == "V" ) {
	// visual runway markings
	gen_visual_rwy( rwy_info, material,
			rwy_polys, texparams, accum );
    } else if ( type_flag == "B" ) {
	// bouys (sea plane base)
	// do nothing for now.
    } else {
	// unknown runway code ... hehe, I know, let's just die
	// right here so the programmer has to fix his code if a
	// new code ever gets introduced. :-)
	cout << "Unknown runway code in build.cxx:build_airport()" << endl;
	cout << "dying ..." << endl;
	exit(-1);
    }

    FGPolygon base;
    if ( rwy_info.really_taxiway ) {
	base = gen_runway_area_w_expand( rwy_info, 10, 10 );
    } else {
	base = gen_runway_area_w_scale( rwy_info, 1.05, 1.5 );
    }

    // add base to apt_base
    *apt_base = polygon_union( base, *apt_base );
}


// build 3d airport
void build_airport( string airport_raw, string_list& runways_raw,
                    string_list& taxiways_raw, const string& root )
{
    int i, j, k;

    superpoly_list rwy_polys;
    texparams_list texparams;

    // poly_list rwy_tris, rwy_txs;
    FGPolygon runway, runway_a, runway_b, clipped_a, clipped_b;
    FGPolygon split_a, split_b;
    FGPolygon apt_base;
    Point3D p;

    FGPolygon accum;
    accum.erase();

    // parse main airport information
    double apt_lon, apt_lat;
    int elev;

    cerr << airport_raw << endl;
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

    SGBucket b( apt_lon, apt_lat );
    Point3D center_geod( b.get_center_lon() * SGD_DEGREES_TO_RADIANS,
			 b.get_center_lat() * SGD_DEGREES_TO_RADIANS, 0 );
    Point3D gbs_center = sgGeodToCart( center_geod );
    cout << b.gen_base_path() << "/" << b.gen_index_str() << endl;

    // Ignore any seaplane bases
    if ( apt_type == "S" ) {
	return;
    }

    // parse runways and generate the vertex list
    runway_list runways;
    runways.clear();
    string rwy_str;

    for ( i = 0; i < (int)runways_raw.size(); ++i ) {
	rwy_str = runways_raw[i];

	FGRunway rwy;

        rwy.really_taxiway = false;

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

	rwy.surface_flags = rwy_str.substr(47, 5);

	rwy.end1_flags = rwy_str.substr(53, 4);

        string rwy_disp_threshold1 = rwy_str.substr(58, 6);
	rwy.disp_thresh1 = atoi( rwy_disp_threshold1.c_str() );

        string rwy_stopway1 = rwy_str.substr(65, 6);
	rwy.stopway1 = atoi( rwy_stopway1.c_str() );

	rwy.end2_flags = rwy_str.substr(72, 4);

        string rwy_disp_threshold2 = rwy_str.substr(77, 6);
	rwy.disp_thresh2 = atoi( rwy_disp_threshold2.c_str() );

        string rwy_stopway2 = rwy_str.substr(84, 6);
	rwy.stopway2 = atoi( rwy_stopway2.c_str() );

	cout << "  no    = " << rwy.rwy_no << endl;
	cout << "  lat   = " << rwy_lat << " " << rwy.lat << endl;
	cout << "  lon   = " << rwy_lon << " " << rwy.lon << endl;
	cout << "  hdg   = " << rwy_hdg << " " << rwy.heading << endl;
	cout << "  len   = " << rwy_len << " " << rwy.length << endl;
	cout << "  width = " << rwy_width << " " << rwy.width << endl;
	cout << "  sfc   = " << rwy.surface_flags << endl;
	cout << "  end1  = " << rwy.end1_flags << endl;
        cout << "  dspth1= " << rwy_disp_threshold1 << " " << rwy.disp_thresh1 << endl;
        cout << "  stop1 = " << rwy_stopway1 << " " << rwy.stopway1 << endl;
	cout << "  end2  = " << rwy.end2_flags << endl;
        cout << "  dspth2= " << rwy_disp_threshold2 << " " << rwy.disp_thresh2 << endl;
        cout << "  stop2 = " << rwy_stopway2 << " " << rwy.stopway2 << endl;

	runways.push_back( rwy );
    }

    // parse taxiways and generate the vertex list
    runway_list taxiways;
    taxiways.clear();

    for ( i = 0; i < (int)taxiways_raw.size(); ++i ) {
	rwy_str = taxiways_raw[i];

	FGRunway taxi;

        taxi.really_taxiway = true;

	cout << rwy_str << endl;
	taxi.rwy_no = rwy_str.substr(2, 4);

	string rwy_lat = rwy_str.substr(6, 10);
	taxi.lat = atof( rwy_lat.c_str() );

	string rwy_lon = rwy_str.substr(17, 11);
	taxi.lon = atof( rwy_lon.c_str() );

	string rwy_hdg = rwy_str.substr(29, 7);
	taxi.heading = atof( rwy_hdg.c_str() );

	string rwy_len = rwy_str.substr(36, 7);
	taxi.length = atoi( rwy_len.c_str() );

	string rwy_width = rwy_str.substr(43, 5);
	taxi.width = atoi( rwy_width.c_str() );

	taxi.surface_flags = rwy_str.substr(48, 3);

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

	taxiways.push_back( taxi );
    }

    FGSuperPoly sp;
    FGTexParams tp;

    // First pass: generate the precision runways since these have
    // precidence
    for ( i = 0; i < (int)runways.size(); ++i ) {
	string type_flag = runways[i].surface_flags.substr(2, 1);
	if ( type_flag == "P" ) {
	    build_runway( runways[i], 
			  &rwy_polys, &texparams, &accum, &apt_base );
	}
    }

    // 2nd pass: generate the non-precision and visual runways
    for ( i = 0; i < (int)runways.size(); ++i ) {
	string type_flag = runways[i].surface_flags.substr(2, 1);
	if ( type_flag == "R" || type_flag == "V" ) {
	    build_runway( runways[i], 
			  &rwy_polys, &texparams, &accum, &apt_base );
	}
    }

    // 3rd pass: generate all remaining runways not covered in the first pass
    for ( i = 0; i < (int)runways.size(); ++i ) {
	string type_flag = runways[i].surface_flags.substr(2, 1);
	if ( type_flag != "P" && type_flag != "R" && type_flag != "V" ) {
	    build_runway( runways[i], 
			  &rwy_polys, &texparams, &accum, &apt_base );
	}
    }

    // 4th pass: generate all taxiways
    for ( i = 0; i < (int)taxiways.size(); ++i ) {
        build_runway( taxiways[i], &rwy_polys, &texparams, &accum, &apt_base );
    }
    // write_polygon( accum, "accum" );

    if ( apt_base.total_size() == 0 ) {
	cout << "no airport points generated" << endl;
	return;
    }

    // 5th pass: generate runway/taxiway lights
    point_list rwy_lights;
    rwy_lights.clear();
    for ( i = 0; i < (int)runways.size(); ++i ) {
	gen_runway_lights( runways[i], &rwy_lights );
    }

    // generate convex hull (no longer)
    // FGPolygon hull = convex_hull(apt_pts);

    FGPolygon filled_base = strip_out_holes( apt_base );
    // write_polygon( filled_base, "filled-base" );
    FGPolygon divided_base = split_long_edges( filled_base, 200.0 );
    // write_polygon( divided_base, "divided-base" );
    FGPolygon base_poly = polygon_diff( divided_base, accum );
    // write_polygon( base_poly, "base-raw" );

    // Try to remove duplicated nodes and other degeneracies
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	cout << "add nodes/remove dups section = " << k
	     << " " << rwy_polys[k].get_material() << endl;
	FGPolygon poly = rwy_polys[k].get_poly();
	cout << "total size before = " << poly.total_size() << endl;
	for ( i = 0; i < poly.contours(); ++i ) {
	    for ( j = 0; j < poly.contour_size(i); ++j ) {
		Point3D tmp = poly.get_pt(i, j);
		printf("  %.7f %.7f %.7f\n", tmp.x(), tmp.y(), tmp.z() );
	    }
	}

	poly = remove_dups( poly );
	cout << "total size after remove_dups() = "
	     << poly.total_size() << endl;

	for ( i = 0; i < poly.contours(); ++i ) {
	    for ( j = 0; j < poly.contour_size(i); ++j ) {
		Point3D tmp = poly.get_pt(i, j);
		printf("    %.7f %.7f %.7f\n", tmp.x(), tmp.y(), tmp.z() );
	    }
	}

	poly = reduce_degeneracy( poly );
	cout << "total size after reduce_degeneracy() = "
	     << poly.total_size() << endl;

	for ( i = 0; i < poly.contours(); ++i ) {
	    for ( j = 0; j < poly.contour_size(i); ++j ) {
		Point3D tmp = poly.get_pt(i, j);
		printf("    %.7f %.7f %.7f\n", tmp.x(), tmp.y(), tmp.z() );
	    }
	}

	rwy_polys[k].set_poly( poly );
    }

    // add segments to polygons to remove any possible "T"
    // intersections
    FGTriNodes tmp_nodes;

    // build temporary node list
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	FGPolygon poly = rwy_polys[k].get_poly();
	for ( i = 0; i < poly.contours(); ++i ) {
	    for ( j = 0; j < poly.contour_size( i ); ++j ) {
		tmp_nodes.unique_add( poly.get_pt(i, j) );
	    }
	}
    }
    for ( i = 0; i < base_poly.contours(); ++i ) {
	for ( j = 0; j < base_poly.contour_size( i ); ++j ) {
	    tmp_nodes.unique_add( base_poly.get_pt(i, j) );
	}
    }

#if 0
    // dump info for debugging purposes
    point_list ttt = tmp_nodes.get_node_list();
    FILE *fp = fopen( "tmp_nodes", "w" );
    for ( i = 0; i < (int)ttt.size(); ++i ) {
	fprintf(fp, "%.8f %.8f\n", ttt[i].x(), ttt[i].y());
    }
    fclose(fp);

    for ( i = 0; i < base_poly.contours(); ++i ) {
	char name[256];
	sprintf(name, "l%d", i );
	FILE *fp = fopen( name, "w" );

	for ( j = 0; j < base_poly.contour_size( i ) - 1; ++j ) {
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

    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	FGPolygon poly = rwy_polys[k].get_poly();
	poly = add_nodes_to_poly( poly, tmp_nodes );
	cout << "total size after add nodes = " << poly.total_size() << endl;

#if 0
	char tmp[256];
	sprintf( tmp, "r%d", k );
	write_polygon( poly, tmp );
#endif

	rwy_polys[k].set_poly( poly );
    }

    // One more pass to try to get rid of other yukky stuff
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	FGPolygon poly = rwy_polys[k].get_poly();

        poly = remove_dups( poly );
	cout << "total size after remove_dups() = " << poly.total_size() << endl;
        poly = remove_bad_contours( poly );
	cout << "total size after remove_bad() = " << poly.total_size() << endl;

	rwy_polys[k].set_poly( poly );
    }

    cout << "add nodes base " << endl;
    base_poly = add_nodes_to_poly( base_poly, tmp_nodes );
    // write_polygon( base_poly, "base-add" );
    cout << "remove dups base " << endl;
    base_poly = remove_dups( base_poly );
    cout << "remove bad contours base" << endl;
    base_poly = remove_bad_contours( base_poly );
    // write_polygon( base_poly, "base-fin" );

    // tesselate the polygons and prepair them for final output

    for ( i = 0; i < (int)rwy_polys.size(); ++i ) {
        cout << "Tesselating section = " << i << endl;

	FGPolygon poly = rwy_polys[i].get_poly();
	cout << "total size before = " << poly.total_size() << endl;
	FGPolygon tri = polygon_tesselate_alt( poly );
	cout << "total size after = " << tri.total_size() << endl;

        FGPolygon tc;
        if ( rwy_polys[i].get_flag() ) {
            cout << "no clip" << endl;
            tc = rwy_section_tex_coords( tri, texparams[i], false );
        } else {
            tc = rwy_section_tex_coords( tri, texparams[i], true );
        }

	rwy_polys[i].set_tris( tri );
	rwy_polys[i].set_texcoords( tc );
	rwy_polys[i].set_tri_mode( GL_TRIANGLES );
    }

    cout << "Tesselating base" << endl;
    FGPolygon base_tris = polygon_tesselate_alt( base_poly );

#if 0
    // dump more debugging output
    for ( i = 0; i < base_strips.contours(); ++i ) {
	char name[256];
	sprintf(name, "s%d", i );
	FILE *fp = fopen( name, "w" );

	for ( j = 0; j < base_strips.contour_size( i ) - 1; ++j ) {
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

    group_list pts_v; pts_v.clear();
    string_list pt_materials; pt_materials.clear();

    group_list tris_v; tris_v.clear();
    group_list tris_tc; tris_tc.clear();
    string_list tri_materials; tri_materials.clear();

    group_list strips_v; strips_v.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();

    Point3D tc;
    int index;
    int_list pt_v, tri_v, strip_v;
    int_list tri_tc, strip_tc;

    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	cout << "tri " << k << endl;
	// FGPolygon tri_poly = rwy_tris[k];
	FGPolygon tri_poly = rwy_polys[k].get_tris();
	FGPolygon tri_txs = rwy_polys[k].get_texcoords();
	string material = rwy_polys[k].get_material();
	cout << "material = " << material << endl;
	cout << "poly size = " << tri_poly.contours() << endl;
	cout << "texs size = " << tri_txs.contours() << endl;
	for ( i = 0; i < tri_poly.contours(); ++i ) {
	    tri_v.clear();
	    tri_tc.clear();
	    for ( j = 0; j < tri_poly.contour_size(i); ++j ) {
		p = tri_poly.get_pt( i, j );
		index = nodes.unique_add( p );
		tri_v.push_back( index );
		tc = tri_txs.get_pt( i, j );
		index = texcoords.unique_add( tc );
		tri_tc.push_back( index );
	    }
	    tris_v.push_back( tri_v );
	    tris_tc.push_back( tri_tc );
	    tri_materials.push_back( material );
	}
    }

    // add base points
    point_list base_txs; 
    int_list base_tc;
    for ( i = 0; i < base_tris.contours(); ++i ) {
	tri_v.clear();
	tri_tc.clear();
	for ( j = 0; j < base_tris.contour_size(i); ++j ) {
	    p = base_tris.get_pt( i, j );
	    index = nodes.unique_add( p );
	    tri_v.push_back( index );
	}
	tris_v.push_back( tri_v );
	tri_materials.push_back( "Grass" );

	base_txs.clear();
	base_txs = calc_tex_coords( b, nodes.get_node_list(), tri_v );

	base_tc.clear();
	for ( j = 0; j < (int)base_txs.size(); ++j ) {
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

    // add base skirt (to hide potential cracks)
    //
    // this has to happen after we've calculated the node elevations
    // but before we convert to wgs84 coordinates
    int uindex, lindex;

    for ( i = 0; i < divided_base.contours(); ++i ) {
	strip_v.clear();
	strip_tc.clear();

	// prime the pump ...
	p = divided_base.get_pt( i, 0 );
	uindex = nodes.find( p );
	if ( uindex >= 0 ) {
	    Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
	    cout << geod_nodes[uindex] << " <-> " << lower << endl;
	    lindex = nodes.simple_add( lower );
	    geod_nodes.push_back( lower );
	    strip_v.push_back( uindex );
	    strip_v.push_back( lindex );
	} else {
	    cout << "Ooops missing node when building skirt ... dying!"
		 << endl;
	    exit(-1);
	}

	// loop through the list
	for ( j = 1; j < divided_base.contour_size(i); ++j ) {
	    p = divided_base.get_pt( i, j );
	    uindex = nodes.find( p );
	    if ( uindex >= 0 ) {
		Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
		cout << geod_nodes[uindex] << " <-> " << lower << endl;
		lindex = nodes.simple_add( lower );
		geod_nodes.push_back( lower );
		strip_v.push_back( lindex );
		strip_v.push_back( uindex );
	    } else {
		cout << "Ooops missing node when building skirt ... dying!"
		     << endl;
		exit(-1);
	    }
	}

	// close off the loop
	p = divided_base.get_pt( i, 0 );
	uindex = nodes.find( p );
	if ( uindex >= 0 ) {
	    Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
	    cout << geod_nodes[uindex] << " <-> " << lower << endl;
	    lindex = nodes.simple_add( lower );
	    geod_nodes.push_back( lower );
	    strip_v.push_back( lindex );
	    strip_v.push_back( uindex );
	} else {
	    cout << "Ooops missing node when building skirt ... dying!"
		 << endl;
	    exit(-1);
	}

	strips_v.push_back( strip_v );
	strip_materials.push_back( "Grass" );

	base_txs.clear();
	base_txs = calc_tex_coords( b, nodes.get_node_list(), strip_v );

	base_tc.clear();
	for ( j = 0; j < (int)base_txs.size(); ++j ) {
	    tc = base_txs[j];
	    // cout << "base_tc = " << tc << endl;
	    index = texcoords.simple_add( tc );
	    base_tc.push_back( index );
	}
	strips_tc.push_back( base_tc );
    }

    // add light points
    FGTriNodes light_nodes;
    light_nodes.clear();
    for ( i = 0; i < (int)rwy_lights.size(); ++i ) {
        p = rwy_lights[i];
        index = light_nodes.simple_add( p );
    }

    // calculate light node elevations
    point_list geod_light_nodes
        = calc_elevations( root, light_nodes.get_node_list() );
    cout << "Done with (light) calc_elevations()" << endl;

    // this is a little round about, but what we want to calculate the
    // light node elevations as ground + an offset so we do them
    // seperately, then we add them back into nodes to get the index
    // out, but also geod_nodes to maintain consistancy between these
    // two lists.
    pt_v.clear();
    for ( i = 0; i < (int)geod_light_nodes.size(); ++i ) {
        p = geod_light_nodes[i];
        p.setz( p.z() + 0.5 );
        index = nodes.simple_add( p );
        pt_v.push_back( index );
        geod_nodes.push_back( p );
    }
    pts_v.push_back( pt_v );
    pt_materials.push_back( "LIGHTS" );

    // calculate wgs84 mapping of nodes
    point_list wgs84_nodes;
    for ( i = 0; i < (int)geod_nodes.size(); ++i ) {
	p.setx( geod_nodes[i].x() * SGD_DEGREES_TO_RADIANS );
	p.sety( geod_nodes[i].y() * SGD_DEGREES_TO_RADIANS );
	p.setz( geod_nodes[i].z() );
	wgs84_nodes.push_back( sgGeodToCart( p ) );
    }
    float gbs_radius = sgCalcBoundingRadius( gbs_center, wgs84_nodes );
    cout << "Done with wgs84 node mapping" << endl;

    // calculate normal for this airport
    p.setx( base_tris.get_pt(0, 0).x() * SGD_DEGREES_TO_RADIANS );
    p.sety( base_tris.get_pt(0, 0).y() * SGD_DEGREES_TO_RADIANS );
    p.setz( 0 );
    Point3D tmp = sgGeodToCart( p );
    // cout << "geod = " << p << endl;
    // cout << "cart = " << tmp << endl;

    sgdVec3 vn;
    sgdSetVec3( vn, tmp.x(), tmp.y(), tmp.z() );
    sgdNormalizeVec3( vn );
    point_list normals;
    normals.clear();
    normals.push_back( Point3D( vn[0], vn[1], vn[2] ) );
    cout << "found normal for this airport = " << tmp << endl;

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    string objpath = root + "/AirportObj";
    string name = apt_code + ".btg";

    SGBinObject obj;

    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals );
    obj.set_texcoords( texcoords.get_node_list() );
    obj.set_pts_v( pts_v );
    obj.set_pt_materials( pt_materials );
    obj.set_tris_v( tris_v );
    obj.set_tris_tc( tris_tc ); 
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_tc( strips_tc ); 
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );

    bool result;
    /* result = obj.write_ascii( objpath, name, b ); */
    result = obj.write_bin( objpath, name, b );
    if ( !result ) {
	cout << "error writing file. :-(" << endl;
	exit(-1);
    }

#if 0
    // checking result of write, remove the read_bin() before this
    // goes into production
    string file = objpath + "/" + b.gen_base_path() + "/" + name;
    point_list tmp_texcoords = texcoords.get_node_list();
    sgReadBinObj( file, gbs_center, &gbs_radius, 
		  wgs84_nodes, normals,
		  tmp_texcoords, 
		  tris_v, tris_tc, tri_materials,
		  strips_v, strips_tc, strip_materials, 
		  fans_v, fans_tc, fan_materials );
#endif

    write_index( objpath, b, name );

    string holepath = root + "/AirportArea";
    // long int poly_index = poly_index_next();
    // write_boundary( holepath, b, hull, poly_index );
    split_polygon( holepath, HoleArea, divided_base );
}
