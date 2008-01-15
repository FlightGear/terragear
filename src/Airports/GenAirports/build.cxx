// build.cxx -- routines to build polygon model of an airport from the runway
//              definition
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: build.cxx,v 1.116 2005-10-31 18:43:02 curt Exp $
//


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/strutils.hxx>
#include <simgear/structure/exception.hxx>

#include <stdio.h>
#include <stdlib.h>		// for atoi() atof()
#include <time.h>

#include <list>
#include <map>
#include STL_STRING

#include <plib/sg.h>			// plib include

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/texcoord.hxx>

#include <Geometry/poly_support.hxx>
#include <Geometry/trinodes.hxx>
#include <Output/output.hxx>
#include <Polygon/chop.hxx>
#include <Polygon/index.hxx>
#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Triangulate/trieles.hxx>

#include "apt_surface.hxx"
#include "convex_hull.hxx"
#include "elevations.hxx"
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

#include "build.hxx"

SG_USING_STD(map);
SG_USING_STD(less);
SG_USING_STD(string);


// calculate texture coordinates for runway section using the provided
// texturing parameters.  Returns a mirror polygon to the runway,
// except each point is the texture coordinate of the corresponding
// point in the original polygon.
static TGPolygon rwy_section_tex_coords( const TGPolygon& in_poly,
					 const TGTexParams& tp,
                                         const bool clip_result )
{
    int i, j;
    TGPolygon result;
    result.erase();
    // double length = rwy.length * SG_FEET_TO_METER;
    // double width = rwy.width * SG_FEET_TO_METER;

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
	    // tx = ((int)(tx * 100)) / 100.0;
	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

            if ( clip_result) {
                if ( tx < 0.0 ) { tx = 0.0; }
                if ( tx > 1.0 ) { tx = 1.0; }
            }

	    // ty = (y - min.y()) / (max.y() - min.y());
	    ty = y / length;
            tmp = y / length;
            ty = tmp * (maxv - minv) + minv;
	    // ty = ((int)(ty * 100)) / 100.0;
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


#if 0                           // DEAD CODE 10/15/2004 CLO
// strip trailing spaces
static void my_chomp( string& str ) {
    SG_LOG(SG_GENERAL, SG_DEBUG, "my_chomp()");
    SG_LOG(SG_GENERAL, SG_DEBUG, "'" << str.substr( str.length() - 1, 1 ) << "'");
    while ( str.substr( str.length() - 1, 1 ) == " " ) {
	str = str.substr( 0, str.length() - 1 );
	SG_LOG(SG_GENERAL, SG_DEBUG, "'" << str.substr( str.length() - 1, 1 ) << "'");
    }
}
#endif


// build a runway
static void build_runway( const TGRunway& rwy_info,
                          double alt_m,
                          superpoly_list *rwy_polys,
                          texparams_list *texparams,
                          TGPolygon *accum,
                          TGPolygon *apt_base,
                          TGPolygon *apt_clearing )
{
    SG_LOG(SG_GENERAL, SG_DEBUG, "surface code = " << rwy_info.surface_code);
    int surface_code = rwy_info.surface_code;
    SG_LOG(SG_GENERAL, SG_DEBUG, "surface code = " << surface_code);
    string lighting_flags = rwy_info.lighting_flags;
    SG_LOG(SG_GENERAL, SG_DEBUG, "lighting flags = " << lighting_flags);

    string vasi1 = lighting_flags.substr(0,1);
    string rwylt1 = lighting_flags.substr(1,1);
    string apprch1 = lighting_flags.substr(2,1);
    string vasi2 = lighting_flags.substr(3,1);
    string rwylt2 = lighting_flags.substr(4,1);
    string apprch2 = lighting_flags.substr(5,1);

    string material;
    if ( surface_code == 1 /* Asphalt */ ) {
        if ( !rwy_info.really_taxiway ) {
            material = "pa_";
        } else {
            if ( rwy_info.width <= 150 && rwylt1 == "6" ) {
                material = "pa_taxiway";
            } else {
                material = "pa_tiedown";
            }
        }
    } else if ( surface_code == 2 /* Concrete */ ) {
        if ( !rwy_info.really_taxiway ) {
            material = "pc_";
        } else {
            if ( rwy_info.width <= 150 && rwylt1 == "6" ) {
                material = "pc_taxiway";
            } else {
                material = "pc_tiedown";
            }
        }
    } else if ( surface_code == 3 /* Turf/Grass */ ) {
        material = "grass_rwy";
    } else if ( surface_code == 4 /* Dirt */
                || surface_code == 5 /* Gravel */ ) {
        material = "dirt_rwy";
    } else if ( surface_code == 12 /* Dry Lakebed */ ) {
        if ( rwy_info.really_taxiway ) {
            material = "lakebed_taxiway";
        } else {
            material = "dirt_rwy";
        }
    } else if ( surface_code == 13 /* Water runway (buoy's?) */ ) {
        // water
    } else {
        SG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << surface_code);
	throw sg_exception("unknown runway type!");
    }


    SG_LOG(SG_GENERAL, SG_DEBUG, "marking code = " << rwy_info.marking_code);

    if ( rwy_info.really_taxiway ) {
	gen_taxiway( rwy_info, alt_m, material,
                     rwy_polys, texparams, accum );
    } else if ( surface_code == 3 /* Turf/Grass */
                || surface_code == 4 /* Dirt */
                || surface_code == 5 /* Gravel */ )
    {
	gen_simple_rwy( rwy_info, alt_m, material,
			rwy_polys, texparams, accum );
    } else if ( rwy_info.marking_code == 3 /* Precision */ ) {
	// precision runway markings
	gen_precision_rwy( rwy_info, alt_m, material,
			   rwy_polys, texparams, accum );
    } else if ( rwy_info.marking_code == 2 /* Non-precision */ ) {
	// non-precision runway markings
	gen_non_precision_rwy( rwy_info, alt_m, material,
			       rwy_polys, texparams, accum );
    } else if ( rwy_info.marking_code == 1 /* Visual */ ) {
	// visual runway markings
	gen_visual_rwy( rwy_info, alt_m, material,
			rwy_polys, texparams, accum );
    } else if ( rwy_info.marking_code == 0 /* No known markings, lets assume Visual */ ) {
	// visual runway markings
	gen_visual_rwy( rwy_info, alt_m, material,
			rwy_polys, texparams, accum );
    } else if ( surface_code == 13 /* Water buoys */ ) {
	// do nothing for now.
    } else {
	// unknown runway code ... hehe, I know, let's just die
	// right here so the programmer has to fix his code if a
	// new code ever gets introduced. :-)
        SG_LOG( SG_GENERAL, SG_ALERT, "Unknown runway code = " <<
                rwy_info.marking_code );
	throw sg_exception("Unknown runway code in build.cxx:build_airport()");
    }

    TGPolygon base, safe_base;
    if ( rwy_info.really_taxiway ) {
	base = gen_runway_area_w_extend( rwy_info, 0.0, 10.0, 0.0, 0.0, 10.0 );
        // also clear a safe area around the taxiway
        safe_base
            = gen_runway_area_w_extend( rwy_info, 0.0, 40.0, 0.0, 0.0, 40.0 );
    } else {
	base = gen_runway_area_w_extend( rwy_info, 0.0, 20.0, 0.0, 0.0, 20.0 );
        // also clear a safe area around the runway
        safe_base
            = gen_runway_area_w_extend( rwy_info, 0.0, 180.0, 0.0, 0.0, 50.0 );
    }
    *apt_clearing = tgPolygonUnion(safe_base, *apt_clearing);

    // add base to apt_base
    *apt_base = tgPolygonUnion( base, *apt_base );
}


// build 3d airport
void build_airport( string airport_id, float alt_m,
                    string_list& runways_raw,
                    string_list& beacons_raw,
                    string_list& towers_raw,
                    string_list& windsocks_raw,                    
                    const string& root,
                    const string_list& elev_src )
{
    int i, j, k;

    superpoly_list rwy_polys;
    texparams_list texparams;

    // poly_list rwy_tris, rwy_txs;
    TGPolygon runway, runway_a, runway_b, clipped_a, clipped_b;
    TGPolygon split_a, split_b;
    TGPolygon apt_base;
    TGPolygon apt_clearing;
    Point3D p;

    TGPolygon accum;
    accum.erase();

    // parse main airport information
    double apt_lon = 0.0, apt_lat = 0.0;
    int rwy_count = 0;

    SG_LOG( SG_GENERAL, SG_INFO, "Building " << airport_id );

    // parse runways/taxiways and generate the vertex list
    runway_list runways; runways.clear();
    runway_list taxiways; taxiways.clear();

    for ( i = 0; i < (int)runways_raw.size(); ++i ) {
        ++rwy_count;

	string rwy_str = runways_raw[i];
        vector<string> token = simgear::strutils::split( rwy_str );

	TGRunway rwy;

	SG_LOG(SG_GENERAL, SG_DEBUG, rwy_str);
	rwy.rwy_no = token[3];
        rwy.really_taxiway = (rwy.rwy_no == "xxx");
        rwy.generated = false;

	rwy.lat = atof( token[1].c_str() );
        apt_lat += rwy.lat;

	rwy.lon = atof( token[2].c_str() );
        apt_lon += rwy.lon;

	rwy.heading = atof( token[4].c_str() );

	rwy.length = atoi( token[5].c_str() );
	rwy.width = atoi( token[8].c_str() );

        string rwy_displ_threshold = token[6];
        vector<string> displ
            = simgear::strutils::split( rwy_displ_threshold, "." );
        rwy.disp_thresh1 = atoi( displ[0].c_str() );
        rwy.disp_thresh2 = atoi( displ[1].c_str() );

        string rwy_stopway = token[7];
        vector<string> stop
            = simgear::strutils::split( rwy_stopway, "." );
        rwy.stopway1 = atoi( stop[0].c_str() );
        rwy.stopway2 = atoi( stop[1].c_str() );

	rwy.lighting_flags = token[9];
	rwy.surface_code = atoi( token[10].c_str() );
	rwy.shoulder_code = token[11];
        rwy.marking_code = atoi( token[12].c_str() );
        rwy.smoothness = atof( token[13].c_str() );
        rwy.dist_remaining = (atoi( token[14].c_str() ) == 1 );

	if (token.size()>15) {
		string vasi_angles = token[15];
		vector<string> vasis = simgear::strutils::split( vasi_angles, "." );
		rwy.gs_angle1 = atof( vasis[0].c_str() ) * 0.01;
		rwy.gs_angle2 = atof( vasis[1].c_str() ) * 0.01;
	} else {
		rwy.gs_angle1 = rwy.gs_angle2 = 3.0;
	}

	SG_LOG( SG_GENERAL, SG_DEBUG, "  no    = " << rwy.rwy_no);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  lat   = " << rwy.lat);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  lon   = " << rwy.lon);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  hdg   = " << rwy.heading);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  len   = " << rwy.length);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  width = " << rwy.width);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  lighting = " << rwy.lighting_flags);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  sfc   = " << rwy.surface_code);
	SG_LOG( SG_GENERAL, SG_DEBUG, "  mrkgs  = " << rwy.marking_code);
        SG_LOG( SG_GENERAL, SG_DEBUG, "  dspth1= " << rwy.disp_thresh1);
        SG_LOG( SG_GENERAL, SG_DEBUG, "  stop1 = " << rwy.stopway1);
        SG_LOG( SG_GENERAL, SG_DEBUG, "  dspth2= " << rwy.disp_thresh2);
        SG_LOG( SG_GENERAL, SG_DEBUG, "  stop2 = " << rwy.stopway2);

        if ( rwy.really_taxiway ) {
            taxiways.push_back( rwy );
        } else {
            runways.push_back( rwy );
        }
    }
    SG_LOG(SG_GENERAL, SG_INFO, "Runway count = " << runways.size() );
    SG_LOG(SG_GENERAL, SG_INFO, "Taxiway count = " << taxiways.size() );

    SGBucket b( apt_lon / (double)rwy_count, apt_lat / (double)rwy_count );
    SG_LOG(SG_GENERAL, SG_INFO, b.gen_base_path() << "/" << b.gen_index_str());
    Point3D center_geod( b.get_center_lon() * SGD_DEGREES_TO_RADIANS, 	 
                         b.get_center_lat() * SGD_DEGREES_TO_RADIANS, 0 );
    Point3D gbs_center = sgGeodToCart( center_geod );

    point_list beacons; beacons.clear();
    for ( i = 0; i < (int)beacons_raw.size(); ++i ) {
	string beacon_str = beacons_raw[i];
        vector<string> token = simgear::strutils::split( beacon_str );

	Point3D beacon;

	SG_LOG(SG_GENERAL, SG_INFO, beacon_str);

	beacon.setlat( atof( token[1].c_str() ) );
	beacon.setlon( atof( token[2].c_str() ) );

	SG_LOG( SG_GENERAL, SG_DEBUG, "  beacon   = " << beacon );

	beacons.push_back( beacon );
    }

    point_list towers; towers.clear();
    for ( i = 0; i < (int)towers_raw.size(); ++i ) {
	string tower_str = towers_raw[i];
        vector<string> token = simgear::strutils::split( tower_str );

	Point3D tower;

	SG_LOG(SG_GENERAL, SG_INFO, tower_str);

	tower.setlat( atof( token[1].c_str() ) );
	tower.setlon( atof( token[2].c_str() ) );

	if (!atoi(token[4].c_str())) {
		// Ralf Gerlich: Skip towers that shall not be drawn
		continue;
	}

	SG_LOG( SG_GENERAL, SG_DEBUG, "  tower   = " << tower );

	towers.push_back( tower );
    }

    point_list windsocks; windsocks.clear();
    int_list windsock_types; windsock_types.clear();
    for ( i = 0; i < (int)windsocks_raw.size(); ++i ) {
	string windsock_str = windsocks_raw[i];
        vector<string> token = simgear::strutils::split( windsock_str );

	Point3D windsock;

	SG_LOG(SG_GENERAL, SG_INFO, windsock_str);

	windsock.setlat( atof( token[1].c_str() ) );
	windsock.setlon( atof( token[2].c_str() ) );

	string windsock_type = token[3];

	SG_LOG( SG_GENERAL, SG_DEBUG, "  windsock   = " << windsock );

	windsocks.push_back( windsock );
	if ( windsock_type == "0" ) {
	    windsock_types.push_back( 0 );
        } else {
	    windsock_types.push_back( 1 );
	}
    }

    TGSuperPoly sp;
    TGTexParams tp;

    // First pass: generate the precision runways since these have
    // precidence
    for ( i = 0; i < (int)runways.size(); ++i ) {
	if ( runways[i].marking_code == 3 /* Precision */ ) {
	    build_runway( runways[i], alt_m,
			  &rwy_polys, &texparams, &accum,
                          &apt_base, &apt_clearing );
	}
    }

    // 2nd pass: generate the non-precision and visual runways
    for ( i = 0; i < (int)runways.size(); ++i ) {
	if ( runways[i].marking_code == 2 /* Non-precision */
             || runways[i].marking_code == 1 /* Visual */ )
        {
            if ( runways[i].surface_code != 13 /* Water */ ) {
                // only build non-water runways
                build_runway( runways[i], alt_m,
                              &rwy_polys, &texparams, &accum,
                              &apt_base, &apt_clearing );
            }
        }
    }

    // 3rd pass: generate all remaining runways not covered in the first pass
    for ( i = 0; i < (int)runways.size(); ++i ) {
	if ( runways[i].marking_code != 3 /* Precision */
             && runways[i].marking_code != 2 /* Non-precision */
             && runways[i].marking_code != 1 /* Visual */ )
        {
            if ( runways[i].surface_code != 6 /* Asphalt Helipad */ &&
                 runways[i].surface_code != 7 /* Concrete Helipad */ &&
                 runways[i].surface_code != 8 /* Turf Helipad */ &&
                 runways[i].surface_code != 9 /* Dirt Helipad */ &&
                 runways[i].surface_code != 13 /* Water/buoy runway */ )
            {
                // only build non-water and non-heliport runways
                build_runway( runways[i], alt_m,
                              &rwy_polys, &texparams, &accum,
                              &apt_base, &apt_clearing );
            }
        }
    }

    // 4th pass: generate all taxiways

#if 0
    // we want to generate in order of largest size first so this will
    // look a little weird, but that's all I'm doing, otherwise a
    // simple list traversal would work fine.
    bool done = false;
    while ( !done ) {
        // find the largest taxiway
        int largest_idx = -1;
        double max_size = 0;
        for ( i = 0; i < (int)taxiways.size(); ++i ) {
            SG_LOG( SG_GENERAL, SG_DEBUG, "taxiway i = " << i );
            double size = taxiways[i].length * taxiways[i].width;
            if ( size > max_size && !taxiways[i].generated ) {
                SG_LOG( SG_GENERAL, SG_DEBUG, "taxiway max i = " << i );
                max_size = size;
                largest_idx = i;
            }
        }

        if ( largest_idx >= 0 ) {
            SG_LOG( SG_GENERAL, SG_DEBUG, "generating " << largest_idx );
            build_runway( taxiways[largest_idx], alt_m,
                          &rwy_polys, &texparams, &accum,
                          &apt_base, &apt_clearing );
            taxiways[largest_idx].generated = true;
        } else {
            SG_LOG( SG_GENERAL, SG_DEBUG, "done with taxiways." );
            done = true;
        }
    }
#else
    /* Ralf Gerlich: Generate Taxiways in specified order from bottom to top */
    for ( size_t i=0; i<taxiways.size(); ++i ) {
            SG_LOG( SG_GENERAL, SG_DEBUG, "generating " << i );
            build_runway( taxiways[i], alt_m,
                          &rwy_polys, &texparams, &accum,
                          &apt_base, &apt_clearing );
            taxiways[i].generated = true;
    }
#endif

    // Now generate small surface for each beacon
    TGPolygon obj_base, obj_safe_base;
    double obj_hdg = runways[0].heading;
    for ( i = 0; i < (int)beacons.size(); ++i ) {
        obj_base = gen_wgs84_area( beacons[i], 20.0, 0.0, 0.0, 20.0,
                                   obj_hdg, alt_m, false );
        obj_safe_base = gen_wgs84_area( beacons[i], 40.0, 0.0, 0.0, 40.0,
                                        obj_hdg, alt_m, false );
        
        apt_base = tgPolygonUnion( obj_base, apt_base );
        apt_clearing = tgPolygonUnion( obj_safe_base, apt_clearing );
    }

    // Now generate small surface for each tower
    for ( i = 0; i < (int)towers.size(); ++i ) {
        obj_base = gen_wgs84_area( towers[i], 20.0, 0.0, 0.0, 20.0,
                                   obj_hdg, alt_m, false );
        obj_safe_base = gen_wgs84_area( towers[i], 40.0, 0.0, 0.0, 40.0,
                                        obj_hdg, alt_m, false );
        
        apt_base = tgPolygonUnion( obj_base, apt_base );
        apt_clearing = tgPolygonUnion( obj_safe_base, apt_clearing );
    }

    // Now generate small surface for each windsock
    for ( i = 0; i < (int)windsocks.size(); ++i ) {
        obj_base = gen_wgs84_area( windsocks[i], 20.0, 0.0, 0.0, 20.0,
                                   obj_hdg, alt_m, false );
        obj_safe_base = gen_wgs84_area( windsocks[i], 40.0, 0.0, 0.0, 40.0,
                                        obj_hdg, alt_m, false );
        
        apt_base = tgPolygonUnion( obj_base, apt_base );
        apt_clearing = tgPolygonUnion( obj_safe_base, apt_clearing );
    }

    // 5th pass: generate runway lights
    superpoly_list rwy_lights; rwy_lights.clear();
    for ( i = 0; i < (int)runways.size(); ++i ) {
	gen_runway_lights( runways[i], alt_m, rwy_lights, &apt_base );
    }

    // 6th pass: generate all taxiway lights
    for ( i = 0; i < (int)taxiways.size(); ++i ) {
        gen_taxiway_lights( taxiways[i], alt_m, rwy_lights );
    }

    // write_polygon( accum, "accum" );
    // write_polygon( apt_base, "base" );
    // write_polygon( apt_clearing, "clear" );
    if ( apt_base.total_size() == 0 ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "no airport points generated");
	return;
    }

    // generate convex hull (no longer)
    // TGPolygon hull = convex_hull(apt_pts);

    TGPolygon filled_base = tgPolygonStripHoles( apt_base );
    // write_polygon( filled_base, "base" );
    TGPolygon divided_base = tgPolygonSplitLongEdges( filled_base, 200.0 );
    // write_polygon( divided_base, "divided-base" );
    TGPolygon base_poly = tgPolygonDiff( divided_base, accum );
    // write_polygon( base_poly, "base-raw" );

    char buf[120];  // For debugging output
    // Try to remove duplicated nodes and other degeneracies
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	SG_LOG(SG_GENERAL, SG_DEBUG, "add nodes/remove dups section = " << k
	       << " " << rwy_polys[k].get_material());
	TGPolygon poly = rwy_polys[k].get_poly();
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size before = " << poly.total_size());
	for ( i = 0; i < poly.contours(); ++i ) {
	    for ( j = 0; j < poly.contour_size(i); ++j ) {
		Point3D tmp = poly.get_pt(i, j);
		snprintf(buf, 119, "  %.7f %.7f %.7f\n", tmp.x(), tmp.y(), tmp.z() );
		SG_LOG(SG_GENERAL, SG_DEBUG, buf);
	    }
	}

	poly = remove_dups( poly );
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_dups() = "
	       << poly.total_size());

	for ( i = 0; i < poly.contours(); ++i ) {
	    for ( j = 0; j < poly.contour_size(i); ++j ) {
		Point3D tmp = poly.get_pt(i, j);
		snprintf(buf, 119, "    %.7f %.7f %.7f\n", tmp.x(), tmp.y(), tmp.z() );
		SG_LOG(SG_GENERAL, SG_DEBUG, buf);
	    }
	}

	poly = reduce_degeneracy( poly );
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after reduce_degeneracy() = "
	       << poly.total_size());

	for ( i = 0; i < poly.contours(); ++i ) {
	    for ( j = 0; j < poly.contour_size(i); ++j ) {
		Point3D tmp = poly.get_pt(i, j);
		snprintf(buf, 119, "    %.7f %.7f %.7f\n", tmp.x(), tmp.y(), tmp.z() );
		SG_LOG(SG_GENERAL, SG_DEBUG, buf);
	    }
	}

	rwy_polys[k].set_poly( poly );
    }

    // add segments to polygons to remove any possible "T"
    // intersections
    TGTriNodes tmp_nodes;

    // build temporary node list
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	TGPolygon poly = rwy_polys[k].get_poly();
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
    // the divided base could contain points not found in base_poly,
    // so we should add them because the skirt needs them.
    for ( i = 0; i < divided_base.contours(); ++i ) {
	for ( j = 0; j < divided_base.contour_size( i ); ++j ) {
	    tmp_nodes.unique_add( divided_base.get_pt(i, j) );
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
	TGPolygon poly = rwy_polys[k].get_poly();
	poly = add_nodes_to_poly( poly, tmp_nodes );
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.total_size());

#if 0
	char tmp[256];
	sprintf( tmp, "r%d", k );
	write_polygon( poly, tmp );
#endif

	rwy_polys[k].set_poly( poly );
    }

    // One more pass to try to get rid of other yukky stuff
    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	TGPolygon poly = rwy_polys[k].get_poly();

	SG_LOG(SG_GENERAL, SG_DEBUG, "total size of section " << k << " before =" << poly.total_size());

        poly = remove_dups( poly );
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_dups() = " << poly.total_size());
        poly = remove_bad_contours( poly );
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after remove_bad() = " << poly.total_size());

	rwy_polys[k].set_poly( poly );
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
    // write_polygon( base_poly, "base-fin" );
    SG_LOG(SG_GENERAL, SG_DEBUG, " after clean up: " << base_poly);

    // tesselate the polygons and prepair them for final output

    for ( i = 0; i < (int)rwy_polys.size(); ++i ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating section = " << i << " flag = " << rwy_polys[i].get_flag());

	TGPolygon poly = rwy_polys[i].get_poly();
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size before = " << poly.total_size());
	TGPolygon tri = polygon_tesselate_alt( poly );
	SG_LOG(SG_GENERAL, SG_DEBUG, "total size after = " << tri.total_size());

        TGPolygon tc;
        if ( rwy_polys[i].get_flag() == "taxi" ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "taxiway, no clip");
            tc = rwy_section_tex_coords( tri, texparams[i], false );
        } else {
            tc = rwy_section_tex_coords( tri, texparams[i], true );
        }

	rwy_polys[i].set_tris( tri );
	rwy_polys[i].set_texcoords( tc );
	rwy_polys[i].set_tri_mode( GL_TRIANGLES );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating base");
    TGPolygon base_tris = polygon_tesselate_alt( base_poly );

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

    Point3D tc;
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

    sgdVec3 tmp;
    sgdSetVec3( tmp, vnt.x(), vnt.y(), vnt.z() );
    sgdNormalizeVec3( tmp );

    Point3D vn( tmp[0], tmp[1], tmp[2] );
    SG_LOG(SG_GENERAL, SG_DEBUG, "found normal for this airport = " << tmp);

    for ( k = 0; k < (int)rwy_polys.size(); ++k ) {
	SG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
	// TGPolygon tri_poly = rwy_tris[k];
	TGPolygon tri_poly = rwy_polys[k].get_tris();
	TGPolygon tri_txs = rwy_polys[k].get_texcoords();
	string material = rwy_polys[k].get_material();
	SG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
	SG_LOG(SG_GENERAL, SG_DEBUG, "poly size = " << tri_poly.contours());
	SG_LOG(SG_GENERAL, SG_DEBUG, "texs size = " << tri_txs.contours());
	for ( i = 0; i < tri_poly.contours(); ++i ) {
	    tri_v.clear();
	    tri_n.clear();
	    tri_tc.clear();
	    for ( j = 0; j < tri_poly.contour_size(i); ++j ) {
		p = tri_poly.get_pt( i, j );
		index = nodes.unique_add( p );
		tri_v.push_back( index );

		// use 'the' normal
		index = normals.unique_add( vn );
		tri_n.push_back( index );

		tc = tri_txs.get_pt( i, j );
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
    point_list base_txs; 
    int_list base_tc;
    for ( i = 0; i < base_tris.contours(); ++i ) {
	tri_v.clear();
	tri_n.clear();
	tri_tc.clear();
	for ( j = 0; j < base_tris.contour_size(i); ++j ) {
	    p = base_tris.get_pt( i, j );
	    index = nodes.unique_add( p );
	    tri_v.push_back( index );

	    index = normals.unique_add( vn );
	    tri_n.push_back( index);
	}
	tris_v.push_back( tri_v );
	tris_n.push_back( tri_n );
	tri_materials.push_back( "Grass" );

	base_txs.clear();
	base_txs = sgCalcTexCoords( b, nodes.get_node_list(), tri_v );

	base_tc.clear();
	for ( j = 0; j < (int)base_txs.size(); ++j ) {
	    tc = base_txs[j];
	    // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
	    index = texcoords.simple_add( tc );
	    base_tc.push_back( index );
	}
	tris_tc.push_back( base_tc );
    }

    // on rare occasion, one or more of the divided base points can be
    // missed.  Make sure they are all in the node list so we can
    // build a proper skirt.

    for ( i = 0; i < divided_base.contours(); ++i ) {
	for ( j = 0; j < divided_base.contour_size( i ); ++j ) {
	    nodes.unique_add( divided_base.get_pt(i, j) );
	}
    }

    // Now that we have assembled all the airport geometry nodes into
    // a list, calculate an "average" airport elevation based on all
    // the actual airport node points.  This is more useful than
    // calculating an average over the entire airport surface because
    // it avoids biases introduced from the surrounding area if the
    // airport is located in a bowl or on a hill.

    double average = tgAverageElevation( root, elev_src,
                                         nodes.get_node_list() );
    // cout << "average airport elevation = " << average << endl;

    // Now build the fitted airport surface ...

    // calculation min/max coordinates of airport area
    Point3D min_deg(9999.0, 9999.0, 0), max_deg(-9999.0, -9999.0, 0);
    for ( j = 0; j < (int)nodes.get_node_list().size(); ++j ) {
        Point3D p = nodes.get_node_list()[j];
        if ( p.lon() < min_deg.lon() ) {
            min_deg.setlon( p.lon() );
        }
        if ( p.lon() > max_deg.lon() ) {
            max_deg.setlon( p.lon() );
        }
        if ( p.lat() < min_deg.lat() ) {
            min_deg.setlat( p.lat() );
        }
        if ( p.lat() > max_deg.lat() ) {
            max_deg.setlat( p.lat() );
        }
    }

    // extend the min/max coordinates of airport area to cover all
    // lights as well
    for ( i = 0; i < (int)rwy_lights.size(); ++i ) {
        for ( j = 0;
              j < (int)rwy_lights[i].get_poly().get_contour(0).size();
              ++j )
        {
            Point3D p = rwy_lights[i].get_poly().get_contour(0)[j];
            if ( p.lon() < min_deg.lon() ) {
                min_deg.setlon( p.lon() );
            }
            if ( p.lon() > max_deg.lon() ) {
                max_deg.setlon( p.lon() );
            }
            if ( p.lat() < min_deg.lat() ) {
                min_deg.setlat( p.lat() );
            }
            if ( p.lat() > max_deg.lat() ) {
                max_deg.setlat( p.lat() );
            }
        }
    }

    // Extend the area a bit so we don't have wierd things on the edges
    double dlon = max_deg.lon() - min_deg.lon();
    double dlat = max_deg.lat() - min_deg.lat();
    min_deg.setlon( min_deg.lon() - 0.01 * dlon );
    max_deg.setlon( max_deg.lon() + 0.01 * dlon );
    min_deg.setlat( min_deg.lat() - 0.01 * dlat );
    max_deg.setlat( max_deg.lat() + 0.01 * dlat );
    cout << "min = " << min_deg << " max = " << max_deg << endl;

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

    SG_LOG(SG_GENERAL, SG_INFO, "Computing beacon node elevations");
    point_list beacon_nodes = calc_elevations( apt_surf, beacons, 0.0 );
    SG_LOG(SG_GENERAL, SG_INFO, "Computing tower node elevations");
    point_list tower_nodes = calc_elevations( apt_surf, towers, 0.0 );
    SG_LOG(SG_GENERAL, SG_INFO, "Computing windsock node elevations");
    point_list windsock_nodes = calc_elevations( apt_surf, windsocks, 0.0 );

    // add base skirt (to hide potential cracks)
    //
    // this has to happen after we've calculated the node elevations
    // but before we convert to wgs84 coordinates
    int uindex, lindex;

    for ( i = 0; i < divided_base.contours(); ++i ) {
	strip_v.clear();
	strip_n.clear();
	strip_tc.clear();

	// prime the pump ...
	p = divided_base.get_pt( i, 0 );
	uindex = nodes.find( p );
	if ( uindex >= 0 ) {
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
	} else {
            string message = "Ooops missing node when building skirt (in init)";
            SG_LOG( SG_GENERAL, SG_INFO, message << " " << p );
	    throw sg_exception( message );
	}

	// loop through the list
	for ( j = 1; j < divided_base.contour_size(i); ++j ) {
	    p = divided_base.get_pt( i, j );
	    uindex = nodes.find( p );
	    if ( uindex >= 0 ) {
		Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
		SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
		lindex = nodes.simple_add( lower );
		geod_nodes.push_back( lower );
		strip_v.push_back( lindex );
		strip_v.push_back( uindex );

		index = normals.unique_add( vn );
		strip_n.push_back( index );
		strip_n.push_back( index );
	    } else {
                string message
                    = "Ooops missing node when building skirt (in loop)";
                SG_LOG( SG_GENERAL, SG_INFO, message << " " << p );
                throw sg_exception( message );
	    }
	}

	// close off the loop
	p = divided_base.get_pt( i, 0 );
	uindex = nodes.find( p );
	if ( uindex >= 0 ) {
	    Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
	    SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
	    lindex = nodes.simple_add( lower );
	    geod_nodes.push_back( lower );
	    strip_v.push_back( lindex );
	    strip_v.push_back( uindex );

	    index = normals.unique_add( vn );
	    strip_n.push_back( index );
	    strip_n.push_back( index );
	} else {
            string message = "Ooops missing node when building skirt (at end)";
            SG_LOG( SG_GENERAL, SG_INFO, message << " " << p );
            throw sg_exception( message );
	}

	strips_v.push_back( strip_v );
	strips_n.push_back( strip_n );
	strip_materials.push_back( "Grass" );

	base_txs.clear();
	base_txs = sgCalcTexCoords( b, nodes.get_node_list(), strip_v );

	base_tc.clear();
	for ( j = 0; j < (int)base_txs.size(); ++j ) {
	    tc = base_txs[j];
	    // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
	    index = texcoords.simple_add( tc );
	    base_tc.push_back( index );
	}
	strips_tc.push_back( base_tc );
    }

    // add light points

    superpoly_list tmp_light_list; tmp_light_list.clear();
    typedef map < string, double, less<string> > elev_map_type;
    typedef elev_map_type::const_iterator const_elev_map_iterator;
    elev_map_type elevation_map;

    SG_LOG(SG_GENERAL, SG_INFO,
	   "Computing runway/approach lighting elevations");

    // pass one, calculate raw elevations from Array

    for ( i = 0; i < (int)rwy_lights.size(); ++i ) {
        TGTriNodes light_nodes;
        light_nodes.clear();
        point_list lights_v = rwy_lights[i].get_poly().get_contour(0);
        for ( j = 0; j < (int)lights_v.size(); ++j ) {
            p = lights_v[j];
            index = light_nodes.simple_add( p );
        }

        // calculate light node elevations

        point_list geod_light_nodes
            = calc_elevations( apt_surf, light_nodes.get_node_list(), 0.5 );
        TGPolygon p;
        p.add_contour( geod_light_nodes, 0 );
        TGSuperPoly s;
        s.set_poly( p );
        tmp_light_list.push_back( s );

        string flag = rwy_lights[i].get_flag();
        if ( flag != (string)"" ) {
            double max = -9999;
            const_elev_map_iterator it = elevation_map.find( flag );
            if ( it != elevation_map.end() ) {
                max = elevation_map[flag];
            }
            for ( j = 0; j < (int)geod_light_nodes.size(); ++j ) {
                if ( geod_light_nodes[j].z() > max ) {
                    max = geod_light_nodes[j].z();
                }
            }
            elevation_map[flag] = max;
            SG_LOG( SG_GENERAL, SG_DEBUG, flag << " max = " << max );
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Done with lighting calc_elevations()");

    // pass two, for each light group check if we need to lift (based
    // on flag) and do so, then output next structures.
    for ( i = 0; i < (int)rwy_lights.size(); ++i ) {
        // tmp_light_list is a parallel structure to rwy_lights
        point_list geod_light_nodes
            = tmp_light_list[i].get_poly().get_contour(0);

#if 0
        // This code forces the elevation of all the approach lighting
        // components for a particular runway end up to the highest
        // max elevation for any of the points.  That can cause other
        // problem so let's nuke code this for the moment.
        string flag = rwy_lights[i].get_flag();
        if ( flag != (string)"" ) {
            const_elev_map_iterator it = elevation_map.find( flag );
            if ( it != elevation_map.end() ) {
                double force_elev = elevation_map[flag];
                for ( j = 0; j < (int)geod_light_nodes.size(); ++j ) {
                    geod_light_nodes[j].setz( force_elev );
                }
            }
        }
#endif
        
        // this is a little round about, but what we want to calculate the
        // light node elevations as ground + an offset so we do them
        // seperately, then we add them back into nodes to get the index
        // out, but also add them to geod_nodes to maintain consistancy
        // between these two lists.
        point_list light_normals = rwy_lights[i].get_normals().get_contour(0);
        pt_v.clear();
        pt_n.clear();
        for ( j = 0; j < (int)geod_light_nodes.size(); ++j ) {
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
    point_list wgs84_nodes;
    for ( i = 0; i < (int)geod_nodes.size(); ++i ) {
	p.setx( geod_nodes[i].x() * SGD_DEGREES_TO_RADIANS );
	p.sety( geod_nodes[i].y() * SGD_DEGREES_TO_RADIANS );
	p.setz( geod_nodes[i].z() );
        SG_LOG(SG_GENERAL, SG_DEBUG, "geod pt = " << geod_nodes[i] );
        Point3D cart = sgGeodToCart( p );
        SG_LOG(SG_GENERAL, SG_DEBUG, "  cart pt = " << cart );
	wgs84_nodes.push_back( cart );
    }
    float gbs_radius = sgCalcBoundingRadius( gbs_center, wgs84_nodes );
    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
    SG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center
           << " radius = " << gbs_radius );

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_n; fans_n.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    string objpath = root + "/AirportObj";
    string name = airport_id + ".btg";

    SGBinObject obj;

    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals.get_node_list() );
    obj.set_texcoords( texcoords.get_node_list() );
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
    if ( !result ) {
        throw sg_exception("error writing file. :-(");
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

    // write out airport object reference
    write_index( objpath, b, name );

    // write out beacon references
    for ( i = 0; i < (int)beacon_nodes.size(); ++i ) {
        write_index_shared( objpath, b, beacon_nodes[i],
                            "Models/Airport/beacon.xml",
                            0.0 );
    }

    // write out tower references
    for ( i = 0; i < (int)tower_nodes.size(); ++i ) {
        write_index_shared( objpath, b, tower_nodes[i],
                            "Models/Airport/tower.xml",
                            0.0 );
    }

    // write out windsock references
    for ( i = 0; i < (int)windsock_nodes.size(); ++i ) {
	if ( windsock_types[i] == 0 ) {
            write_index_shared( objpath, b, windsock_nodes[i],
                                "Models/Airport/windsock.xml",
                                0.0 );
	} else {
            write_index_shared( objpath, b, windsock_nodes[i],
                                "Models/Airport/windsock_lit.xml",
                                0.0 );
	}
    }

    string holepath = root + "/AirportArea";
    // long int poly_index = poly_index_next();
    // write_boundary( holepath, b, hull, poly_index );
    tgChopNormalPolygon( holepath, "Hole", divided_base, true );
    tgChopNormalPolygon( holepath, "Airport", apt_clearing, false );
    
}

