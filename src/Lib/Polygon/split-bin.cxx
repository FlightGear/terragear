// split.cxx -- polygon splitting utils
//
// Written by Curtis Olson, started February 1999.
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


#include <simgear/compiler.h>

#include STL_STRING

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/exception.hxx>

#ifdef _MSC_VER
#  include <win32/mkdir.hpp>
#endif

#include "index.hxx"
#include "names.hxx"
#include "simple_clip.hxx"
#include "split.hxx"


static void clip_and_write_poly( string root, long int p_index, AreaType area, 
				 SGBucket b, const FGPolygon& shape ) {
    Point3D c, min, max, p;
    c = Point3D( b.get_center_lon(), b.get_center_lat(), 0 );
    double span = sg_bucket_span( c.y() );
    FGPolygon base, result;
    char tile_name[256], poly_index[256];

    // calculate bucket dimensions
    if ( (c.y() >= -89.0) && (c.y() < 89.0) ) {
	min.setx( c.x() - span / 2.0 );
	max.setx( c.x() + span / 2.0 );
	min.sety( c.y() - SG_HALF_BUCKET_SPAN );
	max.sety( c.y() + SG_HALF_BUCKET_SPAN );
    } else if ( c.y() < -89.0) {
	min.setx( -90.0 );
	max.setx( -89.0 );
	min.sety( -180.0 );
	max.sety( 180.0 );
    } else if ( c.y() >= 89.0) {
	min.setx( 89.0 );
	max.setx( 90.0 );
	min.sety( -180.0 );
	max.sety( 180.0 );
    } else {
	SG_LOG ( SG_GENERAL, SG_ALERT, 
		 "Out of range latitude in clip_and_write_poly() = " << c.y() );
    }

    SG_LOG( SG_GENERAL, SG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.add_node( 0, Point3D(min.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), max.y(), 0) );
    base.add_node( 0, Point3D(min.x(), max.y(), 0) );

    // SG_LOG( SG_GENERAL, SG_DEBUG, "base = 4 vertices" );

    /*
    FILE *bfp= fopen("base", "w");
    gpc_write_polygon(bfp, &base);
    fclose(bfp);
    */

    result = polygon_int( base, shape );

    if ( result.contours() > 0 ) {
	long int t_index = b.gen_index();
	string path = root + "/" + b.gen_base_path();

#ifdef _MSC_VER
	fg_mkdir( path.c_str() );
#else
	string command = "mkdir -p " + path;
	system( command.c_str() );
#endif

	sprintf( tile_name, "%ld", t_index );
	string polyfile = path + "/" + tile_name;

	sprintf( poly_index, "%ld", p_index );
	polyfile += ".";
	polyfile += poly_index;

	string poly_type = get_area_name( area );
	if ( poly_type == "Unknown" )
	    throw sg_exception("unknown area type in clip_and_write_poly()!");
	
	FILE *rfp= fopen( polyfile.c_str(), "w" );
	fprintf( rfp, "%s\n", poly_type.c_str() );

	fprintf( rfp, "%d\n", result.contours() );
	for ( int i = 0; i < result.contours(); ++i ) {
	    fprintf( rfp, "%d\n", result.contour_size(i) );
	    fprintf( rfp, "%d\n", result.get_hole_flag(i) );
	    for ( int j = 0; j < result.contour_size(i); ++j ) {
		p = result.get_pt( i, j );
		fprintf( rfp, "%.15f  %.15f\n", p.x(), p.y() );
	    }
	}
	fclose( rfp );
    }
}


// process shape (write polygon to all intersecting tiles)
void split_polygon(const string& path, AreaType area, const FGPolygon& shape) {
    Point3D min, max, p;
    // point2d min, max;
    long int index;
    int i, j;

    // bail out immediately if polygon is empty
    if ( shape.contours() == 0 ) {
	return;
    }

    min = Point3D(  200.0 );
    max = Point3D( -200.0 );

    // find min/max of polygon
    for ( i = 0; i < shape.contours(); i++ ) {
	for ( j = 0; j < shape.contour_size(i); j++ ) {
	    p = shape.get_pt( i, j );

	    if ( p.x() < min.x() ) { min.setx( p.x() ); }
	    if ( p.y() < min.y() ) { min.sety( p.y() ); }
	    if ( p.x() > max.x() ) { max.setx( p.x() ); }
	    if ( p.y() > max.y() ) { max.sety( p.y() ); }
	}
    }

    // get next polygon index
    index = poly_index_next();

    SG_LOG( SG_GENERAL, SG_INFO, "  min = " << min << " max = " << max );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( min.x(), min.y() );
    SGBucket b_max( max.x(), max.y() );
    SG_LOG( SG_GENERAL, SG_INFO, "  Bucket min = " << b_min );
    SG_LOG( SG_GENERAL, SG_INFO, "  Bucket max = " << b_max );

    if ( b_min == b_max ) {
	// shape entirely contained in a single bucket, write and bail
	clip_and_write_poly( path, index, area, b_min, shape );
	return;
    }

    SGBucket b_cur;
    int dx, dy;
	    
    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_INFO, 
	    "  polygon spans tile boundaries" );
    SG_LOG( SG_GENERAL, SG_INFO, "  dx = " << dx 
	    << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy <= 1 ) {
	// we are down to at most two rows, write each column and then
	// bail
	for ( j = 0; j <= 1; ++j ) {
	    for ( i = 0; i <= dx; ++i ) {
	        b_cur = sgBucketOffset(min.x(), min.y(), i, j);
	        clip_and_write_poly( path, index, area, b_cur, shape );
	    }
	}
	return;
    }

    // we have more than one row left, split in half and recurse with
    // each chunk

    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset(min.x(), min.y(), 0, mid);
    double clip_line = b_clip.get_center_lat();
    if ( (clip_line >= -89.0) && (clip_line < 89.0) ) {
	clip_line += SG_HALF_BUCKET_SPAN;
    } else if ( clip_line < -89.0 ) {
	clip_line = -89.0;
    } else if ( clip_line >= 89.0 ) {
	clip_line = 90.0;
    } else {
	SG_LOG ( SG_GENERAL, SG_ALERT, 
		 "Out of range latitude in clip_and_write_poly() = " 
		 << clip_line );
    }

    {
	//
	// Crop bottom area (hopefully by putting this in it's own
	// scope we can shorten the life of some really large data
	// structures to reduce memory use)
	//

	SG_LOG ( SG_GENERAL, SG_INFO, 
		 "Generating bottom half (" << min.y() << "-" <<
		 clip_line << ")" );

	FGPolygon bottom, bottom_clip;
	if ( shape.total_size() < 50000 ) {
	    bottom.erase();
	    bottom_clip.erase();

	    bottom.add_node( 0, Point3D(-180.0, min.y(), 0) );
	    bottom.add_node( 0, Point3D(180.0, min.y(), 0) );
	    bottom.add_node( 0, Point3D(180.0, clip_line, 0) );
	    bottom.add_node( 0, Point3D(-180.0, clip_line, 0) );

	    bottom_clip = polygon_int( bottom, shape );
	} else {
	    bottom_clip = horizontal_clip( shape, clip_line, Below );
	}

	split_polygon(path, area, bottom_clip);
    }

    {
	//
	// Crop top area (hopefully by putting this in it's own scope
	// we can shorten the life of some really large data
	// structures to reduce memory use)
	//

	SG_LOG ( SG_GENERAL, SG_INFO, 
		 "Generating top half (" << clip_line << "-" <<
		 max.y() << ")" );

	FGPolygon top, top_clip;
	if ( shape.total_size() < 50000 ) {
	    top.erase();
	    top_clip.erase();

	    top.add_node( 0, Point3D(-180.0, clip_line, 0) );
	    top.add_node( 0, Point3D(180.0, clip_line, 0) );
	    top.add_node( 0, Point3D(180.0, max.y(), 0) );
	    top.add_node( 0, Point3D(-180.0, max.y(), 0) );

	    top_clip = polygon_int( top, shape );
	} else {
	    top_clip = horizontal_clip( shape, clip_line, Above );
	}

	split_polygon(path, area, top_clip);
    }
}
