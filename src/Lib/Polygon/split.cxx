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

#ifdef _MSC_VER
#  include <win32/mkdir.hpp>
#endif

#include "index.hxx"
#include "names.hxx"
#include "split.hxx"

SG_USING_STD(cout);


static void clip_and_write_poly( string root, long int p_index, AreaType area, 
				 FGBucket b, const FGPolygon& shape ) {
    Point3D c, min, max, p;
    c = Point3D( b.get_center_lon(), b.get_center_lat(), 0 );
    double span = bucket_span( c.y() );
    FGPolygon base, result;
    char tile_name[256], poly_index[256];

    // calculate bucket dimensions
    if ( (c.y() >= -89.0) && (c.y() < 89.0) ) {
	min.setx( c.x() - span / 2.0 );
	max.setx( c.x() + span / 2.0 );
	min.sety( c.y() - FG_HALF_BUCKET_SPAN );
	max.sety( c.y() + FG_HALF_BUCKET_SPAN );
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
	FG_LOG ( FG_GENERAL, FG_ALERT, 
		 "Out of range latitude in clip_and_write_poly() = " << c.y() );
    }

    FG_LOG( FG_GENERAL, FG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.add_node( 0, Point3D(min.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), max.y(), 0) );
    base.add_node( 0, Point3D(min.x(), max.y(), 0) );

    // FG_LOG( FG_GENERAL, FG_DEBUG, "base = 4 vertices" );

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
	if ( poly_type == "Unknown" ) {
	    cout << "unknown area type in clip_and_write_poly()!" << endl;
	    exit(-1);
	}
	
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

    FG_LOG( FG_GENERAL, FG_INFO, "  min = " << min << " max = " << max );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    FGBucket b_min( min.x(), min.y() );
    FGBucket b_max( max.x(), max.y() );
    FG_LOG( FG_GENERAL, FG_INFO, "  Bucket min = " << b_min );
    FG_LOG( FG_GENERAL, FG_INFO, "  Bucket max = " << b_max );
	    
    if ( b_min == b_max ) {
	clip_and_write_poly( path, index, area, b_min, shape );
    } else {
	FGBucket b_cur;
	int dx, dy, i, j;
	    
	fgBucketDiff(b_min, b_max, &dx, &dy);
	FG_LOG( FG_GENERAL, FG_INFO, 
		"  polygon spans tile boundaries" );
	FG_LOG( FG_GENERAL, FG_INFO, "  dx = " << dx 
		<< "  dy = " << dy );

	if ( (dx > 2880) || (dy > 1440) ) {
	    FG_LOG( FG_GENERAL, FG_ALERT, 
		    "something is really wrong in split_polygon()!!!!" );
	    exit(-1);
	}

	for ( j = 0; j <= dy; j++ ) {
	    // for performance reasons, we'll clip out just this
	    // horizontal row, and clip all the tiles in this row
	    // against the smaller shape

	    FG_LOG ( FG_GENERAL, FG_INFO, 
		     "Generating clip row " << j << " of " << dy );
		
	    FGBucket b_clip = fgBucketOffset(min.x(), min.y(), 0, j);
	    FGPolygon row, clip_row;
	    Point3D c, clip_max, clip_min;
	    c = Point3D( b_clip.get_center_lon(), b_clip.get_center_lat(), 0 );

	    row.erase();
	    clip_row.erase();

	    // calculate bucket clip_min.y and clip_max.y
	    if ( (c.y() >= -89.0) && (c.y() < 89.0) ) {
		clip_min.sety( c.y() - FG_HALF_BUCKET_SPAN );
		clip_max.sety( c.y() + FG_HALF_BUCKET_SPAN );
	    } else if ( c.y() < -89.0) {
		clip_min.sety( -90.0 );
		clip_max.sety( -89.0 );
	    } else if ( c.y() >= 89.0) {
		clip_min.sety( 89.0 );
		clip_max.sety( 90.0 );
	    } else {
		FG_LOG ( FG_GENERAL, FG_ALERT, 
			 "Out of range latitude in clip_and_write_poly() = " 
			 << c.y() );
	    }
	    clip_min.setx( -180.0 );
	    clip_max.setx( 180.0 );
    
	    // set up clipping tile
	    row.add_node( 0, Point3D(clip_min.x(), clip_min.y(), 0) );
	    row.add_node( 0, Point3D(clip_max.x(), clip_min.y(), 0) );
	    row.add_node( 0, Point3D(clip_max.x(), clip_max.y(), 0) );
	    row.add_node( 0, Point3D(clip_min.x(), clip_max.y(), 0) );

	    clip_row = polygon_int( row, shape );

	    /* FILE *sfp = fopen("shape", "w");
	    gpc_write_polygon(sfp, 0, shape);
	    fclose(sfp);
	    sfp = fopen("clip_row", "w");
	    gpc_write_polygon(sfp, 0, &clip_row);
	    fclose(sfp); */

	    for ( i = 0; i <= dx; i++ ) {
		b_cur = fgBucketOffset(min.x(), min.y(), i, j);
		clip_and_write_poly( path, index, area, b_cur, clip_row );
	    }
	    cout << "  (done)" << endl;
	}
	// string answer; cin >> answer;
    }
}
