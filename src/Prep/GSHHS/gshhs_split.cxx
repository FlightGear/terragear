// gshhs_split.cxx -- split a gshhs polygon
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

#include <Polygon/index.hxx>
#include <Polygon/names.hxx>
#include <Polygon/polygon.hxx>
#include <Polygon/split.hxx>
#include <Polygon/simple_clip.hxx>

#include "gshhs_split.hxx"

#if !defined (SG_HAVE_NATIVE_SGI_COMPILERS)
SG_USING_STD(cout);
#endif
SG_USING_STD(string);


// process shape front end ... split shape into lon = -180 ... 180,
// -360 ... -180, and 180 ... 360 ... shift the offset sections and
// process each separately
void split_and_shift_chunk( const string& path, AreaType area, 
			    const FGPolygon& shape )
{
    FGPolygon lower_mask, center_mask, upper_mask;
    FGPolygon lower_shape, center_shape, upper_shape;

    lower_mask.erase();
    lower_mask.add_node( 0, Point3D(-360, -90, 0) );
    lower_mask.add_node( 0, Point3D(-180, -90, 0) );
    lower_mask.add_node( 0, Point3D(-180, 90, 0) );
    lower_mask.add_node( 0, Point3D(-360, 90, 0) );

    center_mask.erase();
    center_mask.add_node( 0, Point3D(-180, -90, 0) );
    center_mask.add_node( 0, Point3D(180, -90, 0) );
    center_mask.add_node( 0, Point3D(180, 90, 0) );
    center_mask.add_node( 0, Point3D(-180, 90, 0) );

    upper_mask.erase();
    upper_mask.add_node( 0, Point3D(180, -90, 0) );
    upper_mask.add_node( 0, Point3D(360, -90, 0) );
    upper_mask.add_node( 0, Point3D(360, 90, 0) );
    upper_mask.add_node( 0, Point3D(180, 90, 0) );

    lower_shape.erase();
    center_shape.erase();
    upper_shape.erase();

    SG_LOG ( SG_GENERAL, SG_INFO, "Clipping lower shape" );
    lower_shape = polygon_int( lower_mask, shape );
    lower_shape.shift( 360, 0 );

    SG_LOG ( SG_GENERAL, SG_INFO, "Clipping center shape" );
    center_shape = polygon_int( center_mask, shape );

    upper_shape = polygon_int( upper_mask, shape );
    SG_LOG ( SG_GENERAL, SG_INFO, "Clipping upper shape" );
    upper_shape.shift( -360, 0 );

    SG_LOG ( SG_GENERAL, SG_INFO, "Processing lower shape" );
    split_polygon(path, area, lower_shape);

    SG_LOG ( SG_GENERAL, SG_INFO, "Processing center shape" );
    split_polygon(path, area, center_shape);

    SG_LOG ( SG_GENERAL, SG_INFO, "Processing upper shape" );
    split_polygon(path, area, upper_shape);
}


// process a large shape through my crude polygon splitter to reduce
// the polygon sizes before handing off to gpc
void gshhs_split_polygon( const string& path, AreaType area, FGPolygon& shape, 
			  const double min, const double max )
{
    double base_line = (int)(min - 1.0);
    double line = base_line;
    int count = 0;
    cout << "min = " << min << endl;

    while ( line < max ) {
	printf("clipping at %.10f\n", line);
		
	FGPolygon above = horizontal_clip( shape, line, Above );
	FGPolygon below = horizontal_clip( shape, line, Below );

// #define WRITE_FILE
#ifdef WRITE_FILE
	for ( int i = 0; i < above.contours(); ++i ) {
	    char name[256];
	    sprintf(name, "junk%d", i);
	    fp = fopen(name, "w");
	    for ( int j = 0; j < above.contour_size( i ); ++j ) {
		fprintf( fp, "%.6f %.6f\n", 
			 above.get_pt( i, j ).x(), 
			 above.get_pt( i, j ).y() );
	    }
	    fprintf( fp, "%.4f %.4f\n", 
		     above.get_pt( i, 0 ).x(), 
		     above.get_pt( i, 0 ).y() );
		    fclose(fp);
	}
#endif

#if 0
	if ( above.contours() > 1 ) {
	    cout << endl;
	    cout << "multi-part clip result" << endl;
	    cout << endl;
	    sleep(5);
	}
#endif

	// cout << "exiting early" << endl;
	// exit(0);

	split_and_shift_chunk(path, area, below);

	shape = above;

	++count;
	line = base_line + (count * 1.0 / 8.0);
    }
}
