// main.cxx -- process shapefiles and extract polygon outlines,
//             clipping against and sorting them into the revelant
//             tiles.
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
 

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include STL_STRING

#include <simgear/debug/logstream.hxx>

#ifdef HAVE_ZLIB
#  include <zlib.h>
#else
#  include <simgear/zlib/zlib.h>
#endif

#include <Polygon/index.hxx>
#include <Polygon/names.hxx>
#include <Polygon/polygon.hxx>

#include "gshhs.h"


// hackity, hackity, hack ... cough cough
#ifdef i386
#  define FLIP
#endif

SG_USING_STD(cout);
SG_USING_STD(cin);

// return the type of the shapefile record
AreaType get_shapefile_type(int rec) {
    string area;

    return get_area_type( area );
}


// write result to unique file name
void write_result( const FGPolygon& result ) {
    static int count = 0;
    char fname[256];
    FILE *fp;

    cout << "writing " << result.contours() << " contours" << endl;

    for ( int i = 0; i < (int)result.contours(); ++i ) {
	sprintf( fname, "junk%d", count++ );
	fp = fopen(fname, "w");
	for ( int j = 0; j < (int)result.contour_size(i); ++j ) {
	    fprintf(fp,
		    "%.6f %.6f\n", 
		    result.get_pt( i, j ).x(), 
		    result.get_pt( i, j ).y() );
	}
	fprintf(fp,
		"%.6f %.6f\n", 
		result.get_pt( i, 0 ).x(), 
		result.get_pt( i, 0 ).y() );
	fclose(fp);
    }

    if ( (int)result.contours() > 0 ) {
	cout << "press return to continue: ";

	char answer;
	cin >> answer;
    }
}


// process shape front end ... split shape into lon = -180 ... 180,
// -360 ... -180, and 180 ... 360 ... shift the offset sections and
// process each separately
void gen_clipped_polygon( const FGPolygon& shape, const FGPolygon& clip ) {
    FGPolygon lower_mask, center_mask, upper_mask;
    FGPolygon lower_shape, center_shape, upper_shape;
    FGPolygon result;

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
    result = polygon_int( lower_shape, clip );
    write_result( result );

    SG_LOG ( SG_GENERAL, SG_INFO, "Clipping center shape" );
    center_shape = polygon_int( center_mask, shape );
    result = polygon_int( center_shape, clip );
    write_result( result );

    upper_shape = polygon_int( upper_mask, shape );
    SG_LOG ( SG_GENERAL, SG_INFO, "Clipping upper shape" );
    upper_shape.shift( -360, 0 );
    result = polygon_int( upper_shape, clip );
    write_result( result );
}


int main( int argc, char **argv ) {
    struct GSHHS h;
    struct POINT p;
    FGPolygon shape, clip;
    double w, e, s, n, area, lon, lat;
    int k, max_east = 270000000;
    char source;

    sglog().setLogLevels( SG_ALL, SG_DEBUG );

    if ( argc < 2 ) {
	SG_LOG( SG_GENERAL, SG_ALERT, "Usage: " << argv[0] 
		<< " <gshhs_file>" );
	exit(-1);
    }

    gzFile fp;
    if ( (fp = gzopen (argv[1], "rb")) == NULL ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << argv[1] );
        exit(-1);
    }

    SG_LOG( SG_GENERAL, SG_DEBUG, "Opening " << argv[1] << " for reading." );

    while ( gzread(fp, (void *)&h, (unsigned)sizeof(struct GSHHS)) == 
	    (unsigned)sizeof(struct GSHHS) )
    {

	shape.erase();

#ifdef FLIP
	h.id = swabi4 ((unsigned int)h.id);
	h.n = swabi4 ((unsigned int)h.n);
	h.level = swabi4 ((unsigned int)h.level);
	h.west = swabi4 ((unsigned int)h.west);
	h.east = swabi4 ((unsigned int)h.east);
	h.south = swabi4 ((unsigned int)h.south);
	h.north = swabi4 ((unsigned int)h.north);
	h.area = swabi4 ((unsigned int)h.area);
	h.greenwich = swabi2 ((unsigned int)h.greenwich);
	h.source = swabi2 ((unsigned int)h.source);
#endif

	w = h.west  * 1.0e-6;
	e = h.east  * 1.0e-6;
	s = h.south * 1.0e-6;
	n = h.north * 1.0e-6;
	source = (h.source == 1) ? 'W' : 'C';
	area = 0.1 * h.area;

	printf ("P %6d%8d%2d%2c%13.3f%10.5f%10.5f%10.5f%10.5f\n", h.id, h.n, h.level, source, area, w, e, s, n);

	cout << "Loading shape" << endl;
	for ( k = 0; k < h.n; k++ ) {
	    if ( gzread(fp, (void *)&p, (unsigned)sizeof(struct POINT)) !=
		 (unsigned)sizeof(struct POINT) )
	    {
		SG_LOG( SG_GENERAL, SG_ALERT, "Error reading file for polygon "
			<< h.id << " point " << k );
		exit(-1);
	    }
#ifdef FLIP
	    p.x = swabi4 ((unsigned int)p.x);
	    p.y = swabi4 ((unsigned int)p.y);
#endif
	    lon = (h.greenwich && p.x > max_east) ? 
		p.x * 1.0e-6 - 360.0 : 
		p.x * 1.0e-6;
	    lat = p.y * 1.0e-6;
	    // printf ("%10.5lf%10.5lf\n", lon, lat);
	    shape.add_node( 0, Point3D(lon, lat, 0) );
	}
	max_east = 180000000;	/* Only Eurasiafrica needs 270 */

	SG_LOG( SG_GENERAL, SG_INFO, "  record = " << h.id << " size = " <<
		shape.contour_size( 0 ) );

	if ( h.id > -1 ) {
	    continue;
	}

	if ( h.level == 1 ) {
	    cout << "dumping shape to file" << endl;

	    FILE *fp = fopen("junk", "w");
	    for ( int i = 0; i < shape.contour_size( 0 ); ++i ) {
		fprintf( fp, "%.6f %.6f\n", 
			 shape.get_pt( 0, i ).x(), shape.get_pt( 0, i ).y() );
	    }
	    fclose(fp);

	    double lon = -122.309313;
	    double lat = 47.448982;
	    double width = 1.5;
	    double height = 1.0;

	    clip.erase();
	    clip.add_node( 0, Point3D(lon - width, lat - height, 0) );
	    clip.add_node( 0, Point3D(lon + width, lat - height, 0) );
	    clip.add_node( 0, Point3D(lon + width, lat + height, 0) );
	    clip.add_node( 0, Point3D(lon - width, lat + height, 0) );
	
	    gen_clipped_polygon( shape, clip );
	}
    }
    return 0;
}


