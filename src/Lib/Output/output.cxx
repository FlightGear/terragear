// output.cxx -- routines to output a polygon model of an airport
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999 - 2000  Curtis L. Olson  - curt@flightgear.org
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
#include <time.h>
#include <zlib.h>

#include <list>
#include STL_STRING

#include <simgear/bucket/newbucket.hxx>

#include <Polygon/polygon.hxx>

#include "output.hxx"

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

SG_USING_STD( cout );
SG_USING_STD( endl );


void write_polygon( const TGPolygon& poly, const string& base ) {
    for ( int i = 0; i < poly.contours(); ++i ) {
	char name[256];
	sprintf(name, "%s%d", base.c_str(), i );
	FILE *fp = fopen( name, "w" );

	fprintf(fp, "hole = %d\n", poly.get_hole_flag(i));
	for ( int j = 0; j < poly.contour_size( i ); ++j ) {
	    Point3D p0 = poly.get_pt(i, j);
	    fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	}
	Point3D p0 = poly.get_pt(i, 0);
	fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	fclose(fp);
    }
}


// update index
void write_index(const string& base, const SGBucket& b, const string& name) {
    string dir = base + "/" + b.gen_base_path();
#ifdef _MSC_VER
    fg_mkdir( dir.c_str() );
#else
    string command = "mkdir -p " + dir;
    system(command.c_str());
#endif

    string file = dir + "/" + b.gen_index_str() + ".ind";
    // string file = dir + "/" + name;
    cout << "Output file = " << file << endl;

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "a" )) == NULL ) {
	cout << "ERROR: opening " << file << " for writing!" << endl;
	exit(-1);
    }

    fprintf( fp, "OBJECT %s\n", name.c_str() );
    fclose( fp );
}


void write_boundary( const string& base, const SGBucket& b, 
		     const TGPolygon& bounds, long int p_index )
{
    Point3D p;

    string dir = base + "/" + b.gen_base_path();
#ifdef _MSC_VER
    fg_mkdir( dir.c_str() );
#else
    string command = "mkdir -p " + dir;
    system(command.c_str());
#endif

    string file = dir + "/" + b.gen_index_str();

    char poly_index[256];
    sprintf( poly_index, "%ld", p_index );
    file += ".";
    file += poly_index;

    cout << "Output file = " << file << endl;

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "w" )) == NULL ) {
	cout << "ERROR: opening " << file << " for writing!" << endl;
	exit(-1);
    }

    fprintf( fp, "Hole\n" );

    fprintf( fp, "%d\n", bounds.contours() );
    for ( int i = 0; i < bounds.contours(); ++i ) {
	fprintf( fp, "%d\n", bounds.contour_size(i) );
	fprintf( fp, "%d\n", bounds.get_hole_flag(i) );
	for ( int j = 0; j < bounds.contour_size(i); ++j ) {
	    p = bounds.get_pt( i, j );
	    fprintf( fp, "%.15f  %.15f\n", p.x(), p.y() );
	}
    }
    fclose( fp );
}
