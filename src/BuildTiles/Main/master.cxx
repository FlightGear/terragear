// master.cxx -- top level construction routines
//
// Written by Curtis Olson, started May 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: master.cxx,v 1.9 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <stdlib.h>    // for system()
#include <sys/stat.h>  // for stat()
#ifndef _MSC_VER
#   include <unistd.h>    // for stat()
#endif

#include <string>
#include <iostream>

#include <simgear/bucket/newbucket.hxx>

// #include <Include/fg_constants.h>

// #include <Debug/logstream.hxx>
// #include <Array/array.hxx>
// #include <Clipper/clipper.hxx>
// #include <GenOutput/genobj.hxx>
// #include <Match/match.hxx>
// #include <Triangulate/triangle.hxx>

using std::string;
using std::cout;
using std::endl;


// return true if file exists
static bool file_exists( const string& file ) {
    struct stat buf;

    if ( stat( file.c_str(), &buf ) == 0 ) {
	return true;
    } else {
	return false;
    }
}


// check if the specified tile has data defined for it
static bool has_data( const string& path, const SGBucket& b ) {
    string array_file = path + "DEM-3/" + b.gen_base_path()
	+ "/" + b.gen_index_str() + ".arr";
    if ( file_exists( array_file ) ) {
	return true;
    }

    array_file += ".gz";
    if ( file_exists( array_file ) ) {
	return true;
    }

    return false;
}


// build the tile and check for success
bool build_tile( const string& work_base, const string& output_base, 
		 const SGBucket& b ) {
    string result_file = "result";

    string command = "./construct " + work_base + " " + output_base + " "
	+ b.gen_index_str() + " > " + result_file + " 2>&1";
    cout << command << endl;

    system( command.c_str() );

    FILE *fp = fopen( result_file.c_str(), "r" );
    char line[256];
    while ( fgets( line, 256, fp ) != NULL ) {
	string line_str = line;
	line_str = line_str.substr(0, line_str.length() - 1);
 	cout << line_str << endl;
 	if ( line_str == "[Finished successfully]" ) {
	    fclose(fp);
	    return true;
	}
    }

    fclose(fp);
    return false;
}


// display usage and exit
void usage( const string name ) {
    cout << "Usage: " << name << " <work_base> <output_base>" << endl;
    exit(-1);
}


int main(int argc, char **argv) {
    double lon, lat;

    if ( argc < 3 ) {
	usage( argv[0] );
    }

    string work_base = argv[1];
    string output_base = argv[2];

    SGBucket tmp1( 0.0, 0.0 );

    double dy = tmp1.get_height();
    lat = 68.75 + dy * 0.5;

    bool start_lon = true;
    lon = -152.9 /* + (1.0/16.0) */;

    while ( lat <= 90.0 ) {
	SGBucket tmp2( 0.0, lat );
	double dx = tmp2.get_width();

	if ( ! start_lon ) {
	    lon = -180 + dx * 0.5;
	} else {
	    start_lon = false;
	}
	while ( lon <= 180.0 ) {
	    SGBucket b( lon, lat );
	    cout << "Bucket = " << b << endl;

	    if ( has_data( work_base, b ) ) {
		if ( ! build_tile( work_base, output_base, b ) ) {
		    cout << "error building last tile" << endl;
		    exit(-1);
		}
	    }

	    lon += dx;
	}


	lat += dy;
    }

    return 0;
}


