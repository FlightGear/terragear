// main.cxx -- main loop
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998  Curtis L. Olson  - curt@me.umn.edu
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

#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif

#include <list>
#include <stdio.h>
#include <string.h>
#include STL_STRING

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/fgstream.hxx>

#include <Polygon/index.hxx>

#include "build.hxx"
#include "convex_hull.hxx"


int nudge = 10;


// reads the apt_full file and extracts and processes the individual
// airport records
int main( int argc, char **argv ) {
    string_list runways_list;
    string airport, last_airport;
    string line;
    char tmp[256];

    fglog().setLogLevels( FG_ALL, FG_DEBUG );

    // parse arguments
    string work_dir = "";
    string input_file;
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        string arg = argv[arg_pos];
        if ( arg.find("--work=") == 0 ) {
            work_dir = arg.substr(7);
	} else if ( arg.find("--input=") == 0 ) {
	    input_file = arg.substr(8);
 	} else if ( arg.find("--nudge=") == 0 ) {
	    nudge = atoi( arg.substr(8).c_str() );
	} else {
	    FG_LOG( FG_GENERAL, FG_ALERT, 
		    "Usage " << argv[0] << " --input=<apt_file> "
		    << "--work=<work_dir> [ --nudge=n ]" );
	    exit(-1);
	}
    }

    cout << "Input file = " << input_file << endl;
    cout << "Work directory = " << work_dir << endl;
    cout << "Nudge = " << nudge << endl;

    if ( work_dir == "" ) {
	FG_LOG( FG_GENERAL, FG_ALERT, 
		"Error: no work directory specified." );
	exit(-1);
    }

    if ( input_file == "" ) {
	FG_LOG( FG_GENERAL, FG_ALERT, 
		"Error: no input file." );
	exit(-1);
    }

    // make work directory
    string command = "mkdir -p " + work_dir;
    system( command.c_str() );

    // initialize persistant polygon counter
    string counter_file = work_dir + "/poly_counter";
    poly_index_init( counter_file );

    fg_gzifstream in( input_file );
    if ( !in ) {
        FG_LOG( FG_GENERAL, FG_ALERT, "Cannot open file: " << input_file );
        exit(-1);
    }

    // throw away the first line
    in.getline(tmp, 256);

    last_airport = "";

    while ( ! in.eof() ) {
	in.getline(tmp, 256);
	line = tmp;
	// cout << line << endl;

	if ( line.length() == 0 ) {
	    // empty, skip
	} else if ( line[0] == '#' ) {
	    // comment, skip
	} else if ( (line[0] == 'A') || (line[0] == 'H') || (line[0] == 'S') ) {
	    // start of airport record
	    airport = line;

	    if ( last_airport.length() ) {
		// process previous record
		// process_airport(last_airport, runways_list, argv[2]);
		build_airport(last_airport, runways_list, work_dir);
	    }

	    // clear runway list for start of next airport
	    runways_list.clear();

	    last_airport = airport;
	} else if ( line[0] == 'R' ) {
	    // runway entry
	    runways_list.push_back(line);
	} else if ( line == "[End]" ) {
	    // end of file
	    break;
	} else {
	    FG_LOG( FG_GENERAL, FG_ALERT, 
		    "Unknown line in file" << endl << line );
	    exit(-1);
	}
    }

    if ( last_airport.length() ) {
	// process previous record
	// process_airport(last_airport, runways_list, argv[2]);
	build_airport(last_airport, runways_list, work_dir);
    }

    return 0;
}


