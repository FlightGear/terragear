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
#include <simgear/structure/exception.hxx>

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
#include <simgear/misc/sgstream.hxx>

#include <Polygon/index.hxx>
#include <Geometry/util.hxx>

#include "build.hxx"
#include "convex_hull.hxx"

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

int nudge = 10;


// Display usage
static void usage( int argc, char **argv ) {
    SG_LOG(SG_GENERAL, SG_ALERT, 
	   "Usage " << argv[0] << " --input=<apt_file> "
	   << "--work=<work_dir> [ --start-id=abcd ] [ --nudge=n ]"
	   << "[--min-lon=<deg>] [--max-lon=<deg>] [--min-lat=<deg>] [--max-lat=<deg>]"
           << "[--chunk=<chunk>]");
}


// reads the apt_full file and extracts and processes the individual
// airport records
int main( int argc, char **argv ) {
    float min_lon = -180;
    float max_lon = 180;
    float min_lat = -90;
    float max_lat = 90;
    bool ready_to_go = true;

    string_list elev_src;
    elev_src.clear();

    sglog().setLogLevels( SG_GENERAL, SG_INFO );

    // parse arguments
    string work_dir = "";
    string input_file = "";
    string start_id = "";
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        string arg = argv[arg_pos];
        if ( arg.find("--work=") == 0 ) {
            work_dir = arg.substr(7);
	} else if ( arg.find("--input=") == 0 ) {
	    input_file = arg.substr(8);
        } else if ( arg.find("--terrain=") == 0 ) {
            elev_src.push_back( arg.substr(10) );
 	} else if ( arg.find("--start-id=") == 0 ) {
	    start_id = arg.substr(11);
	    ready_to_go = false;
 	} else if ( arg.find("--nudge=") == 0 ) {
	    nudge = atoi( arg.substr(8).c_str() );
	} else if ( arg.find("--min-lon=") == 0 ) {
	    min_lon = atof( arg.substr(10).c_str() );
	} else if ( arg.find("--max-lon=") == 0 ) {
	    max_lon = atof( arg.substr(10).c_str() );
	} else if ( arg.find("--min-lat=") == 0 ) {
	    min_lat = atof( arg.substr(10).c_str() );
	} else if ( arg.find("--max-lat=") == 0 ) {
	    max_lat = atof( arg.substr(10).c_str() );
        } else if ( arg.find("--chunk=") == 0 ) {
            tg::Rectangle rectangle = tg::parseChunk(arg.substr(8).c_str());
            min_lon = rectangle.getMin().x();
            min_lat = rectangle.getMin().y();
            max_lon = rectangle.getMax().x();
            max_lat = rectangle.getMax().y();
	} else {
	    usage( argc, argv );
	    exit(-1);
	}
    }

    elev_src.push_back( "SRTM-United_States-1" );
    elev_src.push_back( "SRTM-North_America-3" );
    elev_src.push_back( "SRTM-South_America-3" );
    elev_src.push_back( "SRTM-Eurasia-3" );
    elev_src.push_back( "DEM-USGS-3" );
    elev_src.push_back( "SRTM-30" );

    SG_LOG(SG_GENERAL, SG_INFO, "Input file = " << input_file);
    SG_LOG(SG_GENERAL, SG_INFO, "Terrain sources = ");
    for ( unsigned int i = 0; i < elev_src.size(); ++i ) {
        SG_LOG(SG_GENERAL, SG_INFO, "  " << work_dir << "/" << elev_src[i] );
    }
    SG_LOG(SG_GENERAL, SG_INFO, "Work directory = " << work_dir);
    SG_LOG(SG_GENERAL, SG_INFO, "Nudge = " << nudge);
    SG_LOG(SG_GENERAL, SG_INFO, "Longitude = " << min_lon << ':' << max_lon);
    SG_LOG(SG_GENERAL, SG_INFO, "Latitude = " << min_lat << ':' << max_lat);

    if (max_lon < min_lon || max_lat < min_lat ||
	min_lat < -90 || max_lat > 90 ||
	min_lon < -180 || max_lon > 180) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Bad longitude or latitude");
	exit(1);
    }

    if ( work_dir == "" ) {
	SG_LOG( SG_GENERAL, SG_ALERT, 
		"Error: no work directory specified." );
	usage( argc, argv );
	exit(-1);
    }

    if ( input_file == "" ) {
	SG_LOG( SG_GENERAL, SG_ALERT, 
		"Error: no input file." );
	exit(-1);
    }

    // make work directory
#ifdef _MSC_VER
    fg_mkdir( work_dir.c_str() );
#else
    string command = "mkdir -p " + work_dir;
    system( command.c_str() );
#endif

    // initialize persistant polygon counter
    string counter_file = work_dir + "/poly_counter";
    poly_index_init( counter_file );

    sg_gzifstream in( input_file );
    if ( !in.is_open() ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << input_file );
        exit(-1);
    }

    string_list runways_list, taxiways_list;
    string last_apt_id = "";
    string last_apt_info = "";
    string line;
    char tmp[2048];

    while ( ! in.eof() ) {
	in.getline(tmp, 2048);
	line = tmp;
	SG_LOG( SG_GENERAL, SG_INFO, "-> " << line );

	if ( line.length() == 0 ) {
	    // empty, skip
	} else if (( line[0] == '#' ) || (line[0] == '/' && line[1] == '/')) {
	    // comment, skip
	} else if ( line[0] == 'A' || line[0] == 'H' || line[0] == 'S' ) {
            // extract some airport runway info
            char ctmp, tmpid[32], rwy[32];
            string id;
            float lat, lon;
            int elev = 0;

            sscanf( line.c_str(), "%c %s %d",
                    &ctmp, tmpid, &elev );
            id = tmpid;
            SG_LOG( SG_GENERAL, SG_INFO, "Airport = " << id << " "
                    << elev );

            if ( !last_apt_id.empty()) {
                if ( runways_list.size() ) {
                    sscanf( runways_list[0].c_str(), "%c %s %s %f %f",
                            &ctmp, tmpid, rwy, &lat, &lon );
                }

                if ( lon >= min_lon && lon <= max_lon &&
                     lat >= min_lat && lat <= max_lat )
                {
                    if ( start_id.length() && start_id == last_apt_id ) {
                        ready_to_go = true;
                    }

                    if ( ready_to_go ) {
                        // check point our location
                        char command[256];
                        sprintf( command,
                                 "echo before building %s >> last_apt",
                                 last_apt_id.c_str() );
                        system( command );

                        // process previous record
                        // process_airport(last_apt_id, runways_list, argv[2]);
                        try {
                            build_airport( last_apt_id, elev * SG_FEET_TO_METER,
                                           runways_list, taxiways_list,
                                           work_dir, elev_src );
                        } catch (sg_exception &e) {
                            SG_LOG( SG_GENERAL, SG_ALERT,
                                    "Failed to build airport = "
                                    << last_apt_id );
                            SG_LOG( SG_GENERAL, SG_ALERT, "Exception: "
                                    << e.getMessage() );
                            exit(-1);
                        }
                    }
		} else {
                    SG_LOG(SG_GENERAL, SG_INFO, "Skipping airport " << id);
		}
	    }
            last_apt_id = id;
            last_apt_info = line;
            // clear runway list for start of next airport
            runways_list.clear();
            taxiways_list.clear();
        } else if ( line[0] == 'R' ) {
            // runway entry
            runways_list.push_back(line);
        } else if ( line[0] == 'T' ) {
            // runway entry
            taxiways_list.push_back(line);
        } else {
            SG_LOG( SG_GENERAL, SG_ALERT, 
                    "Unknown line in file: " << line );
            exit(-1);
        }
    }

    cout << "last_apt_id.length() = " << last_apt_id.length() << endl;

    if ( !last_apt_id.empty()) {
        char ctmp, tmpid[32], rwy[32];
        string id;
        float lat, lon;
        int elev = 0;

        if ( runways_list.size() ) {
            sscanf( runways_list[0].c_str(), "%c %s %s %f %f",
                    &ctmp, tmpid, rwy, &lat, &lon );
        }

        if ( lon >= min_lon && lon <= max_lon &&
             lat >= min_lat && lat <= max_lat )
        {
            if ( start_id.length() && start_id == last_apt_id ) {
                ready_to_go = true;
            }

            if ( ready_to_go ) {
                // check point our location
                char command[256];
                sprintf( command,
                         "echo before building %s >> last_apt",
                         last_apt_id.c_str() );
                system( command );

                // process previous record
                // process_airport(last_apt_id, runways_list, argv[2]);
                try {
                    build_airport( last_apt_id, elev * SG_FEET_TO_METER,
                                   runways_list, taxiways_list,
                                   work_dir, elev_src );
                } catch (sg_exception &e) {
                    SG_LOG( SG_GENERAL, SG_ALERT,
                            "Failed to build airport = "
                            << last_apt_id );
                    SG_LOG( SG_GENERAL, SG_ALERT, "Exception: "
                            << e.getMessage() );
                    exit(-1);
                }
            }
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "Skipping airport " << id);
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "[FINISHED CORRECTLY]");

    return 0;
}
