// main.cxx -- main loop
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: main.cxx,v 1.37 2005/12/19 15:53:21 curt Exp $
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
#include <vector>
using std::vector;

#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>

using std::cout;
using std::endl;

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>
#include <simgear/misc/strutils.hxx>

#include <Polygon/index.hxx>
#include <Geometry/util.hxx>

#include "build.hxx"
#include "convex_hull.hxx"

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif



int nudge = 10;

static int is_in_range( string_list & runway_list, float min_lat, float max_lat, float min_lon, float max_lon );

// Display usage
static void usage( int argc, char **argv ) {
    SG_LOG(SG_GENERAL, SG_ALERT, 
	   "Usage " << argv[0] << " --input=<apt_file> "
	   << "--work=<work_dir> [ --start-id=abcd ] [ --nudge=n ] "
	   << "[--min-lon=<deg>] [--max-lon=<deg>] [--min-lat=<deg>] [--max-lat=<deg>] "
           << "[ --airport=abcd ]  [--tile=<tile>] [--chunk=<chunk>] [--verbose] [--help]");
}

// Display help and usage
static void help( int argc, char **argv ) {
    cout << "genapts generates airports for use in generating scenery for the FlightGear flight simulator.  ";
    cout << "Airport, runway, and taxiway vector data and attributes are input, and generated 3D airports ";
    cout << "are output for further processing by the TerraGear scenery creation tools.  ";
    cout << "\n\n";
    cout << "The standard input file is runways.dat.gz which is found in $FG_ROOT/Airports.  ";
    cout << "This file is periodically generated for the FlightGear project by Robin Peel, who ";
    cout << "maintains an airport database for both the X-Plane and FlightGear simulators.  ";
    cout << "The format of this file is documented on the FlightGear web site.  ";
    cout << "Any other input file corresponding to this format may be used as input to genapts.  ";
    cout << "Input files may be gzipped or left as plain text as required.  ";
    cout << "\n\n";
    cout << "Processing all the world's airports takes a *long* time.  To cut down processing time ";
    cout << "when only some airports are required, you may refine the input selection either by airport ";
    cout << "or by area.  By airport, either one airport can be specified using --airport=abcd, where abcd is ";
    cout << "a valid airport code eg. --airport-id=KORD, or a starting airport can be specified using --start-id=abcd ";
    cout << "where once again abcd is a valid airport code.  In this case, all airports in the file subsequent to the ";
    cout << "start-id are done.  This is convienient when re-starting after a previous error.  ";
    cout << "\nAn input area may be specified by lat and lon extent using min and max lat and lon.  ";
    cout << "Alternatively, you may specify a chunk (10 x 10 degrees) or tile (1 x 1 degree) using a string ";
    cout << "such as eg. w080n40, e000s27.  ";
    cout << "\nAn input file containing only a subset of the world's ";
    cout << "airports may of course be used.";
    cout << "\n\n";
    cout << "It is necessary to generate the elevation data for the area of interest PRIOR TO GENERATING THE AIRPORTS.  ";
    cout << "Failure to do this will result in airports being generated with an elevation of zero.  ";
    cout << "The following subdirectories of the work-dir will be searched for elevation files:\n\n";
    cout << "SRTM2-Africa-3\n";
    cout << "SRTM2-Australia-3\n";
    cout << "SRTM2-Eurasia-3\n";
    cout << "SRTM2-Islands-3\n";
    cout << "SRTM2-North_America-3\n";
    cout << "SRTM2-South_America-3\n";
    cout << "DEM-USGS-3\n";
    cout << "SRTM-30";
    cout << "\n\n";
    usage( argc, argv );
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
    string airport_id = "";
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
            tg::Rectangle rectangle = tg::parseChunk(arg.substr(8).c_str(),
                                                     10.0);
            min_lon = rectangle.getMin().x();
            min_lat = rectangle.getMin().y();
            max_lon = rectangle.getMax().x();
            max_lat = rectangle.getMax().y();
        } else if ( arg.find("--tile=") == 0 ) {
            tg::Rectangle rectangle = tg::parseTile(arg.substr(7).c_str());
            min_lon = rectangle.getMin().x();
            min_lat = rectangle.getMin().y();
            max_lon = rectangle.getMax().x();
            max_lat = rectangle.getMax().y();
	} else if ( arg.find("--airport=") == 0 ) {
	    airport_id = arg.substr(10).c_str();
	    ready_to_go = false;
	} else if ( (arg.find("--verbose") == 0) || (arg.find("-v") == 0) ) {
	    sglog().setLogLevels( SG_GENERAL, SG_BULK );
	} else if ( (arg.find("--help") == 0) || (arg.find("-h") == 0) ) {
	    help( argc, argv );
	    exit(-1);
	} else {
	    usage( argc, argv );
	    exit(-1);
	}
    }

    // Please update the help near the top of this file if you update this list.
    elev_src.push_back( "SRTM2-Africa-3" );
    elev_src.push_back( "SRTM2-Australia-3" );
    elev_src.push_back( "SRTM2-Eurasia-3" );
    elev_src.push_back( "SRTM2-Islands-3" );
    elev_src.push_back( "SRTM2-North_America-3" );
    elev_src.push_back( "SRTM2-South_America-3" );
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
    string airportareadir=work_dir+"/AirportArea";
#ifdef _MSC_VER
    fg_mkdir( airportareadir.c_str() );
#else
    string command = "mkdir -p " + airportareadir;
    system( command.c_str() );
#endif

    // initialize persistant polygon counter
    string counter_file = airportareadir+"/poly_counter";
    poly_index_init( counter_file );

    sg_gzifstream in( input_file );
    if ( !in.is_open() ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << input_file );
        exit(-1);
    }

    string_list runways_list;
    string_list beacon_list;
    string_list tower_list;
    string_list windsock_list;

    vector<string> token;
    string last_apt_id = "";
    string last_apt_info = "";
    string last_apt_type = "";
    string line;
    char tmp[2048];

    while ( ! in.eof() ) {
	in.getline(tmp, 2048);
	line = tmp;
	SG_LOG( SG_GENERAL, SG_DEBUG, "-> '" << line << "'" );
        if ( line.length() ) {
            token = simgear::strutils::split( line );
            if ( token.size() ) {
                SG_LOG( SG_GENERAL, SG_DEBUG, "token[0] " << token[0] );
            }
        } else {
            token.clear();
        }

        if ( !line.length() || !token.size() ) {
            // empty line, skip
        } else if ( (token[0] == "#") || (token[0] == "//") ) {
	    // comment, skip
        } else if ( token[0] == "I" ) {
            // First line, indicates IBM (i.e. DOS line endings I
            // believe.)

            // move past this line and read and discard the next line
            // which is the version and copyright information
            in.getline(tmp, 2048);
            vector<string> vers_token = simgear::strutils::split( tmp );
            SG_LOG( SG_GENERAL, SG_INFO, "Data version = " << vers_token[0] );
	} else if ( token[0] == "1" /* Airport */ ||
                    token[0] == "16" /* Seaplane base */ ||
                    token[0] == "17" /* Heliport */ ) {

            // extract some airport runway info
            string rwy;
            float lat, lon;
            
            string id = token[4];
            int elev = atoi( token[1].c_str() );
            SG_LOG( SG_GENERAL, SG_INFO, "Next airport = " << id << " "
                    << elev );

            if ( !last_apt_id.empty()) {
                if ( runways_list.size() ) {
                    vector<string> rwy_token
                        = simgear::strutils::split( runways_list[0] );
                    rwy = token[3];
                    lat = atof( token[1].c_str() );
                    lon = atof( token[2].c_str() );

                    if ( airport_id.length() && airport_id == last_apt_id ) {
                        ready_to_go = true;
                    } else if ( start_id.length() && start_id == last_apt_id ) {
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
                            if ( last_apt_type == "16" /* Seaplane base */ ||
                                 last_apt_type == "17" /* Heliport */ ) {
                                // skip building heliports and
                                // seaplane bases
                            } else {
                                if( is_in_range( runways_list, min_lat, max_lat, min_lon, max_lon ) ) {
                                    build_airport( last_apt_id,
                                                   elev * SG_FEET_TO_METER,
                                                   runways_list,
                                                   beacon_list,
                                                   tower_list,
                                                   windsock_list,
                                                   work_dir, elev_src );
                                }
                            }
                        } catch (sg_exception &e) {
                            SG_LOG( SG_GENERAL, SG_ALERT,
                                    "Failed to build airport = "
                                    << last_apt_id );
                            SG_LOG( SG_GENERAL, SG_ALERT, "Exception: "
                                    << e.getMessage() );
                            exit(-1);
                        }
                        if ( airport_id.length() ) {
                            ready_to_go = false;
                        }
                    }
		} else {
		    if(!airport_id.length()) {
			SG_LOG(SG_GENERAL, SG_INFO,
                               "ERRO: No runways, skipping = " << id);
		    }
		}
            }

            last_apt_id = id;
            last_apt_info = line;
            last_apt_type = token[0];

            // clear runway list for start of next airport
            runways_list.clear();
            beacon_list.clear();
            tower_list.clear();
            windsock_list.clear();
        } else if ( token[0] == "10" ) {
            // runway entry
            runways_list.push_back(line);
        } else if ( token[0] == "18" ) {
            // beacon entry
            beacon_list.push_back(line);
        } else if ( token[0] == "14" ) {
            // control tower entry
            tower_list.push_back(line);
        } else if ( token[0] == "19" ) {
            // windsock entry
            windsock_list.push_back(line);
        } else if ( token[0] == "15" ) {
            // ignore custom startup locations
        } else if ( token[0] == "50" || token[0] == "51" || token[0] == "52" 
                    || token[0] == "53" || token[0] == "54" || token[0] == "55" 
                    || token[0] == "56" )
        {
            // ignore frequency entries
        } else if ( token[0] == "99" ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "End of file reached" );
	} else if ( token[0] == "00" ) {
		// ??
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
                if ( last_apt_type == "16" /* Seaplane base */ ||
                     last_apt_type == "17" /* Heliport */ ) {
                    // skip building heliports and
                    // seaplane bases
                } else {
                    if( is_in_range( runways_list, min_lat, max_lat, min_lon, max_lon ) ) {
                        build_airport( last_apt_id, elev * SG_FEET_TO_METER,
                                       runways_list,
                                       beacon_list,
                                       tower_list,
                                       windsock_list,
                                       work_dir, elev_src );
                    }
                }
            } catch (sg_exception &e) {
                SG_LOG( SG_GENERAL, SG_ALERT,
                        "Failed to build airport = "
                        << last_apt_id );
                SG_LOG( SG_GENERAL, SG_ALERT, "Exception: "
                        << e.getMessage() );
                exit(-1);
            }
        } else {
            SG_LOG(SG_GENERAL, SG_INFO, "Skipping airport " << id);
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "[FINISHED CORRECTLY]");

    return 0;
}

static int is_in_range( string_list & runways_raw, float min_lat, float max_lat, float min_lon, float max_lon )
{
    int i;
    int rwy_count = 0;
    double apt_lon = 0.0, apt_lat = 0.0;

    for ( i = 0; i < (int)runways_raw.size(); ++i ) {
        ++rwy_count;

	string rwy_str = runways_raw[i];
        vector<string> token = simgear::strutils::split( rwy_str );

        apt_lat += atof( token[1].c_str() );
        apt_lon += atof( token[2].c_str() );
    }

    if( rwy_count > 0 ) {
      apt_lat /= rwy_count;
      apt_lon /= rwy_count;
    }

    if( apt_lat >= min_lat && apt_lat <= max_lat &&
        apt_lon >= min_lon && apt_lon <= max_lon ) {
        return 1;
    }

    return 0;
}
