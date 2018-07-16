// main.cxx -- top level construction routines
//
// Written by Curtis Olson, started March 1999.
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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef _MSC_VER
#  include <windows.h>
#  define tgSleep(x) Sleep(x*1000)
#else
#  define tgSleep(x) sleep(x)
#endif

#include <chrono>
#include <iostream>

#include <boost/thread.hpp>

#include <simgear/debug/logstream.hxx>
#include <Include/version.h>

#include "tgconstruct.hxx"
#include "priorities.hxx"
#include "usgs.hxx"

using std::string;
using std::vector;

// display usage and exit
static void usage( const string name ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Usage: " << name);
    SG_LOG(SG_GENERAL, SG_ALERT, "[ --output-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --work-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --share-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --match-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --cover=<path to land-cover raster>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --tile-id=<id>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --min-lon=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --max-lon=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --min-lat=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --max-lat=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --nudge=<float>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --priorities=<filename>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --usgs-map=<filename>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --ignore-landmass");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --threads");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --threads=<numthreads>");
    SG_LOG(SG_GENERAL, SG_ALERT, " ] <load directory...>");
    exit(-1);
}

void RemoveDuplicateBuckets( std::vector<SGBucket>& keep, std::vector<SGBucket>& remove )
{
    for ( unsigned int i=0; i<remove.size(); i++) {
        for ( unsigned int j=0; j<keep.size(); j++ ) {
            if ( remove[i] == keep[j] ) {
                keep.erase( keep.begin()+j );
                break;
            }
        }
    }
}

int main(int argc, char **argv) {
    string output_dir = ".";
    string work_dir = ".";
    string share_dir = "";
    string match_dir = "";
    string cover = "";
    string priorities_file = DEFAULT_PRIORITIES_FILE;
    string usgs_map_file = DEFAULT_USGS_MAPFILE;
    SGGeod min, max;
    long tile_id = -1;
    int num_threads = 1;

    vector<string> load_dirs;
    bool ignoreLandmass = false;
    double nudge=0.0;

    string debug_dir = ".";
    vector<string> debug_shape_defs;
    vector<string> debug_area_defs;

    auto start_time = std::chrono::high_resolution_clock::now();

    sglog().setLogLevels( SG_ALL, SG_INFO );

    //
    // Parse the command-line arguments.
    //
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        string arg = argv[arg_pos];

        if (arg.find("--output-dir=") == 0) {
            output_dir = arg.substr(13);
        } else if (arg.find("--work-dir=") == 0) {
            work_dir = arg.substr(11);
        } else if (arg.find("--share-dir=") == 0) {
            share_dir = arg.substr(12);
        } else if (arg.find("--match-dir=") == 0) {
            match_dir = arg.substr(12);
        } else if (arg.find("--tile-id=") == 0) {
            tile_id = atol(arg.substr(10).c_str());
        } else if ( arg.find("--min-lon=") == 0 ) {
            min.setLongitudeDeg(atof( arg.substr(10).c_str() ));
        } else if ( arg.find("--max-lon=") == 0 ) {
            max.setLongitudeDeg(atof( arg.substr(10).c_str() ));
        } else if ( arg.find("--min-lat=") == 0 ) {
            min.setLatitudeDeg(atof( arg.substr(10).c_str() ));
        } else if ( arg.find("--max-lat=") == 0 ) {
            max.setLatitudeDeg(atof( arg.substr(10).c_str() ));
        } else if (arg.find("--nudge=") == 0) {
            nudge = atof(arg.substr(8).c_str())*SG_EPSILON;
        } else if (arg.find("--cover=") == 0) {
            cover = arg.substr(8);
        } else if (arg.find("--priorities=") == 0) {
            priorities_file = arg.substr(13);
        } else if (arg.find("--usgs-map=") == 0) {
            usgs_map_file = arg.substr(11);
        } else if (arg.find("--ignore-landmass") == 0) {
            ignoreLandmass = true;
        } else if (arg.find("--threads=") == 0) {
            num_threads = atoi( arg.substr(10).c_str() );
        } else if (arg.find("--threads") == 0) {
            num_threads = boost::thread::hardware_concurrency();
        } else if (arg.find("--debug-dir=") == 0) {
            debug_dir = arg.substr(12);
        } else if (arg.find("--debug-areas=") == 0) {
            debug_area_defs.push_back( arg.substr(14) );
        } else if (arg.find("--debug-shapes=") == 0) {
            debug_shape_defs.push_back( arg.substr(15) );
        } else if (arg.find("--") == 0) {
            usage(argv[0]);
        } else {
            break;
        }
    }

    if ( share_dir == "" ) {
        share_dir = work_dir + "/Shared";
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "tg-construct version " << getTGVersion() << "\n");
    SG_LOG(SG_GENERAL, SG_ALERT, "Output directory is " << output_dir);
    SG_LOG(SG_GENERAL, SG_ALERT, "Working directory is " << work_dir);
    SG_LOG(SG_GENERAL, SG_ALERT, "Shared directory is " << share_dir);
    SG_LOG(SG_GENERAL, SG_ALERT, "Match directory is " << match_dir);
    
    if ( tile_id > 0 ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Tile id is " << tile_id);
    } else {
        if (min.isValid() && max.isValid() && (min != max))
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Longitude = " << min.getLongitudeDeg() << ':' << max.getLongitudeDeg());
            SG_LOG(SG_GENERAL, SG_ALERT, "Latitude = " << min.getLatitudeDeg() << ':' << max.getLatitudeDeg());
        } else
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Lon/Lat unset or wrong");
            exit(1);
        }
    }
    SG_LOG(SG_GENERAL, SG_ALERT, "Nudge is " << nudge);
    for (int i = arg_pos; i < argc; i++) {
        load_dirs.push_back(argv[i]);
        SG_LOG(SG_GENERAL, SG_ALERT, "Load directory: " << argv[i]);
    }

    TGAreaDefinitions areas;
    if ( areas.init( priorities_file ) ) {
        return EXIT_FAILURE;
    }

    // tile work queue 
    std::vector<SGBucket> matchList;
    std::vector<SGBucket> bucketList;
    SGLockedQueue<SGBucket> wq;
    SGMutex filelock;

    // First, check if we want to match existing scenery - 
    // if we do, generate a list of buckets outside of the bounding box
    if (tile_id == -1) {
        // build all the tiles in an area
        SG_LOG(SG_GENERAL, SG_ALERT, "Building tile(s) within given bounding box");

        SGBucket b_min( min );
        SGBucket b_max( max );

        if ( b_min == b_max ) {
            bucketList.push_back( b_min );
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "  construction area spans tile boundaries");
            sgGetBuckets( min, max, bucketList );            
        }
        if ( match_dir != "" ) {
            b_min = b_min.sibling(-1,-1);
            b_max = b_max.sibling(1,1);
            
            sgGetBuckets( b_min.get_center(), b_max.get_center(), matchList );            
        }
    } else {
        // construct the specified tile
        SG_LOG(SG_GENERAL, SG_ALERT, "Building tile " << tile_id);
        bucketList.push_back( SGBucket( tile_id ) );
        if ( match_dir != "" ) {
            SGBucket b_min( SGBucket(tile_id).sibling(-1,-1) );
            SGBucket b_max( SGBucket(tile_id).sibling( 1, 1) );
            sgGetBuckets( b_min.get_center(), b_max.get_center(), matchList );
        }
    }

    if ( match_dir != "" ) {
        RemoveDuplicateBuckets( matchList, bucketList );
        
        // generate the immuatble shared files - when tile matching, we must not
        // modify shared edges from an immutable file - new tile will collapse
        // triangles as appropriate.
        TGConstruct* construct = new TGConstruct( areas, 1, wq, &filelock );
        //construct->set_cover( cover );
        construct->set_paths( work_dir, share_dir, match_dir, output_dir, load_dirs );        
        construct->CreateMatchedEdgeFiles( matchList );
        delete construct;
    }
    
    std::vector<TGConstruct *> constructs;

	/* fill the workqueue */
	for (unsigned int i=0; i<bucketList.size(); i++) {
	    wq.push( bucketList[i] );
	}

    // now create the worker threads for stage 1
    for (int i=0; i<num_threads; i++) {
        TGConstruct* construct = new TGConstruct( areas, 1, wq, &filelock );
        //construct->set_cover( cover );
        construct->set_paths( work_dir, share_dir, match_dir, output_dir, load_dirs );
        construct->set_options( ignoreLandmass, nudge );
        construct->set_debug( debug_dir, debug_area_defs, debug_shape_defs );
        constructs.push_back( construct );
    }

    // start all threads
    for (unsigned int i=0; i<constructs.size(); i++) {
        constructs[i]->start();
    }
    // wait for workqueue to empty
    while( wq.size() ) {
        tgSleep( 5 );
    }
    // wait for all threads to complete
    for (unsigned int i=0; i<constructs.size(); i++) {
        constructs[i]->join();
    }

    // delete the stage 1 construct objects
    for (unsigned int i=0; i<constructs.size(); i++) {
        delete constructs[i];
    }
    constructs.clear();

	/* fill the workqueue */
	for (unsigned int i=0; i<bucketList.size(); i++) {
	    wq.push( bucketList[i] );
	}

    for (int i=0; i<num_threads; i++) {
        TGConstruct* construct = new TGConstruct( areas, 2, wq, &filelock );
        //construct->set_cover( cover );
        construct->set_paths( work_dir, share_dir, match_dir, output_dir, load_dirs );
        construct->set_options( ignoreLandmass, nudge );
        construct->set_debug( debug_dir, debug_area_defs, debug_shape_defs );
        constructs.push_back( construct );
    }

    // start all threads
    for (unsigned int i=0; i<constructs.size(); i++) {
        constructs[i]->start();
    }
    // wait for workqueue to empty
    while( wq.size() ) {
        tgSleep( 5 );
    }
    // wait for all threads to complete
    for (unsigned int i=0; i<constructs.size(); i++) {
        constructs[i]->join();
    }
    // delete the stage 2 construct objects
    for (unsigned int i=0; i<constructs.size(); i++) {
        delete constructs[i];
    }
    constructs.clear();

	/* fill the workqueue */
	for (unsigned int i=0; i<bucketList.size(); i++) {
	    wq.push( bucketList[i] );
	}

    for (int i=0; i<num_threads; i++) {
        TGConstruct* construct = new TGConstruct( areas, 3, wq, &filelock );
        //construct->set_cover( cover );
        construct->set_paths( work_dir, share_dir, match_dir, output_dir, load_dirs );
        construct->set_options( ignoreLandmass, nudge );
        construct->set_debug( debug_dir, debug_area_defs, debug_shape_defs );
        constructs.push_back( construct );
    }

    // start all threads
    for (unsigned int i=0; i<constructs.size(); i++) {
        constructs[i]->start();
    }
    // wait for workqueue to empty
    while( wq.size() ) {
        tgSleep( 5 );
    }
    // wait for all threads to complete
    for (unsigned int i=0; i<constructs.size(); i++) {
        constructs[i]->join();
    }
    // delete the stage 3 construct objects
    for (unsigned int i=0; i<constructs.size(); i++) {
        delete constructs[i];
    }
    constructs.clear();

    SG_LOG(SG_GENERAL, SG_ALERT, "[Finished successfully]");
    
    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    std::cout << std::endl << "Elapsed time: " << elapsed.count() << " seconds" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
