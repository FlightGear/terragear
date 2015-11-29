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

#include <boost/thread.hpp>

#include <simgear/debug/logstream.hxx>
#include <Include/version.h>

#include "tgconstruct_stage1.hxx"
#include "priorities.hxx"

// display usage and exit
static void usage( const std::string& name ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Usage: " << name);
    SG_LOG(SG_GENERAL, SG_ALERT, "[ --output-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --work-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --share-dir=<directory>");
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
    SG_LOG(SG_GENERAL, SG_ALERT, " ]");
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

std::vector<SGBucket> fillBucketList( long tile_id, const SGGeod& min, const SGGeod& max )
{
    std::vector<SGBucket> bucketList;
    
    // single tile, or range?
    if (tile_id == -1) {
        // build all the tiles in an area        
        SGBucket b_min( min );
        SGBucket b_max( max );
        
        sgGetBuckets( min, max, bucketList );
        SG_LOG(SG_GENERAL, SG_ALERT, "Given bounding box includes " << bucketList.size() << " tiles");
    } else {
        // construct the specified tile
        SG_LOG(SG_GENERAL, SG_ALERT, "Building tile " << tile_id);
        bucketList.push_back( SGBucket( tile_id ) );
    }
    
    return bucketList;
}

void doStage1( int num_threads, std::vector<SGBucket>& bucketList, 
               const std::string& priorities_file,
               const std::string& work_base, const std::string& dem_base, 
               const std::string& share_base, const std::string& debug_base )
{
    SGLockedQueue<SGBucket> wq;
    
    /* fill the workqueue */
    for (unsigned int i=0; i<bucketList.size(); i++) {
        wq.push( bucketList[i] );
    }
    
    // now create the worker threads for stage 1
    std::vector<tgConstructFirst *> constructs;    
    SGMutex filelock;
    
    for (int i=0; i<num_threads; i++) {
        tgConstructFirst* construct = new tgConstructFirst( priorities_file, wq, &filelock );
        construct->setPaths( work_base, dem_base, share_base, debug_base );
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
}

int main(int argc, char **argv) {
    std::string output_dir = ".";
    std::string work_dir = ".";
    std::string dem_dir = ".";
    std::string share_dir = "";
    std::string match_dir = "";
    std::string debug_dir = ".";
    
    std::string priorities_file = DEFAULT_PRIORITIES_FILE;
    
    SGGeod min, max;
    long   tile_id = -1;
    int    num_threads = 1;

    sglog().setLogLevels( SG_ALL, SG_INFO );

    //
    // Parse the command-line arguments.
    //
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        std::string arg = argv[arg_pos];

        if (arg.find("--output-dir=") == 0) {
            output_dir = arg.substr(13);
        } else if (arg.find("--work-dir=") == 0) {
            work_dir = arg.substr(11);
        } else if (arg.find("--dem-dir=") == 0) {
            dem_dir = arg.substr(10);
        } else if (arg.find("--share-dir=") == 0) {
            share_dir = arg.substr(12);
        } else if (arg.find("--match-dir=") == 0) {
            match_dir = arg.substr(12);            
        } else if (arg.find("--debug-dir=") == 0) {
            debug_dir = arg.substr(12);
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
        } else if (arg.find("--priorities=") == 0) {
            priorities_file = arg.substr(13);
        } else if (arg.find("--threads=") == 0) {
            num_threads = atoi( arg.substr(10).c_str() );
        } else if (arg.find("--threads") == 0) {
            num_threads = boost::thread::hardware_concurrency();
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
    SG_LOG(SG_GENERAL, SG_ALERT, "DEM directory is " << dem_dir);
    SG_LOG(SG_GENERAL, SG_ALERT, "Shared directory is " << share_dir);
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

    std::vector<SGBucket> bucketList = fillBucketList( tile_id, min, max );
    
# if 0 // tile matching     
    // tile work queue
    std::vector<SGBucket>   matchList;

    std::vector<TGConstruct *> constructs;    
    SGMutex filelock;
    
    if ( match_dir != "" ) {
        RemoveDuplicateBuckets( matchList, bucketList );
        
        // generate the immuatble shared files - when tile matching, we must not
        // modify shared edges from an immutable file - new tile will collapse
        // triangles as appropriate.
        TGConstruct* construct = new TGConstruct( areas, 1, wq, &filelock );
        //construct->set_cover( cover );
        construct->set_paths( work_dir, dem_dir, share_dir, match_dir, output_dir );        
        //construct->CreateMatchedEdgeFiles( matchList );
        delete construct;
    }
#endif

// STAGE 1
    doStage1( num_threads, bucketList, priorities_file, work_dir, dem_dir, share_dir, debug_dir );
    
// STAGE 2    
#if 0    
    /* fill the workqueue */
    for (unsigned int i=0; i<bucketList.size(); i++) {
        wq.push( bucketList[i] );
    }

    for (int i=0; i<num_threads; i++) {
        TGConstruct* construct = new TGConstruct( areas, 2, wq, &filelock );
        //construct->set_cover( cover );
        construct->set_paths( work_dir, dem_dir, share_dir, match_dir, output_dir );
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
#endif

// STAGE 3
#if 0
    /* fill the workqueue */
    for (unsigned int i=0; i<bucketList.size(); i++) {
        wq.push( bucketList[i] );
    }

    for (int i=0; i<num_threads; i++) {
        TGConstruct* construct = new TGConstruct( areas, 3, wq, &filelock );
        //construct->set_cover( cover );
        construct->set_paths( work_dir, dem_dir, share_dir, match_dir, output_dir );
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
#endif

    SG_LOG(SG_GENERAL, SG_ALERT, "[Finished successfully]");
    return 0;
}
