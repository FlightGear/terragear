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

#include <simgear/debug/logstream.hxx>
#include <Include/version.h>

#include "tgconstruct.hxx"
#include "usgs.hxx"

using std::string;
using std::vector;

// display usage and exit
static void usage( const string name ) {
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
    SG_LOG(SG_GENERAL, SG_ALERT, " ] <load directory...>");
    exit(-1);
}

int main(int argc, char **argv) {
    string output_dir = ".";
    string work_dir = ".";
    string share_dir = "";
    string cover = "";
    string priorities_file = DEFAULT_PRIORITIES_FILE;
    string usgs_map_file = DEFAULT_USGS_MAPFILE;
    SGGeod min, max;
    long tile_id = -1;

    vector<string> load_dirs;
    bool ignoreLandmass = false;
    double nudge=0.0;

    string debug_dir = ".";
    vector<string> debug_shape_defs;
    vector<string> debug_area_defs;

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

    if ( !load_area_types( priorities_file )
        || (!cover.empty() && !load_usgs_map( usgs_map_file)) ) {
        exit(-1);
    }

    // main construction data management class : Stage 1
    if (tile_id == -1) {
        // build all the tiles in an area
        SG_LOG(SG_GENERAL, SG_ALERT, "Building tile(s) within given bounding box");
        SGBucket b_min( min );
        SGBucket b_max( max );

        if ( b_min == b_max ) {
            TGConstruct* all_stages;

            all_stages = new TGConstruct();
            all_stages->set_cover( cover );
            all_stages->set_paths( work_dir, share_dir, output_dir, load_dirs );
            all_stages->set_options( ignoreLandmass, nudge );
            all_stages->set_bucket( b_min );
            all_stages->set_debug( debug_dir, debug_area_defs, debug_shape_defs );

            all_stages->ConstructBucketStage1();
            all_stages->ConstructBucketStage2();
            all_stages->ConstructBucketStage3();

            delete all_stages;
        } else {
            SGBucket b_cur;
            int dx, dy, i, j;
            int total_buckets, cur_bucket;

            sgBucketDiff(b_min, b_max, &dx, &dy);
            SG_LOG(SG_GENERAL, SG_ALERT, "  construction area spans tile boundaries");
            SG_LOG(SG_GENERAL, SG_ALERT, "  dx = " << dx << "  dy = " << dy);

            // construct stage 1
            total_buckets = (dx+1) * (dy + 1);
            cur_bucket = 0;
            for ( j = 0; j <= dy; j++ ) {
                for ( i = 0; i <= dx; i++ ) {
                    b_cur = sgBucketOffset(min.getLongitudeDeg(), min.getLatitudeDeg(), i, j);

                    TGConstruct* stage1;
                    stage1 = new TGConstruct();
                    stage1->set_cover( cover );
                    stage1->set_paths( work_dir, share_dir, output_dir, load_dirs );
                    stage1->set_options( ignoreLandmass, nudge );
                    stage1->set_bucket( b_cur );
                    stage1->set_debug( debug_dir, debug_area_defs, debug_shape_defs );

                    SG_LOG(SG_GENERAL, SG_ALERT, "STAGE 1: Construct bucket " << cur_bucket++ << " of " << total_buckets );
                    stage1->ConstructBucketStage1();
                    stage1->SaveToIntermediateFiles(1);

                    delete stage1;
                }
            }

            // construct stage 2
            cur_bucket = 0;
            for ( j = 0; j <= dy; j++ ) {
                for ( i = 0; i <= dx; i++ ) {
                    b_cur = sgBucketOffset(min.getLongitudeDeg(), min.getLatitudeDeg(), i, j);

                    TGConstruct* stage2;
                    stage2 = new TGConstruct();
                    stage2->set_cover( cover );
                    stage2->set_paths( work_dir, share_dir, output_dir, load_dirs );
                    stage2->set_options( ignoreLandmass, nudge );
                    stage2->set_bucket( b_cur );
                    stage2->set_debug( debug_dir, debug_area_defs, debug_shape_defs );

                    SG_LOG(SG_GENERAL, SG_ALERT, "STAGE 2: Construct bucket " << cur_bucket++ << " of " << total_buckets );
                    stage2->LoadFromIntermediateFiles(1);
                    stage2->ConstructBucketStage2();
                    stage2->SaveToIntermediateFiles(2);

                    delete stage2;
                }
            }

            // construct stage 3
            cur_bucket = 0;
            for ( j = 0; j <= dy; j++ ) {
                for ( i = 0; i <= dx; i++ ) {
                    b_cur = sgBucketOffset(min.getLongitudeDeg(), min.getLatitudeDeg(), i, j);

                    TGConstruct* stage3;
                    stage3 = new TGConstruct();
                    stage3->set_cover( cover );
                    stage3->set_paths( work_dir, share_dir, output_dir, load_dirs );
                    stage3->set_options( ignoreLandmass, nudge );
                    stage3->set_bucket( b_cur );
                    stage3->set_debug( debug_dir, debug_area_defs, debug_shape_defs );

                    SG_LOG(SG_GENERAL, SG_ALERT, "STAGE 3: Construct bucket " << cur_bucket++ << " of " << total_buckets );
                    stage3->LoadFromIntermediateFiles(2);
                    stage3->ConstructBucketStage3();

                    delete stage3;
                }
            }
        }
    } else {
        // construct the specified tile
        SG_LOG(SG_GENERAL, SG_ALERT, "Building tile " << tile_id);
        SGBucket b( tile_id );
        TGConstruct* all_stages;

        all_stages = new TGConstruct();
        all_stages->set_cover( cover );
        all_stages->set_paths( work_dir, share_dir, output_dir, load_dirs );
        all_stages->set_options( ignoreLandmass, nudge );
        all_stages->set_bucket( b );
        all_stages->set_debug( debug_dir, debug_area_defs, debug_shape_defs );

        all_stages->ConstructBucketStage1();
        all_stages->ConstructBucketStage2();
        all_stages->ConstructBucketStage3();

        delete all_stages;
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "[Finished successfully]");
    return 0;
}
