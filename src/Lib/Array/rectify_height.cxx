// rectify_height.cxx -- Correct height grid for incorrect heights near
//                       discontinuities included in cliff files
//
// Written by James Hester 2018
//
// Copyright (C) 2018 James Hester 
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/debug/logstream.hxx>

#include <sys/types.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include "array.hxx"

/* This program will find all height files in the directory provided,
and if they have an associated cliff file, will adjust and then output
the heights. */

// display usage and exit
static void usage( const std::string& name ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Usage: " << name);
    SG_LOG(SG_GENERAL, SG_ALERT, "  --work-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --height-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "[  --tile-id=<id>]");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --min-lon=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --max-lon=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --min-lat=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --max-lat=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "[  --min-dist=<float>]");
    exit(-1);
}

int main( int argc, char **argv) {
  std::string output_dir = ".";
  std::string work_dir = ".";
  std::string height_dir = "";
  SGGeod min,max;
  long tile_id = -1;
  double bad_zone = 100;   //distance in m from cliff to rectify

  sglog().setLogLevels(SG_ALL,SG_INFO);
  sglog().set_log_priority(SG_DEBUG);
  
  //
  // Parse the command-line arguments.
  //
  int arg_pos;
  for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        std::string arg = argv[arg_pos];
    
        if (arg.compare(0, 11, "--work-dir=") == 0) {
            work_dir = arg.substr(11);
        } else if (arg.compare(0, 13, "--height-dir=") == 0) {
            height_dir = arg.substr(13);
        } else if (arg.compare(0, 10, "--tile-id=") == 0) {
            tile_id = atol(arg.substr(10).c_str());
        } else if ( arg.compare(0, 10, "--min-lon=") == 0 ) {
            min.setLongitudeDeg(atof( arg.substr(10).c_str() ));
        } else if ( arg.compare(0, 10, "--max-lon=") == 0 ) {
            max.setLongitudeDeg(atof( arg.substr(10).c_str() ));
        } else if ( arg.compare(0, 10, "--min-lat=") == 0 ) {
            min.setLatitudeDeg(atof( arg.substr(10).c_str() ));
        } else if ( arg.compare(0, 10, "--max-lat=") == 0 ) {
            max.setLatitudeDeg(atof( arg.substr(10).c_str() ));
        } else if ( arg.compare(0, 11, "--min-dist=") == 0) {
            bad_zone = atof(arg.substr(11).c_str());
        } else if (arg.compare(0, 2, "--") == 0) {
            usage(argv[0]);
        } else {
            break;
        }
    }

  SG_LOG(SG_GENERAL, SG_ALERT, "Working directory is " << work_dir);
  SG_LOG(SG_GENERAL, SG_ALERT, "Heights are in " << height_dir);
  SG_LOG(SG_GENERAL, SG_ALERT, "Rectification zone within " << bad_zone << "m of cliffs");
  
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

  // Now generate the buckets
  std::vector<SGBucket> bucketList;
  if (tile_id == -1) {
        // build all the tiles in an area
        SG_LOG(SG_GENERAL, SG_ALERT, "Fixing heights within given bounding box");

        SGBucket b_min( min );
        SGBucket b_max( max );

        if ( b_min == b_max ) {
            bucketList.push_back( b_min );
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "  adjustment area spans tile boundaries");
            sgGetBuckets( min, max, bucketList );            
        }
  } else {
        // adjust the specified tile
        SG_LOG(SG_GENERAL, SG_ALERT, "Adjusting tile " << tile_id);
        bucketList.push_back( SGBucket( tile_id ) );
    }

  // Finally, loop over the buckets adjusting heights
  for (unsigned int i=0; i < bucketList.size(); i++) {
    SGBucket bucket = bucketList[i];
    TGArray array;
    std::string base_path = bucket.gen_base_path();
    std::string array_path = work_dir + "/" + height_dir + "/" + base_path + "/" + bucket.gen_index_str();
    if (!array.open(array_path)) {
      SG_LOG(SG_GENERAL,SG_DEBUG, "Failed to open array file " << array_path);
      continue;
    }
    SG_LOG(SG_GENERAL,SG_DEBUG, "Successfully opened array file");
    array.parse(bucket);
    array.close();
    array.remove_voids();
    array.rectify_heights(bad_zone);
    array.write_bin(work_dir + "/" + height_dir,true,bucket);
  }
}
