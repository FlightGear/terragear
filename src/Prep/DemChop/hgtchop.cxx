// hgtchop.cxx -- chop up a hgt file into it's corresponding pieces and stuff
//                them into the workspace directory
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1997  Curtis L. Olson  - http://www.flightgear.org/~curt
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

#include <chrono>
#include <cstdlib>
#include <string>
#include <iostream>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/misc/sg_dir.hxx>

#include <Include/version.h>
#include <HGT/hgt.hxx>

using std::cout;
using std::endl;
using std::string;


int main(int argc, char **argv) {
    sglog().setLogLevels( SG_ALL, SG_WARN );
    SG_LOG( SG_GENERAL, SG_ALERT, "hgtchop version " << getTGVersion() << "\n" );

    if ( argc != 4 ) {
        cout << "Usage " << argv[0] << " <resolution> <hgt_file> <work_dir>" << endl;
        cout << endl;
        cout << "\tresolution must be either 1 or 3 (1-arc-sec or 3-arc-sec)" << endl;

        return EXIT_FAILURE;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    int resolution = std::stoi(string(argv[1]));
    string hgt_name = string(argv[2]);
    string work_dir = string(argv[3]);

    // determine if file is 1arc-sec or 3arc-sec variety
    if ( resolution != 1 && resolution != 3 ) {
        cout << "ERROR: resolution must be 1 or 3." << endl;
        return EXIT_FAILURE;
    }

    SGPath sgp( work_dir );
    simgear::Dir workDir(sgp);
    workDir.create(0755);

    TGHgt hgt(resolution, hgt_name);
    hgt.load();
    hgt.close();

    SGGeod min = SGGeod::fromDeg( hgt.get_originx() / 3600.0 + SG_HALF_BUCKET_SPAN,
                                  hgt.get_originy() / 3600.0 + SG_HALF_BUCKET_SPAN );
    SGGeod max = SGGeod::fromDeg( (hgt.get_originx() + hgt.get_cols() * hgt.get_col_step()) / 3600.0 - SG_HALF_BUCKET_SPAN,
                                  (hgt.get_originy() + hgt.get_rows() * hgt.get_row_step()) / 3600.0 - SG_HALF_BUCKET_SPAN );
    SGBucket b_min( min );
    SGBucket b_max( max );

    if ( b_min == b_max ) {
        hgt.write_area( work_dir, b_min );
    } else {
        SGBucket b_cur;

        int dx, dy;
        sgBucketDiff(b_min, b_max, &dx, &dy);

        cout << "HGT file spans tile boundaries (ok)" << endl;
        cout << "  dx = " << dx << "  dy = " << dy << endl;

        if ( (dx > 20) || (dy > 20) ) {
            cout << "somethings really wrong!!!!" << endl;
            return EXIT_FAILURE;
        }

        for ( int j = 0; j <= dy; ++j ) {
            for ( int i = 0; i <= dx; ++i ) {
                b_cur = b_min.sibling(i, j);
                hgt.write_area( work_dir, b_cur );
            }
        }
    }

    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    std::cout << std::endl << "Elapsed time: " << elapsed.count() << " seconds" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
