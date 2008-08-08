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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: hgtchop.cxx,v 1.7 2007-08-15 14:35:36 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <string>
#include <iostream>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>

#include <HGT/hgt.hxx>
#include <Polygon/point2d.hxx>

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

using std::cout;
using std::endl;
using std::string;


int main(int argc, char **argv) {
    sglog().setLogLevels( SG_ALL, SG_WARN );

    if ( argc != 4 ) {
	cout << "Usage " << argv[0] << " <resolution> <hgt_file> <work_dir>"
             << endl;
        cout << endl;
 	cout << "\tresolution must be either 1 or 3 for 1arcsec or 3arcsec"
             << endl;       
	exit(-1);
    }

    int resolution = atoi( argv[1] );
    string hgt_name = argv[2];
    string work_dir = argv[3];

    // determine if file is 1arcsec or 3arcsec variety
    if ( resolution != 1 && resolution != 3 ) {
        cout << "ERROR: resolution must be 1 or 3." << endl;
        exit( -1 );
    }

#ifdef _MSC_VER
    fg_mkdir( work_dir.c_str() );
#else
    string command = "mkdir -p " + work_dir;
    system( command.c_str() );
#endif

    TGHgt hgt(resolution, hgt_name);
    hgt.load();
    hgt.close();

    point2d min, max;
    min.x = hgt.get_originx() / 3600.0 + SG_HALF_BUCKET_SPAN;
    min.y = hgt.get_originy() / 3600.0 + SG_HALF_BUCKET_SPAN;
    SGBucket b_min( min.x, min.y );

    max.x = (hgt.get_originx() + hgt.get_cols() * hgt.get_col_step()) / 3600.0 
	- SG_HALF_BUCKET_SPAN;
    max.y = (hgt.get_originy() + hgt.get_rows() * hgt.get_row_step()) / 3600.0 
	- SG_HALF_BUCKET_SPAN;
    SGBucket b_max( max.x, max.y );

    if ( b_min == b_max ) {
	hgt.write_area( work_dir, b_min );
    } else {
	SGBucket b_cur;
	int dx, dy, i, j;

	sgBucketDiff(b_min, b_max, &dx, &dy);
	cout << "HGT file spans tile boundaries (ok)" << endl;
	cout << "  dx = " << dx << "  dy = " << dy << endl;

	if ( (dx > 20) || (dy > 20) ) {
	    cout << "somethings really wrong!!!!" << endl;
	    exit(-1);
	}

	for ( j = 0; j <= dy; j++ ) {
	    for ( i = 0; i <= dx; i++ ) {
		b_cur = sgBucketOffset(min.x, min.y, i, j);
		hgt.write_area( work_dir, b_cur );
	    }
	}
    }

    return 0;
}


