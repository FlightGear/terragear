// hgtchop.cxx -- chop up a hgt file into it's corresponding pieces and stuff
//                them into the workspace directory
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1997  Curtis L. Olson  - curt@flightgear.org
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


#include <simgear/compiler.h>

#include STL_STRING

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>

#include <HGT/hgt.hxx>

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

#include "point2d.hxx"

SG_USING_STD(cout);
SG_USING_STD(string);


int main(int argc, char **argv) {
    sglog().setLogLevels( SG_ALL, SG_WARN );

    if ( argc != 3 ) {
	SG_LOG( SG_GENERAL, SG_ALERT, 
		"Usage " << argv[0] << " <hgt_file> <work_dir>" );
	exit(-1);
    }

    string hgt_name = argv[1];
    string work_dir = argv[2];

    // determine if file is 1arcsec or 3arcsec variety
    int resolution = 3;
    SGPath tmp1( hgt_name );
    SGPath tmp2( tmp1.dir() );
    string dir = tmp2.file();
    if ( dir == "1arcsec" ) {
        resolution = 1;
    } else if ( dir == "3arcsec" ) {
        resolution = 3;
    } else {
        cout << "ERROR: major problem in directory hierachy or file name!"
             << endl;
        cout << "unable to determine resolution." << endl;
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
	cout << "HGT file spans tile boundaries" << endl;
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


