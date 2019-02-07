// hgt.hxx -- SRTM "hgt" data management class
//
// Written by Curtis Olson, started February 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: hgt.cxx,v 1.7 2005-12-19 16:06:45 curt Exp $


#include <iostream>
#include <stdlib.h>
#include <zlib.h>

#include <simgear/compiler.h>
#include <simgear/io/lowlevel.hxx>

#include "srtmbase.hxx"

using std::cout;
using std::endl;
using std::string;

TGSrtmBase::~TGSrtmBase()
{
    if (remove_tmp_file) {
        tmp_dir.remove(true /*recursive*/);
    }
}

// write out the area of data covered by the specified bucket.  Data
// is written out column by column starting at the lower left hand
// corner.
bool
TGSrtmBase::write_area( const string& root, SGBucket& b ) {
    // calculate some boundaries
    double min_x = ( b.get_center_lon() - 0.5 * b.get_width() ) * 3600.0;
    double max_x = ( b.get_center_lon() + 0.5 * b.get_width() ) * 3600.0;

    double min_y = ( b.get_center_lat() - 0.5 * b.get_height() ) * 3600.0;
    double max_y = ( b.get_center_lat() + 0.5 * b.get_height() ) * 3600.0;

    cout << b << endl;
    cout << "width = " << b.get_width() << " height = " << b.get_height()
	 << endl;
    cout << "min = " << min_x << "," << min_y
         << "  max = " << max_x << "," << max_y << endl;
    int start_x = (int)((min_x - originx) / col_step);
    int span_x = (int)(b.get_width() * 3600.0 / col_step);

    int start_y = (int)((min_y - originy) / row_step);
    int span_y = (int)(b.get_height() * 3600.0 / row_step);

    cout << "start_x = " << start_x << "  span_x = " << span_x << endl;
    cout << "start_y = " << start_y << "  span_y = " << span_y << endl;

    // Do a simple sanity checking.  But, please, please be nice to
    // this write_area() routine and feed it buckets that coincide
    // well with the underlying grid structure and spacing.

    if ( ( min_x < originx )
	 || ( max_x > originx + cols * col_step )
	 || ( min_y < originy )
	 || ( max_y > originy + rows * row_step ) ) {
	cout << "  ERROR: bucket at least partially outside HGT data range!" <<
	    endl;
	return false;
    }

    // If the area is all ocean, skip it.
    if ( !has_non_zero_elev(start_x, span_x, start_y, span_y) ) {
        cout << "Tile is all zero elevation: skipping" << endl;
        return false;
    }

    // generate output file name
    string base = b.gen_base_path();
    string path = root + "/" + base;
    SGPath sgp( path );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    string array_file = path + "/" + b.gen_index_str() + ".arr.gz";
    cout << "array_file = " << array_file << endl;

    write_area_bin(array_file, start_x, start_y, min_x, min_y,
        span_x, span_y, col_step, row_step);

    return true;
}

bool TGSrtmBase::write_area_bin(const SGPath& aPath, int start_x, int start_y,
    int min_x, int min_y,
    int span_x, int span_y, int col_step, int row_step)
{
    gzFile fp;
    if ( (fp = gzopen( aPath.c_str(), "wb9" )) == NULL ) {
	    cout << "ERROR:  cannot open " << aPath.str() << " for writing!" << endl;
	    return false;
    }

    int32_t header = 0x54474152; // 'TGAR'
    sgWriteLong(fp, header);
    sgWriteInt(fp, min_x); sgWriteInt(fp, min_y);
    sgWriteInt(fp, span_x + 1); sgWriteInt(fp, col_step);
    sgWriteInt(fp, span_y + 1); sgWriteInt(fp, row_step);

    for ( int i = start_x; i <= start_x + span_x; ++i ) {
	    for ( int j = start_y; j <= start_y + span_y; ++j ) {
            sgWriteShort(fp, height(i,j));
	    }
    }

    gzclose(fp);
    return true;
}

bool
TGSrtmBase::has_non_zero_elev (int start_x, int span_x,
                          int start_y, int span_y) const
{
    for ( int row = start_y; row < start_y + span_y; row++ ) {
        for ( int col = start_x; col < start_x + span_x; col++ ) {
            if ( height(col,row) != 0 )
                return true;
        }
    }

    return false;
}
