// testassem.cxx -- assemble array files for a 1x1 degree area and
//                  dump as a stupid ascii format
//
// Written by Curtis Olson, started March 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - curt@flightgear.org
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include STL_STRING

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

SG_USING_STD(cout);
SG_USING_STD(string);


int data[3601][3601];

int main(int argc, char **argv) {
    sglog().setLogLevels( SG_ALL, SG_WARN );

    if ( argc != 5 ) {
	SG_LOG( SG_GENERAL, SG_ALERT, 
		"Usage " << argv[0] << " <work_dir> <base_lon> <base_lat> <out_file>" );
	exit(-1);
    }

    string work_dir = argv[1];
    double lon_deg = atof( argv[2] );
    double lat_deg = atof( argv[3] );
    string out_file = argv[4];
    int lon_arcsec = (int)(lon_deg * 3600.0);
    int lat_arcsec = (int)(lat_deg * 3600.0);
    int res = 1;

    point2d min, max;
    min.x = lon_deg + SG_HALF_BUCKET_SPAN;
    min.y = lat_deg + SG_HALF_BUCKET_SPAN;
    SGBucket b_min( min.x, min.y );

    max.x = lon_deg + 1 - SG_HALF_BUCKET_SPAN;
    max.y = lat_deg + 1 - SG_HALF_BUCKET_SPAN;
    SGBucket b_max( max.x, max.y );

    SGBucket b_cur;
    int dx, dy, i, j;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    cout << "  dx = " << dx << "  dy = " << dy << endl;

    if ( (dx > 20) || (dy > 20) ) {
        cout << "somethings really wrong!!!!" << endl;
        exit(-1);
    }

    for ( j = 0; j <= dy; j++ ) {
        for ( i = 0; i <= dx; i++ ) {
            b_cur = sgBucketOffset(min.x, min.y, i, j);
            string file = work_dir + "/";
            file += b_cur.gen_base_path() + "/";
            file += b_cur.gen_index_str();
            cout << file << endl;

            TGArray array;
            array.open( file );
            array.parse( b_cur );

            res = (int)(array.get_row_step());

            int startx = (int)((array.get_originx() - lon_arcsec) 
                               / array.get_col_step());
            int starty = (int)((array.get_originy() - lat_arcsec)
                               / array.get_row_step());

            cout << " start = " << startx << "," << starty << endl;
            for ( int jj = 0; jj < array.get_rows(); jj++ ) {
                for ( int ii = 0; ii < array.get_cols(); ii++ ) {
                    data[startx+ii][starty+jj] = array.get_array_elev(ii, jj);
                }
            }
        }
    }

    gzFile fp;
    if ( (fp = gzopen( out_file.c_str(), "wb9" )) == NULL ) {
	cout << "ERROR:  cannot open " << out_file << " for writing!" << endl;
	exit(-1);
    }

    gzprintf( fp, "%d\n", (int)(3600 / res) + 1);
    gzprintf( fp, "%d\n", (int)(3600 / res) + 1);
    for ( j = (int)(3600 / res); j >= 0; --j ) {
        for ( i = 0; i <= (int)(3600 / res); ++i ) {
            gzprintf( fp, "%d\n", data[i][j] );
        }
    }
    gzclose(fp);

    return 0;
}


