// hgt.hxx -- SRTM "hgt" data management class
//
// Written by Curtis Olson, started February 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - curt@flightgear.org
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <stdlib.h>   // atof()

#ifdef SG_HAVE_STD_INCLUDES
#  include <cerrno>
#else
#  include <errno.h>
#endif

#include <simgear/constants.h>
#include <simgear/io/lowlevel.hxx>

#ifdef _MSC_VER
#  include <win32/mkdir.hpp>
#endif

#include "hgt.hxx"

SG_USING_STD(cout);
SG_USING_STD(endl);


TGHgt::TGHgt( int _res ) {
    hgt_resolution = _res;

    data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    output_data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
}


TGHgt::TGHgt( int _res, const SGPath &file ) {
    hgt_resolution = _res;

    data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    output_data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];

    TGHgt::open( file );
}


// open an HGT file
bool
TGHgt::open ( const SGPath &f ) {
    SGPath file_name = f;

    // open input file (or read from stdin)
    if ( file_name.str() ==  "-" ) {
	cout << "Loading HGT data file: stdin" << endl;
        if ( (fd = gzdopen(STDIN_FILENO, "r")) == NULL ) {
            cout << "ERROR: opening stdin" << endl;
            return false;
        }
    } else {
	cout << "Loading HGT data file: " << file_name.str() << endl;
        if ( (fd = gzopen( file_name.c_str(), "rb" )) == NULL ) {
            SGPath file_name_gz = file_name;
            file_name_gz.append( ".gz" );
            if ( (fd = gzopen( file_name_gz.c_str(), "rb" )) == NULL ) {
                cout << "ERROR: opening " << file_name.str() << " or "
                     << file_name_gz.str() << "for reading!" << endl;
                return false;
            }
        }
    }

    // Determine originx/originy from file name
    string name = file_name.file();
    cout << "  Name = " << name << endl;
    originy = atof( name.substr(1, 2).c_str() ) * 3600.0;
    if ( name.substr(0, 1) == "S" ) {
        originy = -originy;
    }
    originx = atof( name.substr(4, 3).c_str() ) * 3600.0;
    if ( name.substr(3, 1) == "W" ) {
        originx = -originx;
    }
    cout << "  Origin = " << originx << ", " << originy << endl;

    return true;
}


// close an HGT file
bool
TGHgt::close () {
    gzclose(fd);
    return true;
}


// load an hgt file
bool
TGHgt::load( ) {
    int size;
    if ( hgt_resolution == 1 ) {
        cols = rows = size = 3601;
        col_step = row_step = 1;
    } else if ( hgt_resolution == 3 ) {
        cols = rows = size = 1201;
        col_step = row_step = 3;
    } else {
        cout << "Unknown HGT resolution, only 1 and 3 arcsec formats" << endl;
        cout << " are supported!" << endl;
        return false;
    }

    short int *var;
    for ( int row = 0; row < size; ++row ) {
        for ( int col = 0; col < size; ++col ) {
            var = &data[row][col];
            if ( gzread ( fd, var, sizeof(short) ) != sizeof(short) ) {
                return false;
            }
            if ( sgIsLittleEndian() ) {
                sgEndianSwap( (unsigned short int*)var);
            }
        }
    }

    return true;
}


// write out the area of data covered by the specified bucket.  Data
// is written out column by column starting at the lower left hand
// corner.
bool
TGHgt::write_area( const string& root, SGBucket& b ) {
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
#ifdef _MSC_VER
    fg_mkdir( path.c_str() );
#else
    string command = "mkdir -p " + path;
    system( command.c_str() );
#endif

    string array_file = path + "/" + b.gen_index_str() + ".arr.gz";
    cout << "array_file = " << array_file << endl;

    // write the file
    gzFile fp;
    if ( (fp = gzopen( array_file.c_str(), "wb9" )) == NULL ) {
	cout << "ERROR:  cannot open " << array_file << " for writing!" << endl;
	exit(-1);
    }

    gzprintf( fp, "%d %d\n", (int)min_x, (int)min_y );
    gzprintf( fp, "%d %d %d %d\n", span_x + 1, (int)col_step, 
              span_y + 1, (int)row_step );
    for ( int i = start_x; i <= start_x + span_x; ++i ) {
	for ( int j = start_y; j <= start_y + span_y; ++j ) {
	    gzprintf( fp, "%d ", (int)data[i][j] );
	}
	gzprintf( fp, "\n" );
    }
    gzclose(fp);

    return true;
}


// write the entire area out in a simple ascii format
bool TGHgt::write_whole_ascii( const string& file ) {
    cout << "writing to " << file << endl;
    // write the file
    gzFile fp;
    if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
	cout << "ERROR:  cannot open " << file << " for writing!" << endl;
	exit(-1);
    }

    gzprintf( fp, "%d\n%d\n", rows, cols );
    for ( int i = 0; i < rows; i++ ) {
        for ( int j = 0; j < cols; j++ ) {
            gzprintf( fp, "%d\n", (int)data[i][j] );
        }
    }
    gzclose(fp);

    return true;
}



TGHgt::~TGHgt() {
    // printf("class TGHgt DEstructor called.\n");
    delete [] data;
    delete [] output_data;
}


bool
TGHgt::has_non_zero_elev (int start_x, int span_x,
                          int start_y, int span_y) const
{
    for ( int i = start_x; i < start_x + span_x; i++ ) {
        for ( int j = start_y; j < start_y + span_y; j++ ) {
            if ( data[i][j] != 0 )
                return true;
        }
    }

    return false;
}

