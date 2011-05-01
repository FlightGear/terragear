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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: hgt.cxx,v 1.7 2005-12-19 16:06:45 curt Exp $


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <stdlib.h>   // atof()
#include <iostream>

#ifdef SG_HAVE_STD_INCLUDES
#  include <cerrno>
#else
#  include <errno.h>
#endif

#ifdef _MSC_VER
#  include <direct.h>
#endif

#include <simgear/constants.h>
#include <simgear/io/lowlevel.hxx>

#include <plib/ul.h>

#include "hgt.hxx"

using std::cout;
using std::endl;
using std::string;


TGHgt::TGHgt( int _res ) {
    hgt_resolution = _res;

    data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    output_data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];

    remove_tmp_file = false;
}


TGHgt::TGHgt( int _res, const SGPath &file ) {
    hgt_resolution = _res;

    data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    output_data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];

    remove_tmp_file = false;

    TGHgt::open( file );
}


// open an HGT file
bool
TGHgt::open ( const SGPath &f ) {
    SGPath file_name = f;

    // open input file (or read from stdin)
    if ( file_name.str() ==  "-" ) {
	cout << "Loading HGT data file: stdin" << endl;
        if ( (fd = gzdopen(0, "r")) == NULL ) { // 0 == STDIN_FILENO
            cout << "ERROR: opening stdin" << endl;
            return false;
        }
    } else {
        if ( file_name.extension() == "zip" ) {
            // extract the .zip file to /tmp and point the file name
            // to the extracted file
	    SGPath tmp_dir = string( tempnam( 0, "hgt" ) );
	    tmp_dir.append( "dummy" );
	    tmp_dir.create_dir( 0700 );
	    cout << "Extracting " << file_name.str() << " to " << tmp_dir.dir() << endl;
	    string command = "unzip -d \"" + tmp_dir.dir() + "\" " + file_name.base();
	    system( command.c_str() );

	    file_name = tmp_dir.dir();
	    ulDir *dir = ulOpenDir( tmp_dir.dir().c_str() );
	    if ( dir ) {
		ulDirEnt *de;
		while ( ( de = ulReadDir( dir ) ) != 0 ) {
		    if ( !strcmp(de->d_name,".") || !strcmp(de->d_name,"..") || de->d_isdir ) {
			continue;
		    }
		    SGPath file( de->d_name );
		    string ext = file.extension();
		    if ( ext == "HGT" || ext == "hgt" ) {
			file_name.append( de->d_name );
			break;
		    }
		}
		ulCloseDir( dir );
	    }

            remove_tmp_file = true;
            remove_file_name = file_name.str();

            cout << "Proceeding with " << file_name.str() << endl;
        }

        cout << "Loading HGT data file: " << file_name.str() << endl;
        if ( (fd = gzopen( file_name.c_str(), "rb" )) == NULL ) {
            SGPath file_name_gz = file_name;
            file_name_gz.append( ".gz" );
            if ( (fd = gzopen( file_name_gz.c_str(), "rb" )) == NULL ) {
                cout << "ERROR: opening " << file_name.str() << " or "
                     << file_name_gz.str() << " for reading!" << endl;
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

    if ( remove_tmp_file ) {
        unlink( remove_file_name.c_str() );
        rmdir( remove_file_name.dir().c_str() );
    }

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
    for ( int row = size - 1; row >= 0; --row ) {
        for ( int col = 0; col < size; ++col ) {
            var = &data[col][row];
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



TGHgt::~TGHgt() {
    // printf("class TGSrtmBase DEstructor called.\n");
    delete [] data;
    delete [] output_data;
    if ( remove_tmp_file ) {
	ulDir *dir = ulOpenDir( remove_file_name.dir().c_str() );
	if ( dir ) {
	    ulDirEnt *de;
	    while ( ( de = ulReadDir( dir ) ) != 0 ) {
                if ( !strcmp(de->d_name,".") || !strcmp(de->d_name,"..") || de->d_isdir ) {
                    continue;
                }
		SGPath file( remove_file_name.dir() );
		file.append( de->d_name );
		unlink( file.c_str() );
	    }
	    ulCloseDir( dir );
	}
        rmdir( remove_file_name.dir().c_str() );
    }
}
