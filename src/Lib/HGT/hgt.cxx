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
#include <simgear/misc/sg_dir.hxx>
#include <simgear/debug/logstream.hxx>


#include "hgt.hxx"

using std::cout;
using std::endl;
using std::string;


TGHgt::TGHgt( int _res ) 
{
    hgt_resolution = _res;

    data = new short int[MAX_HGT_SIZE][MAX_HGT_SIZE];
    read_buffer = new short int[MAX_HGT_SIZE];

    for (int x = 0; x < MAX_HGT_SIZE; ++x) {
        for (int y = 0; y < MAX_HGT_SIZE; ++y) {
            data[x][y] = 0;
        }

        read_buffer[x] = 0;
    }
}


TGHgt::TGHgt( int _res, const SGPath &file ) :
    TGHgt( _res )
{
    TGHgt::open( file );
}


TGHgt::~TGHgt() {
    // printf("class TGSrtmBase DEstructor called.\n");
    delete [] data;
    delete [] read_buffer;
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
            tmp_dir = simgear::Dir::tempDir("hgt");
            
            cout << "Extracting " << file_name.str() << " to " << tmp_dir.path().str() << endl;
            string command = "unzip -d \"" + tmp_dir.path().str() + "\" " + file_name.base();
            if ( system( command.c_str() ) != -1 )
            {
                simgear::PathList files = tmp_dir.children(simgear::Dir::TYPE_FILE | simgear::Dir::NO_DOT_OR_DOTDOT);
                for (const SGPath& file : files) {
                    string ext = file.lower_extension();
                    if ( ext == "hgt" ) {
                        file_name = file;
                        break;
                    }
                }
            
                remove_tmp_file = true;
                cout << "Proceeding with " << file_name.str() << endl;
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to issue system call " << command );
                exit(1);
            }
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
    if ( name.substr(0, 1) == "S" || name.substr(0, 1) == "s" ) {
        originy = -originy;
    }
    originx = atof( name.substr(4, 3).c_str() ) * 3600.0;
    if ( name.substr(3, 1) == "W" ||  name.substr(3, 1) == "w") {
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

    for ( int row = size - 1; row >= 0; --row ) {
        if ( gzfread( (voidp)read_buffer, 2, size, fd ) != (unsigned)size ) {
            return false;
        }

        // convert to column-major
        for ( int col = 0; col < size; ++col )
            data[col][row] = *(read_buffer + col);
    }

    if (sgIsLittleEndian()) {
        auto pData = (unsigned short *)data;
        for (int i = 0; i < rows * cols; ++i) {
            sgEndianSwap(pData++);
        }
    }

    return true;
}
