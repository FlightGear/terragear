// dted.cxx -- SRTM "dted" data management class
//
// Written by James Hester based on hgt code of
// Curtis Olson, started February 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - http://www.flightgear.org/~curt
// Copyright (C) 2018  James Hester 
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

#include "dted.hxx"

using std::cout;
using std::endl;
using std::string;


TGDted::TGDted( int _res ) 
{
    dted_resolution = _res;

    data = new short int[MAX_DTED_SIZE][MAX_DTED_SIZE];
    output_data = new short int[MAX_DTED_SIZE][MAX_DTED_SIZE];

    for (int x = 0; x < MAX_DTED_SIZE; ++x) {
        for (int y = 0; y < MAX_DTED_SIZE; ++y) {
            data[x][y] = 0;
            output_data[x][y] = 0;
        }
    }
}


TGDted::TGDted( int _res, const SGPath &file ) :
    TGDted(_res)
{
    TGDted::open( file );
}


TGDted::~TGDted() {
    // printf("class TGSrtmBase DEstructor called.\n");
    delete [] data;
    delete [] output_data;
}


// open an DTED file
bool
TGDted::open ( const SGPath &f ) {
    SGPath file_name = f;

    // open input file (or read from stdin)
    if ( file_name.str() ==  "-" ) {
        cout << "Loading DTED data file: stdin" << endl;
        if ( (fd = gzdopen(0, "r")) == NULL ) { // 0 == STDIN_FILENO
            cout << "ERROR: opening stdin" << endl;
            return false;
        }
    } else {
        if ( file_name.extension() == "zip" ) {
            // extract the .zip file to /tmp and point the file name
            // to the extracted file
            tmp_dir = simgear::Dir::tempDir("dted");
            
            cout << "Extracting " << file_name.str() << " to " << tmp_dir.path().str() << endl;
            string command = "unzip -d \"" + tmp_dir.path().str() + "\" " + file_name.base();
            if ( system( command.c_str() ) != -1 )
            {
                simgear::PathList files = tmp_dir.children(simgear::Dir::TYPE_FILE | simgear::Dir::NO_DOT_OR_DOTDOT);
                for (const SGPath& file : files) {
                    string ext = file.lower_extension();
                    if ( ext == "dted" ) {
                        file_name = file;
                        break;
                    }
                }
            
                remove_tmp_file = true;
                cout << "Proceeding with " << file_name.str() << endl;
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to issue system call " << command );
                return EXIT_FAILURE;
            }
        }

        cout << "Loading DTED data file: " << file_name.str() << endl;
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

    // Determine originx/originy from file contents
    // User Header Label
    char header[3];
    char location[7];
    int degrees,minutes,seconds;
    char hemisphere;
    // Check header
    gzread(fd,header,3);
    if (strncmp(header,"UHL",3) != 0) {
	    cout << "UHL User Header Label not found" << endl;
	    return false;
    }
    gzread(fd,header,1);  //dummy
    gzread(fd,location,8);//longitude
    sscanf(location,"%3d%2d%2d%c",&degrees,&minutes,&seconds,&hemisphere);
    originx = degrees *3600 + minutes*60 + seconds;
    if(hemisphere == 'W') {
	   originx = - originx;
    } 
    gzread(fd,location,8);//latitude
    sscanf(location,"%3d%2d%2d%c",&degrees,&minutes,&seconds,&hemisphere);
    originy = degrees *3600 + minutes*60 + seconds;
    if(hemisphere == 'S') {
	   originy = - originy;
    } 
    cout << "  Origin = " << originx << ", " << originy << endl;

    return true;
}


// close an DTED file
bool
TGDted::close () {
    gzclose(fd);
    return true;
}


// load an DTED file
bool
TGDted::load( ) {
    int size;
    bool little_endian = sgIsLittleEndian();
    if ( dted_resolution == 1 ) {
        cols = rows = size = 3601;
        col_step = row_step = 1;
    } else if ( dted_resolution == 3 ) {
        cols = rows = size = 1201;
        col_step = row_step = 3;
    } else {
        cout << "Unknown DTED resolution, only 1 and 3 arcsec formats" << endl;
        cout << " are supported!" << endl;
        return false;
    }
    if (little_endian) {
	    cout << "Little Endian: swapping input values" << endl;
    }

    //Skip headers 
    gzseek(fd,3428,SEEK_SET); 
    unsigned short int latct, longct;
    short int *var;
    int dummy;  //to read column header
    for ( int col = 0; col < size; ++col ) {
	    dummy = 0;  // zero out all bytes
	    longct = 0;
	    latct = 0;
	    gzread(fd,&dummy,1);   //sentinel
	    if(dummy != 170) {
		    cout << "Failed to find sentinel at col " << col << endl;
		    return false;
	    }
	    gzread(fd,&dummy,3);   //block count
	    gzread(fd,&longct,2);   //Longitude count
	    gzread(fd,&latct,2);   //Latitude count
            if ( little_endian ) {
	        sgEndianSwap(&longct);
	    }
	    // cout << "Longitude count " << longct << endl;
            for ( int row = 0; row < size; ++row ) {
                var = &data[col][row];
                if ( gzread ( fd, var, 2 ) != sizeof(short) ) {
                return false;
                }
                if ( little_endian ) { 
		    sgEndianSwap( (unsigned short int*)var); 
	        }
            }
	    gzread(fd,&dummy,4);   //Checksum
	    // Check values are right
	    if (col == 0) {
		    cout << data[col][0] << endl;
		    cout << data[col][1] << endl;
		    cout << data[col][2] << endl;
	    }
    }

    return true;
}
