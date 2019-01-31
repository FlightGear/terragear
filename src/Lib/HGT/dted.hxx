// dted.hxx -- SRTM "dted" data management class
//
// Written by James Hester based on hgt.hxx, which was
// written by Curtis Olson who started February 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - http://www.flightgear.org/~curt
// Copyright (c) 2018  James Hester
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


#ifndef _DTED_HXX
#define _DTED_HXX

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include "srtmbase.hxx"

#include <zlib.h>

#include <string>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

#define MAX_DTED_SIZE 3601


class TGDted : public TGSrtmBase {

private:

    // file pointer for input
    gzFile fd;

    int dted_resolution;
    
    // pointers to the actual grid data allocated here
    short int (*data)[MAX_DTED_SIZE];
    short int (*output_data)[MAX_DTED_SIZE];

public:

    // Constructor, _res must be either "1" for the 1arcsec data or
    // "3" for the 3arcsec data.
    explicit TGDted( int _res );
    TGDted( int _res, const SGPath &file );

    // Destructor
    ~TGDted();

    // open an DTED file (use "-" if input is coming from stdin)
    bool open ( const SGPath &file );

    // close an DTED file
    bool close();

    // load an dted file
    bool load();

    virtual short height( int x, int y ) const override { return data[x][y]; }
};


#endif // _DTED_HXX


