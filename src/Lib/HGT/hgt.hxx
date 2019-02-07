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
// $Id: hgt.hxx,v 1.4 2004-11-19 22:25:50 curt Exp $


#ifndef _HGT_HXX
#define _HGT_HXX

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include "srtmbase.hxx"

#include <zlib.h>

#include <string>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

#define MAX_HGT_SIZE 3601


class TGHgt : public TGSrtmBase {

private:

    // file pointer for input
    gzFile fd;

    int hgt_resolution;
    
    // pointers to the actual grid data allocated here
    short int (*data)[MAX_HGT_SIZE];
    short int *read_buffer;

public:

    // Constructor, _res must be either "1" for the 1arcsec data or
    // "3" for the 3arcsec data.
    explicit TGHgt( int _res );
    TGHgt( int _res, const SGPath &file );

    // Destructor
    ~TGHgt();

    // open an HGT file (use "-" if input is coming from stdin)
    bool open ( const SGPath &file );

    // close an HGT file
    bool close();

    // load an hgt file
    bool load();

    virtual short height( int x, int y ) const override { return data[x][y]; }
};


#endif // _HGT_HXX


