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


#ifndef _HGT_HXX
#define _HGT_HXX

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <zlib.h>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>


#define MAX_HGT_SIZE 3601


class TGHgt {

private:

    // file pointer for input
    gzFile fd;

    // coordinates (in arc seconds) of south west corner
    double originx, originy;
    
    // number of columns and rows
    int cols, rows;
    
    // Distance between column and row data points (in arc seconds)
    double col_step, row_step;
    
    // pointers to the actual grid data allocated here
    short int (*data)[MAX_HGT_SIZE];
    short int (*output_data)[MAX_HGT_SIZE];

    int hgt_resolution;
  
public:

    // Constructor, _res must be either "1" for the 1arcsec data or
    // "3" for the 3arcsec data.
    TGHgt( int _res );
    TGHgt( int _res, const SGPath &file );

    // Destructor
    ~TGHgt();

    // open an HGT file (use "-" if input is coming from stdin)
    bool open ( const SGPath &file );

    // close an HGT file
    bool close();

    // load an hgt file
    bool load();

    // write out the area of data covered by the specified bucket.
    // Data is written out column by column starting at the lower left
    // hand corner.
    bool write_area( const string& root, SGBucket& b );

    // Informational methods
    inline double get_originx() const { return originx; }
    inline double get_originy() const { return originy; }
    inline int get_cols() const { return cols; }
    inline int get_rows() const { return rows; }
    inline double get_col_step() const { return col_step; }
    inline double get_row_step() const { return row_step; }

    /**
     * Test whether an area contains any non-zero elevations.
     */
    bool has_non_zero_elev (int start_x, int span_x,
                            int start_y, int span_y) const;
};


#endif // _HGT_HXX


