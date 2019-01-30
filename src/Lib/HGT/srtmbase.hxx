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


#ifndef _SRTMBASE_HXX
#define _SRTMBASE_HXX

#include <simgear/compiler.h>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_dir.hxx>

class TGSrtmBase {

protected:
    TGSrtmBase() :
        remove_tmp_file(false)
    {
        originx = originy = 0.0;
        cols = rows = 0;
        col_step = row_step = 0.0;
    }

    ~TGSrtmBase();

    // coordinates (in arc seconds) of south west corner
    double originx, originy;

    // number of columns and rows
    int cols, rows;

    // Distance between column and row data points (in arc seconds)
    double col_step, row_step;

    bool remove_tmp_file;
    simgear::Dir tmp_dir;

public:

    // write out the area of data covered by the specified bucket.
    // Data is written out column by column starting at the lower left
    // hand corner.
    bool write_area( const std::string& root, SGBucket& b );

    bool write_area_bin(const SGPath& aPath,
        int start_x, int start_y, int min_x, int min_y,
    int span_x, int span_y, int col_step, int row_step);

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

    virtual short height( int x, int  ) const = 0;
};


#endif // _SRTMBASE_HXX
