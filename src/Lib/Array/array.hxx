// array.hxx -- Array management class
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998 - 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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


#ifndef _ARRAY_HXX
#define _ARRAY_HXX

#include <simgear/compiler.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/io/iostreams/sgstream.hxx>
#include <simgear/misc/sg_dir.hxx>
#include <boost/foreach.hpp>

#include "tg_contour.hxx"
#include "tg_polygon.hxx"

class TGArray {

private:
    gzFile array_in;

    // fitted file pointer
    sg_gzifstream *fitted_in;

    // coordinates (in arc seconds) of south west corner
    double originx, originy;

    // number of columns and rows
    int cols, rows;

    // Whether or not the input data have been rectified
    bool rectified;

    // Distance between column and row data points (in arc seconds)
    double col_step, row_step;

    // pointers to the actual grid data allocated here
    short *in_data;

    // output nodes
    std::vector<SGGeod> corner_list;
    std::vector<SGGeod> fitted_list;

    // list of cliff contours
    tgcontour_list cliffs_list;

    void parse_bin();

    // Routines for height rectification
    std::vector<int> collect_bad_points(const double bad_zone);
    bool is_bad_point(const int xgrid, const int ygrid, const std::vector<int>& bad_points) const;
    double rectify_point(const int xgrid, const int ygrid, const std::vector<int>& bad_points) const;
    bool is_near_cliff(const double lon1,const double lon2, const double bad_zone) const;

public:

    // Constructor
    TGArray( void );
    explicit TGArray( const std::string& file );

    // Destructor
    ~TGArray( void );

    // open an Array file (use "-" if input is coming from stdin)
    bool open ( const std::string& file_base );

    // Load contours from polygon files delineating height discontinuities
    void load_cliffs(const std::string & height_base);
      
    // return if array was successfully opened or not
    bool is_open() const;

    // close a Array file
    bool close();

    // parse a Array file
    bool parse( SGBucket& b );

    // write an Array file
    bool write( const std::string& root_dir, SGBucket& b );

    // write an Array file in binary format. If ht_rect is true,
    // the file will have extension 'arr.rectified.gz'
    void write_bin(const std::string& root_dir, bool ht_rect, SGBucket& b);
  
    // do our best to remove voids by picking data from the nearest
    // neighbor.
    void remove_voids();

    void rectify_heights(const double bad_zone);

    // Return the elevation of the closest non-void grid point to lon, lat
    double closest_nonvoid_elev( double lon, double lat ) const;

    // return the current altitude based on grid data.  We should
    // rewrite this to interpolate exact values, but for now this is
    // good enough
    double altitude_from_grid( double lon, double lat ) const;

    // Informational methods
    inline double get_originx() const { return originx; }
    inline double get_originy() const { return originy; }
    inline int get_cols() const { return cols; }
    inline int get_rows() const { return rows; }
    inline double get_col_step() const { return col_step; }
    inline double get_row_step() const { return row_step; }

    inline std::vector<SGGeod> const& get_corner_list() const { return corner_list; }
    inline std::vector<SGGeod> const& get_fitted_list() const { return fitted_list; }

    int get_array_elev( int col, int row ) const;
    void set_array_elev( int col, int row, int val );

    // Check whether or not two points are on the same side of contour
    bool check_points (const double a,const double b, const double c, const double d) const;

    // reset Array to initial state - ready to load another elevation file
    void unload( void );
};

#endif // _ARRAY_HXX
