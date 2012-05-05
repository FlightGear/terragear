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
// $Id: array.hxx,v 1.18 2005-11-10 16:26:59 curt Exp $


#ifndef _ARRAY_HXX
#define _ARRAY_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <simgear/bucket/newbucket.hxx>
#include <Geometry/point3d.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/misc/sgstream.hxx>

#define ARRAY_SIZE_1 20000


class TGArray {

private:

    // array file pointer
    sg_gzifstream *array_in;

    // fitted file pointer
    sg_gzifstream *fitted_in;

    // coordinates (in arc seconds) of south west corner
    double originx, originy;
    
    // number of columns and rows
    int cols, rows;
    
    // Distance between column and row data points (in arc seconds)
    double col_step, row_step;
    
    // pointers to the actual grid data allocated here
    int **in_data;
    // float (*out_data)[ARRAY_SIZE_1];

    // output nodes
    point_list corner_list;
    point_list fitted_list;

public:

    // Constructor
    TGArray( void );
    TGArray( const std::string& file );

    // Destructor
    ~TGArray( void );

    // open an Array file (use "-" if input is coming from stdin)
    bool open ( const std::string& file_base );

    // return if array was successfully opened or not
    inline bool is_open() { 
      if ( array_in != NULL ) {
	return array_in->is_open();
      } else {
	return false;
      }
    }

    // close a Array file
    bool close();

    // parse a Array file
    bool parse( SGBucket& b );

    // write an Array file
    bool write( const std::string root_dir, SGBucket& b );

    // do our best to remove voids by picking data from the nearest
    // neighbor.
    void remove_voids();

    // add a node to the output corner node list
    void add_corner_node( int i, int j, double val );

    // add a node to the output fitted node list
    void add_fit_node( int i, int j, double val );

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

    inline point_list get_corner_list() const { return corner_list; }
    inline point_list get_fitted_list() const { return fitted_list; }

    inline int get_array_elev( int col, int row ) {
        return in_data[col][row];
    }
    inline void set_array_elev( int col, int row, int val ) {
        in_data[col][row] = val;
    }
    inline Point3D get_fitted_pt( int i ) {
        return fitted_list[i];
    }
};


#endif // _ARRAY_HXX


