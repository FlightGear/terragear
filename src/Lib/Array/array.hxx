// array.hxx -- Array management class
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998 - 1999  Curtis L. Olson  - curt@flightgear.org
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


#ifndef _ARRAY_HXX
#define _ARRAY_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <vector>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/point3d.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/misc/sgstream.hxx>


SG_USING_STD(vector);


#define ARRAY_SIZE_1 1201


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
    int (*in_data)[ARRAY_SIZE_1];
    // float (*out_data)[ARRAY_SIZE_1];

    // output nodes
    point_list corner_list;
    point_list fitted_list;

    // bool
    bool fit_on_the_fly;

public:

    // Constructor
    TGArray( void );
    TGArray( const string& file );

    // Destructor
    ~TGArray( void );

    // open an Array file (use "-" if input is coming from stdin)
    bool open ( const string& file_based );

    // close a Array file
    bool close();

    // parse a Array file
    bool parse( SGBucket& b );

    // Use least squares to fit a simpler data set to dem data.
    // Return the number of fitted nodes.  This is a horrible approach
    // that doesn't really work, but it's better than nothing if
    // you've got nothing.  Using src/Prep/ArrayFit to create .fit
    // files from the .arr files is a *much* better approach, but it
    // is slower which is why it needs to be done "offline".
    int fit( double error );

    // add a node to the output corner node list
    void add_corner_node( int i, int j, double val );

    // add a node to the output fitted node list
    void add_fit_node( int i, int j, double val );

    // return the current altitude based on grid data.  We should
    // rewrite this to interpolate exact values, but for now this is
    // good enough
    double interpolate_altitude( double lon, double lat ) const;

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
    inline Point3D get_fitted_pt( int i ) {
        return fitted_list[i];
    }
};


#endif // _ARRAY_HXX


