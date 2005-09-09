// apt_surface.hxx -- class to manage airport terrain surface
//                    approximation and smoothing
//
// Written by Curtis Olson, started March 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
//


#ifndef _APT_SURFACE_HXX
#define _APT_SURFACE_HXX


#include <string>

// libnewmat includes and defines
#define WANT_STREAM		// include.h will get stream fns
#define WANT_MATH		// include.h will get math fns
				// newmatap.h will get include.h
#include <newmat/newmatap.h>	// need matrix applications
#include <newmat/newmatio.h>	// need matrix output routines

#include <simgear/math/point3d.hxx>


/***
 * A dirt simple matrix class for our convenience based on top of Point3D
 */

class SimpleMatrix {

private:

  int _rows;
  int _cols;
  point_list m;

public:

  inline SimpleMatrix( int columns, int rows ) {
    _cols = columns;
    _rows = rows;
    m.resize( _cols * _rows );
  }

  inline Point3D element( int col, int row ) {
    int index = ( row * _cols ) + col;
    if ( col < 0 || col >= _cols ) {
      cout << "column out of bounds on read (" << col << " >= " << _cols << ")"
	   << endl;
      int *p = 0; *p = 1; // force crash
    } else if ( row < 0 || row >= _rows ) {
      cout << "row out of bounds on read (" << row << " >= " << _rows << ")"
	   << endl;
      int *p = 0; *p = 1; // force crash
    }
    return m[index];
  }

  inline void set( int col, int row, Point3D p ) {
    int index = ( row * _cols ) + col;
    if ( col < 0 || col >= _cols ) {
      cout << "column out of bounds on set (" << col << " >= " << _cols << ")"
	   << endl;
      int *p = 0; *p = 1; // force crash
    } else if ( row < 0 || row >= _rows ) {
      cout << "row out of bounds on set (" << row << " >= " << _rows << ")"
	   << endl;
      int *p = 0; *p = 1; // force crash
    }
    m[index] = p;
  }

  inline int cols() const { return _cols; }
  inline int rows() const { return _rows; }
};


/***
 * Note of explanation.  When a TGAptSurface instance is created, you
 * must specify a min and max lon/lat containing the entire airport
 * area.  The class will divide up that area into a reasonably sized
 * regular grid.  It will then look up the elevation of each point on
 * the grid from the DEM/Array data.  Finally it will fit do a linear
 * least squares polygonal surface approximation from this grid.  Each
 * vertex of the actual airport model is drapped over this fitted
 * surface rather than over the underlying terrain data.  This
 * provides a) smoothing of noisy terrain data and b) natural rises
 * and dips in the airport surface.
 */

class TGAptSurface {

private:

    // The actual nurbs surface approximation for the airport
    SimpleMatrix *Pts;
    ColumnVector surface_coefficients;

    Point3D min_deg, max_deg;

    // externally seeded average airport elevation
    double average_elev_m;

public:

    // Constructor, specify min and max coordinates of desired area in
    // lon/lat degrees, also please specify an "average" airport
    // elevations in meters.
    TGAptSurface( const string &path, const string_list& elev_src,
                  Point3D _min_deg, Point3D _max_deg, double _average_elev_m );

    // Destructor
    ~TGAptSurface();

    // Use a linear least squares method to fit a 3d polynomial to the
    // sampled surface data
    void fit();

    // Query the elevation of a point, return -9999 if out of range.
    // This routine makes a simplistic assumption that X,Y space is
    // proportional to u,v space on the nurbs surface which it isn't.
    double query( double lon_deg, double lat_deg );

};


#endif // _APT_SURFACE_HXX
