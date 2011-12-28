// elevations.hxx -- routines to help calculate DEM elevations for a
//                   set of points
//
// Written by Curtis Olson, started April 2004.
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: elevations.hxx,v 1.4 2005-09-09 15:05:15 curt Exp $
//


// libnewmat includes and defines
#define WANT_STREAM		// include.h will get stream fns
#define WANT_MATH		// include.h will get math fns
				// newmatap.h will get include.h
#include <newmat/newmatap.h>	// need matrix applications
#include <newmat/newmatio.h>	// need matrix output routines

#include <simgear/constants.h>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#include "global.hxx"
#include "apt_surface.hxx"


// lookup node elevations for each point in the point_list.  Returns
// average of all points.  Doesn't modify the original list.
double tgAverageElevation( const string &root, const string_list elev_src,
                           const point_list points_source );

// lookup node elevations for each point in the specified nurbs++
// matrix.
void tgCalcElevations( const string &root, const string_list elev_src,
                       SimpleMatrix &Pts, double average );

// clamp all elevations to the specified range
void tgClampElevations( SimpleMatrix &Pts,
			double center_m, double max_clamp_m );
