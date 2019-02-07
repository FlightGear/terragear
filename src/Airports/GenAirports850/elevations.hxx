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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#include <terragear/tg_surface.hxx>

// lookup node elevations for each point in the SGGeod list.  Returns
// average of all points.  Doesn't modify the original list.
double tgAverageElevation( const std::string &root, const string_list elev_src,
                           const std::vector<SGGeod>& points_source );

// lookup node elevations for each point in the specified nurbs++
// matrix.
void tgCalcElevations( const std::string &root, const string_list elev_src, tgMatrix& Pts, double average );

// clamp all elevations to the specified range
void tgClampElevations( tgMatrix& Pts, double center_m, double max_clamp_m );
