// apt_surface.hxx -- class to manage airport terrain surface
//                    approximation and smoothing
//
// Written by Curtis Olson, started March 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - curt@flightgear.org
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

#include <nurbs++/nurbsS.h>

#include <simgear/math/point3d.hxx>


/***
 * Note of explanation.  When a TGAptSurface instance is created, you
 * must specify a min and max lon/lat containing the entire airport
 * area.  The class will divide up that area into a reasonably sized
 * regular grid.  It will then look up the elevation of each point on
 * the grid from the DEM/Array data.  Finally it will build a nurbs
 * surface from this grid.  Each vertex of the actual airport model is
 * drapped over this nurbs surface rather than over the underlying
 * terrain data.  This provides a) smoothing of noisy terrain data, b)
 * natural rises and dips in the airport surface, c) chance to impress
 * colleages by using something or other nurbs surfaces -- or whatever
 * they are called. :-)
 */

class TGAptSurface {

private:

    // The actual nurbs surface approximation for the airport
    PlNurbsSurfaced *apt_surf;

    Point3D min_deg, max_deg;

public:

    // Constructor, specify min and max coordinates of desired area in
    // lon/lat degrees
    TGAptSurface( const string &path, const string_list& elev_src,
                  Point3D _min_deg, Point3D _max_deg );

    // Destructor
    ~TGAptSurface();

    // Query the elevation of a point, return -9999 if out of range.
    // This routine makes a simplistic assumption that X,Y space is
    // proportional to u,v space on the nurbs surface which it isn't.
    double query( double lon_deg, double lat_deg );

    // Query the elevation of a point, return -9999 if out of range.
    // This routine incorporates a complex solver that attempts to
    // find the exact u,v coordinates correspondinging to the
    // requested lat, lon.  But there seems to be problems with my
    // solver routine that I haven't tracked down yet so I'm testing
    // the _dumb version above.
    double query_solver( double lon_deg, double lat_deg );
};


#endif // _APT_SURFACE_HXX
