// osgb36.hxx -- Routines to convert UK OS coordinates to and
//               from world WGS84 coordinates.
//
// Written by David Luff, started December 2000.
//
// Copyright (C) 2000  David C. Luff  - david.luff@nottingham.ac.uk
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

#include <simgear/math/point3d.hxx>

//*******************************************************************
//
// NOTE All functions which take or return Latitude and Longitude
//	take and return degrees.  XYZ coordinates and ellipsoid
//	heights are returned and required in meters.
//
//*******************************************************************

// Convert WGS84 Lat/Lon (degrees) to OSGB36 Eastings and Northings (meters)
Point3D WGS84ToOSGB36(Point3D p);

// Convert OSGB36 Eastings and Northings (meters) to WGS84 Lat/Lon (degrees)
Point3D OSGB36ToWGS84(Point3D p);
