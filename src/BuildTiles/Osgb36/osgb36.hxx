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

#include <iostream.h>
#include <math.h>
#include <simgear/math/point3d.hxx>


//*******************************************************************
//
// NOTE All functions which take or return Latitude and Longitude
//	take and return degrees.  Conversion to and from radians
//	is performed internaly.  XYZ coordinates and ellipsoid
//	heights are returned and required in meters.
//
//*******************************************************************


//Convert OSGB36 Lat and Lon to OSGB36 Eastings and Northings (Grid Reference)
Point3D ConvertLatLonToEastingsNorthings(Point3D LatLon);

//Convert OSGB36 Eastings and Northings to OSGB36 Lat and Lon
Point3D ConvertEastingsNorthingsToLatLon(Point3D GridRef);

//Convert OSGB36 coordinates from polar to cartesian (x,y,z) form
Point3D ConvertAiry1830PolarToCartesian(Point3D LatLon);

//Convert WGS84 coordinates from polar to cartesian (x,y,z) form
Point3D ConvertGRS80PolarToCartesian(Point3D LatLon);

//Convert OSGB36 coordinates from cartesian (x,y,z) to polar form
Point3D ConvertAiry1830CartesianToPolar(Point3D XYZCoord);

//Convert WGS84 coordinates from cartesian (x,y,z) to polar form
Point3D ConvertGRS80CartesianToPolar(Point3D XYZCoord);

//Transform a point in WGS84 cartesian coordinates to OSGB36 cartesian coordinates
//Uses the Helmert Transformation with coefficients from www.gps.gov.org
//Only accurate to around 5m since OSGB36 is an inhomogenous TRF
Point3D ConvertWGS84ToOSGB36(Point3D WGS84);

//Transform a point in OSGB36 cartesian coordinates to WGS84 cartesian coordinates
//Uses the Helmert Transformation with coefficients from www.gps.gov.org
//Only accurate to around 5m since OSGB36 is an inhomogenous TRF
Point3D ConvertOSGB36ToWGS84(Point3D OSGB36);

