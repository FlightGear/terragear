// uk.hxx -- Routines to determine whether a point is within the UK.
//
// Written by David Luff, started Janurary 2001.
//
// Copyright (C) 2001  David C. Luff  - david.luff@nottingham.ac.uk
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

#include <Geometry/point3d.hxx>

//Returns true if a point is within the mainland UK, excluding
//Northern Ireland.  Requires lat and lon to be passed in degrees.
bool isInUK(Point3D p);
