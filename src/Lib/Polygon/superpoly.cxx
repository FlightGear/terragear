// superpoly.cxx -- Manage all aspects of a rendered polygon
//
// Written by Curtis Olson, started June 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: superpoly.cxx,v 1.6 2004-11-19 22:25:50 curt Exp $


#include "superpoly.hxx"


// Constructor
TGSuperPoly::TGSuperPoly() :
    flag( "" )
{
}


// Destructor
TGSuperPoly::~TGSuperPoly()
{
}

// erase the "super" polygon
void TGSuperPoly::erase()
{
    material = "";
    poly.erase();
    normals.erase();
    texcoords.erase();
    tris.erase();
    face_normals.clear();
}
