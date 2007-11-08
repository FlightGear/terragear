// build.hxx -- routines to build polygon model of an airport from the runway
//              definition
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: build.hxx,v 1.10 2004-11-19 22:25:49 curt Exp $
//


#ifndef _BUILD_HXX
#define _BUILD_HXX


#include <list>

#include "global.hxx"
#include "point2d.hxx"


// build 3d airport
void build_airport( string airport_id, float alt_m,
                    string_list& runways_raw,
                    string_list& beacons_raw,
                    string_list& towers_raw,
                    string_list& windsocks_raw,                    
                    const string& root,
                    const string_list& elev_src );



#endif // _BUILD_HXX


