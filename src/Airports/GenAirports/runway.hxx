// runway.hxx -- class to store runway info
//
// Written by Curtis Olson, started November 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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


#ifndef _RUNWAY_HXX
#define _RUNWAY_HXX


#include <string>
#include <vector>

#include <simgear/math/point3d.hxx>

#include <Polygon/polygon.hxx>


struct FGRunway {
    string rwy_no;

    double lon;
    double lat;
    double heading;
    double length;
    double width;
    
    string surface_flags;
    string end1_flags;
    string end2_flags;

    FGPolygon threshold;
    FGPolygon tens, tens_margin, ones, ones_margin;
    FGPolygon letter, letter_margin_left, letter_margin_right;
    FGPolygon pre_td_zone;
    FGPolygon td3_zone, td2_zone, td1a_zone, td1b_zone;
    FGPolygon aim_point;
};


typedef vector < FGRunway > runway_list;
typedef runway_list::iterator runway_list_iterator;
typedef runway_list::const_iterator const_runway_list_iterator;


// generate an area for a runway (return result points in degrees)
FGPolygon gen_runway_area( const FGRunway& runway, 
			   double len_scale = 1.0,
			   double width_scale = 1.0 );

// generate an area for half a runway
FGPolygon gen_runway_w_mid( const FGRunway& runway,
			    double len_scale = 1.0,
			    double width_scale = 1.0 );


#endif // _RUNWAY_HXX
