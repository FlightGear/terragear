// lights.hxx -- Generate runway lighting
//
// Written by Curtis Olson, started February 2002.
//
// Copyright (C) 2002  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: lights.hxx,v 1.8 2004-11-19 22:25:49 curt Exp $
//


#ifndef _RWY_LIGHTS_HXX
#define _RWY_LIGHTS_HXX


#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>

#include "runway.hxx"
#include "texparams.hxx"


// generate runway lighting
void gen_runway_lights( const TGRunway& rwy_info, float alt_m,
                        superpoly_list &lights, TGPolygon *apt_base );

// generate taxiway lighting
void gen_taxiway_lights( const TGRunway& taxiway_info, float alt_m,
                         superpoly_list &lights );

// generate light objects
void gen_airport_lightobj( const TGLightobj& rwy_light, float alt_m, superpoly_list &lights );

#endif // _RWY_LIGHTS_HXX
