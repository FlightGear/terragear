// uk.cxx -- Routines to determine whether a point is within the UK.
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#include "uk.hxx"

//Returns true if a point is within the mainland UK, excluding
//Northern Ireland.  Requires lat and lon to be passed in degrees.
bool isInUK(Point3D p)
{
   bool inUK = false;
   double lat = p.lat();
   double lon = p.lon();
   //The entire UK (excepting a small portion of Northern Ireland which we arn't doing anyway)
   //falls within the box of -8 -> 2 deg lon, and 49 -> 60 deg lat
   //We'll check for within this box first, and if so then we'll refine
   //further to avoid bits of France and Ireland.
   if((lat >= 49.0) && (lat <= 60.0) && (lon >= -8.0) && (lon <= 2.0))
   {
      //we might be within the UK
      //set inUK = true and then test for the exceptions
      inUK = true;

      //check for France (Normandy/Calais)
      if((lon >= -2.0) && (lat <=50.0))
	 //Normandy
	 inUK = false;
      if((lon > 1.0) && (lat <= 51.0))
	 //Calais area
	 inUK = false;

      //Check for Ireland
      if((lon <= -6.0) && (lat > 51.0) && (lat <= 55.5))
	 //Ireland
	 inUK = false;
      //Check for last bit of NI
      if((lat > 54.0) && (lat <= 55.1) && (lon <= -5.3333))
	 inUK = false;

   }
   else
      inUK = false;

   return(inUK);
}
