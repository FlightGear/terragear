// names.hxx -- process shapefiles names
//
// Written by Curtis Olson, started February 1999.
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
 

#ifndef _NAMES_HXX
#define _NAMES_HXX


#include <simgear/compiler.h>

#include STL_STRING

FG_USING_STD(string);


// Posible shape file types.  Note the order of these is important and
// defines the priority of these shapes if they should intersect.  The
// smaller the number, the higher the priority.
enum AreaType {
    SomeSortOfArea    = 0,
    HoleArea          = 1,
    PondArea          = 2,
    LakeArea          = 3,
    DryLakeArea       = 4,
    IntLakeArea       = 5,
    ReservoirArea     = 6,
    IntReservoirArea  = 7,
    StreamArea        = 8,
    CanalArea         = 9,
    GlacierArea       = 10,
    OceanArea         = 11,
    UrbanArea         = 12,
    MarshArea         = 13,

    // USGS Land Covers
    // These are low-priority, since known polygons should always win.

    BuiltUpCover      = 14,	// Urban and Built-Up Land
    DryCropPastureCover = 15,	// Dryland Cropland and Pasture
    IrrCropPastureCover = 16,	// Irrigated Cropland and Pasture
    MixedCropPastureCover = 17,	// Mixed Dryland/Irrigated Cropland and Pasture
    CropGrassCover    = 18,	// Cropland/Grassland Mosaic
    CropWoodCover = 19,		// Cropland/Woodland Mosaic
    GrassCover = 20,		// Grassland
    ShrubCover = 21,		// Shrubland
    ShrubGrassCover = 22,	// Mixed Shrubland/Grassland
    SavannaCover = 23,		// Savanna
    DeciduousBroadCover = 24,	// Deciduous Broadleaf Forest
    DeciduousNeedleCover = 25,	// Deciduous Needleleaf Forest
    EvergreenBroadCover = 26,	// Evergreen Broadleaf Forest
    EvergreenNeedleCover = 27,	// Evergreen Needleleaf Forest
    MixedForestCover = 28,	// Mixed Forest
    WaterBodyCover = 29,	// Water Bodies
    HerbWetlandCover = 30,	// Herbaceous Wetland
    WoodedWetlandCover = 31,	// Wooded Wetland
    BarrenCover = 32,		// Barren or Sparsely Vegetated
    HerbTundraCover = 33,	// Herbaceous Tundra
    WoodedTundraCover = 34,	// Wooded Tundra
    MixedTundraCover = 35,	// Mixed Tundra
    BareTundraCover = 36,	// Bare Ground Tundra
    SnowCover = 37,		// Snow or Ice

    IslandArea        = 38,
    DefaultArea       = 39,
    VoidArea          = 9997,
    NullArea          = 9998,
    UnknownArea       = 9999
};


// return area type from text name
AreaType get_area_type( const string &area );

// return text form of area name
string get_area_name( AreaType area );


#endif // _NAMES_HXX

