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

    // USGS Land Covers
    // These are low-priority, since known polygons should always win.

    BuiltUpCover      = 13,	// Urban and Built-Up Land
    DryCropPastureCover = 14,	// Dryland Cropland and Pasture
    IrrCropPastureCover = 15,	// Irrigated Cropland and Pasture
    MixedCropPastureCover = 16,	// Mixed Dryland/Irrigated Cropland and Pasture
    CropGrassCover    = 17,	// Cropland/Grassland Mosaic
    CropWoodCover = 18,		// Cropland/Woodland Mosaic
    GrassCover = 19,		// Grassland
    ShrubCover = 20,		// Shrubland
    ShrubGrassCover = 21,	// Mixed Shrubland/Grassland
    SavannaCover = 22,		// Savanna
    DeciduousBroadCover = 23,	// Deciduous Broadleaf Forest
    DeciduousNeedleCover = 24,	// Deciduous Needleleaf Forest
    EvergreenBroadCover = 25,	// Evergreen Broadleaf Forest
    EvergreenNeedleCover = 26,	// Evergreen Needleleaf Forest
    MixedForestCover = 27,	// Mixed Forest
    WaterBodyCover = 28,	// Water Bodies
    HerbWetlandCover = 29,	// Herbaceous Wetland
    WoodedWetlandCover = 30,	// Wooded Wetland
    BarrenCover = 31,		// Barren or Sparsely Vegetated
    HerbTundraCover = 32,	// Herbaceous Tundra
    WoodedTundraCover = 33,	// Wooded Tundra
    MixedTundraCover = 34,	// Mixed Tundra
    BareTundraCover = 35,	// Bare Ground Tundra
    SnowCover = 36,		// Snow or Ice

    MarshArea         = 37,

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

