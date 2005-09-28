// names.hxx -- process shapefiles names
//
// Written by Curtis Olson, started February 1999.
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
// $Id$
 

#ifndef _NAMES_HXX
#define _NAMES_HXX


#include <simgear/compiler.h>

#include STL_STRING

SG_USING_STD(string);


// Posible shape file types.  Note the order of these is important and
// defines the priority of these shapes if they should intersect.  The
// smaller the number, the higher the priority.
enum AreaType {
    SomeSortOfArea = 0,
    HoleArea,			// Leave area completely empty
    AirportArea,
    FreewayArea,
    RoadArea,
    RailroadArea,
    PondArea,
    LakeArea,
    DryLakeArea,
    IntLakeArea,
    ReservoirArea,
    IntReservoirArea,
    StreamArea,
    IntStreamArea,
    CanalArea,
    GlacierArea,		// Any solid ice/snow
    PackIceArea,		// Water with ice packs
    OceanArea,
    UrbanArea,			// Densely-populated city or large town
    TownArea,			// Small town or village
    FloodLandArea,		// Land subject to flooding
    BogArea,			// Bog
    MarshArea,			// Marshland or swamp
    SandArea,			// Sand-covered area
    LavaArea,			// Lava-covered area

    // USGS Land Covers
    // These are low-priority, since known polygons should always win.

    BuiltUpCover,		// Urban and Built-Up Land
    DryCropPastureCover,	// Dryland Cropland and Pasture
    IrrCropPastureCover,	// Irrigated Cropland and Pasture
    MixedCropPastureCover,	// Mixed Dryland/Irrigated Cropland and Pasture
    CropGrassCover,		// Cropland/Grassland Mosaic
    CropWoodCover,		// Cropland/Woodland Mosaic
    GrassCover,			// Grassland
    ShrubCover,			// Shrubland
    ShrubGrassCover,		// Mixed Shrubland/Grassland
    SavannaCover,		// Savanna
    DeciduousBroadCover,	// Deciduous Broadleaf Forest
    DeciduousNeedleCover,	// Deciduous Needleleaf Forest
    EvergreenBroadCover,	// Evergreen Broadleaf Forest
    EvergreenNeedleCover,	// Evergreen Needleleaf Forest
    MixedForestCover,		// Mixed Forest
    WaterBodyCover,		// Water Bodies
    HerbWetlandCover,		// Herbaceous Wetland
    WoodedWetlandCover,		// Wooded Wetland
    BarrenCover,		// Barren or Sparsely Vegetated
    HerbTundraCover,		// Herbaceous Tundra
    WoodedTundraCover,		// Wooded Tundra
    MixedTundraCover,		// Mixed Tundra
    BareTundraCover,		// Bare Ground Tundra
    SnowCover,			// Snow or Ice

    IslandArea,			// any island area not covered otherwise
    DefaultArea,		// any land area not covered otherwise

    VoidArea,
    NullArea,
    UnknownArea
};


// return area type from text name
AreaType get_area_type( const string &area );

// return text form of area name
string get_area_name( AreaType area );


#endif // _NAMES_HXX

