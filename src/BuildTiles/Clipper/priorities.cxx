// priorities.cxx -- manage area type priorities
//
// Written by Ralf Gerlich
//
// Copyright (C) 2008  Ralf Gerlich  - ralf.gerlich <at> custom-scenery <dot> org
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
 
#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>

#include <map>
#include <string>

#include "priorities.hxx"

using std::string;
using std::map;

typedef map<AreaType, string> area_type_map;
typedef map<string, AreaType> area_name_map;

static area_type_map area_types;
static area_name_map area_names;


inline static void set_area (const string &name, AreaType type)
{
  area_types[type] = name;
  area_names[name] = type;
}


static bool _initialized = false;


inline static void init ()
{
  if (_initialized)
    return;

  set_area("SomeSort", SomeSortOfArea);
  set_area("Hole", HoleArea);
  set_area("Airport", AirportArea);
  set_area("Island", IslandArea);
  set_area("Pond", PondArea);
  set_area("Swamp or Marsh", MarshArea);
  set_area("Marsh", MarshArea);
  set_area("Littoral", LittoralArea);
  set_area("Bog", BogArea);
  set_area("Sand", SandArea);
  set_area("Lava", LavaArea);
  set_area("FloodLand", FloodLandArea);
  set_area("Lake", LakeArea);
  set_area("Lake   Dry", DryLakeArea);
  set_area("DryLake", DryLakeArea);
  set_area("Lake   Intermittent", IntLakeArea);
  set_area("IntermittentLake", IntLakeArea);
  set_area("Reservoir", ReservoirArea);
  set_area("Reservoir   Intermittent", IntReservoirArea);
  set_area("IntermittentReservoir", IntReservoirArea);
  set_area("Freeway", FreewayArea);
  set_area("Road", RoadArea);
  set_area("Railroad", RailroadArea);
  set_area("Stream", StreamArea);
  set_area("IntermittentStream", IntStreamArea);
  set_area("Canal", CanalArea);
  set_area("Glacier", GlacierArea);
  set_area("PackIce", PackIceArea);
  set_area("PolarIce", PolarIceArea);
  set_area("Urban", UrbanArea);
  set_area("Town", TownArea);
  set_area("BuiltUpCover", BuiltUpCover);
  set_area("DryCropPastureCover", DryCropPastureCover);
  set_area("IrrCropPastureCover", IrrCropPastureCover);
  set_area("MixedCropPastureCover", MixedCropPastureCover);
  set_area("CropGrassCover", CropGrassCover);
  set_area("CropWoodCover", CropWoodCover);
  set_area("GrassCover", GrassCover);
  set_area("ShrubCover", ShrubCover);
  set_area("ShrubGrassCover", ShrubGrassCover);
  set_area("SavannaCover", SavannaCover);
  set_area("DeciduousBroadCover", DeciduousBroadCover);
  set_area("DeciduousNeedleCover", DeciduousNeedleCover);
  set_area("EvergreenBroadCover", EvergreenBroadCover);
  set_area("EvergreenNeedleCover", EvergreenNeedleCover);
  set_area("MixedForestCover", MixedForestCover);
  set_area("WaterBodyCover", WaterBodyCover);
  set_area("HerbWetlandCover", HerbWetlandCover);
  set_area("WoodedWetlandCover", WoodedWetlandCover);
  set_area("BarrenCover", BarrenCover);
  set_area("HerbTundraCover", HerbTundraCover);
  set_area("WoodedTundraCover", WoodedTundraCover);
  set_area("MixedTundraCover", MixedTundraCover);
  set_area("BareTundraCover", BareTundraCover);
  set_area("SnowCover", SnowCover);
  set_area("Default", DefaultArea);
  set_area("Bay  Estuary or Ocean", OceanArea);
  set_area("Ocean", OceanArea);
  set_area("Void Area", VoidArea);
  set_area("Null", NullArea);

  _initialized = true;
}


// return area type from text name
AreaType 
get_area_type (const string &area) {
    init();
    area_name_map::const_iterator it = area_names.find(area);
    if (it != area_names.end()) {
        return it->second;
    } else {
	SG_LOG(SG_GENERAL, SG_WARN, "unknown area = '" << area << "'");
	// SG_LOG(SG_GENERAL, SG_DEBUG, "area = " << area);
	// for ( int i = 0; i < area.length(); i++ ) {
	//  SG_LOG(SG_GENERAL, SG_DEBUG, i << ") " << (int)area[i]);
	// }
	return UnknownArea;
    }
}


// return text from of area name
string get_area_name( AreaType area ) {
    init();
    area_type_map::const_iterator it = area_types.find(area);
    if (it != area_types.end()) {
        return it->second;
    } else {
	SG_LOG(SG_GENERAL, SG_WARN, "unknown area code = " << (int)area);
	return "Unknown";
    }
}


