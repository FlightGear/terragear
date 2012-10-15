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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.

#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>

#include <fstream>
#include <vector>
#include <map>
#include <string>

#include <stdlib.h>

#include "priorities.hxx"

using std::ifstream;
using std::string;
using std::map;
using std::vector;

typedef enum {
    Hole,
    Landmass,
    Island,
    Ocean,
    Lake,
    Stream,
    Road,
    Other
} AreaKind;

typedef struct {
    string name;
    AreaKind kind;
} area_type_descriptor;

typedef vector<area_type_descriptor> area_type_list;
typedef map<string, AreaType> area_name_map;

static area_type_list area_types;
static area_name_map area_names;
static AreaType default_area_type;
static AreaType sliver_target_area_type;

int load_area_types( const std::string& filename ) {
    ifstream in ( filename.c_str() );
    
    if ( ! in ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Unable to open priorities file " << filename);
        return 0;
    }
    SG_LOG(SG_GENERAL, SG_ALERT, "Priorities file is " << filename);
    
    in >> skipcomment;
    string sliver_area_name, default_area_name;
    
    in >> default_area_name;
    in >> skipcomment;
    in >> sliver_area_name;
    in >> skipcomment;
    while ( !in.eof() ) {
        area_type_descriptor descriptor;
        string type;
        descriptor.kind = Other;
        in >> descriptor.name;
        in >> type;
        if ( type=="hole" ) {
            descriptor.kind = Hole;
        } else if ( type=="landmass" ) {
            descriptor.kind = Landmass;
        } else if ( type=="island" ) {
            descriptor.kind = Island;
        } else if ( type=="ocean" ) {
            descriptor.kind = Ocean;
        } else if ( type=="lake" ) {
            descriptor.kind = Lake;
        } else if ( type=="stream" ) {
            descriptor.kind = Stream;
        } else if ( type=="road" ) {
            descriptor.kind = Road;
        } else if ( type=="other" ) {
            descriptor.kind = Other;
        }
        AreaType index = (AreaType)area_types.size();
        area_types.push_back(descriptor);
        area_names[descriptor.name]=index;
        SG_LOG(SG_GENERAL, SG_INFO, "  " << index << " " << descriptor.name << " " << descriptor.kind);
        in >> skipcomment;
    }
    
    in.close();
    
    sliver_target_area_type = get_area_type( sliver_area_name );
    default_area_type = get_area_type( default_area_name );
    
    return 1;
}
// return area type from text name
AreaType 
get_area_type (const string &area) {
    area_name_map::const_iterator it = area_names.find(area);
    if (it != area_names.end()) {
        return it->second;
    } else {
	SG_LOG(SG_GENERAL, SG_ALERT, "unknown area = '" << area << "'");
	exit(-1);
    }
}


static area_type_descriptor& get_area_descriptor( AreaType area ) {
    if ( 0<=area || area < area_types.size() ) {
        return area_types[area];
    } else {
	SG_LOG(SG_GENERAL, SG_ALERT, "unknown area code = " << (int)area);
	exit(-1);
    }
}

// return text from of area name
string get_area_name( AreaType area ) {
    return get_area_descriptor( area ).name;
}

bool is_hole_area( AreaType area ) {
    return get_area_descriptor( area ).kind==Hole;
}

bool is_water_area( AreaType area ) {
    const AreaKind kind = get_area_descriptor( area ).kind;
    return (kind==Ocean || kind==Lake || kind==Stream);
}

bool is_landmass_area( AreaType area ) {
    const AreaKind kind = get_area_descriptor( area ).kind;
    return (kind==Landmass);
}

bool is_island_area( AreaType area ) {
    const AreaKind kind = get_area_descriptor( area ).kind;
    return (kind==Island);
}

bool is_lake_area( AreaType area ) {
    const AreaKind kind = get_area_descriptor( area ).kind;
    return (kind==Lake);
}

bool is_stream_area( AreaType area ) {
    const AreaKind kind = get_area_descriptor( area ).kind;
    return (kind==Stream);
}

bool is_road_area( AreaType area ) {
    const AreaKind kind = get_area_descriptor( area ).kind;
    return (kind==Road);
}

bool is_ocean_area( AreaType area ) {
    const AreaKind kind = get_area_descriptor( area ).kind;
    return (kind==Ocean);
}

AreaType get_sliver_target_area_type() {
    return sliver_target_area_type;
}

AreaType get_default_area_type() {
    return default_area_type;
}
