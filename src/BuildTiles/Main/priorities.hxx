// priorities.hxx -- manage material priorities
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: names.hxx,v 1.14 2005-09-28 16:43:18 curt Exp $
 

#ifndef _PRIORITIES_HXX
#define _PRIORITIES_HXX

#include <map>
#include <string>

#include <simgear/compiler.h>

#include <terragear/tg_polygon.hxx>

class TGAreaDefinition {
public:
    TGAreaDefinition( const std::string& n, const std::string& c, unsigned int p ) :
        name(n),
        category(c)
    {
        priority = p;
        smooth_method = 0;
        layered = false;
        default_layer = 0;
    };

    std::string const& GetName() const {
        return name;
    }

    unsigned int GetPriority() const {
        return priority;
    }

    std::string const& GetCategory() const {
        return category;
    }

private:
    std::string  name;
    unsigned int priority;
    std::string  category;

    // future improvements
    unsigned int smooth_method;
    tgTexMethod  texture_method;
    bool         layered;
    unsigned int default_layer;
};

typedef std::vector<TGAreaDefinition> area_definition_list;
typedef area_definition_list::const_iterator area_definition_iterator;

class TGAreaDefinitions {
public:
    TGAreaDefinitions() :
        sliver_area_name("<unnamed>")
    {
        sliver_area_priority = 0;
    };

    int init( const std::string& filename );
    unsigned int size() const {
        return area_defs.size();
    }

    bool is_hole_area( unsigned int p ) const {
        if ( area_defs[p].GetCategory() == "hole" ) {
            return true;
        } else {
            return false;
        }
    }

    bool is_landmass_area( unsigned int p ) const {
        if (( area_defs[p].GetCategory() == "landmass" ) || 
	    ( area_defs[p].GetCategory() == "other" ) ||
            ( area_defs[p].GetCategory() == "cliff" )) {
            return true;
        } else {
            return false;
        }
    }

    bool is_island_area( unsigned int p ) const {
        if ( area_defs[p].GetCategory() == "island" ) {
            return true;
        } else {
            return false;
        }
    }

    bool is_road_area( unsigned int p ) const {
        if ( area_defs[p].GetCategory() == "road" ) {
            return true;
        } else {
            return false;
        }
    }

    bool is_water_area( unsigned int p ) const {
        if ( ( area_defs[p].GetCategory() == "ocean" ) ||
             ( area_defs[p].GetCategory() == "lake" ) ){
            return true;
        } else {
            return false;
        }
    }

    bool is_lake_area( unsigned int p ) const {
        if ( area_defs[p].GetCategory() == "lake" ) {
            return true;
        } else {
            return false;
        }
    }

    bool is_stream_area( unsigned int p ) const {
        if ( area_defs[p].GetCategory() == "stream" ) {
            return true;
        } else {
            return false;
        }
    }

    bool is_ocean_area( unsigned int p ) const {
        if ( area_defs[p].GetCategory() == "ocean" ) {
            return true;
        } else {
            return false;
        }
    }

    bool is_cliff_area( unsigned int p ) const {
	    if (area_defs[p].GetCategory() == "cliff" ) {
		    return true;
            } else {
		    return false;
            }
    }	

    std::string const& get_area_name( unsigned int p ) const {
        return area_defs[p].GetName();
    }

    unsigned int get_area_priority( const std::string& name ) const {
        for (unsigned int i=0; i < area_defs.size(); i++) {
            if ( area_defs[i].GetName() == name ) {
                return i;
            }
        }

        SG_LOG(SG_GENERAL, SG_ALERT, "No area named " << name);
        exit(0);
        return 0;
    }


    std::string const& get_sliver_area_name( void ) const {
        return sliver_area_name;
    }

    unsigned int get_sliver_area_priority( void ) const {
        return sliver_area_priority;
    }


private:
    area_definition_list area_defs;
    std::string  sliver_area_name;
    unsigned int sliver_area_priority;
};

#endif // _PRIORITIES_HXX
