// usgs.cxx -- manage USGS mapping
//
// Written by Ralf Gerlich, started December 2009
//
// Copyright (C) 2009  Ralf Gerlich  - ralf.gerlich <at> custom-scenery <dot> org
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
#include <simgear/misc/sgstream.hxx>

#include <fstream>
#include <string>
#include <vector>

#include "usgs.hxx"

using std::ifstream;
using std::string;
using std::vector;

static vector<AreaType> usgs_map;

int load_usgs_map( const std::string& filename ) {
    ifstream in ( filename.c_str() );
    
    if ( ! in ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Unable to open file " << filename);
        return 0;
    }
    
    in >> skipcomment;
    while ( !in.eof() ) {
    	string name;
    	in >> name;
    	usgs_map.push_back( get_area_type( name ) );
        in >> skipcomment;
    }
    
    in.close();
    
    return 1;
}

// Translate USGS land cover values into TerraGear area types.
AreaType translateUSGSCover (int usgs_value)
{
    if ( 0<usgs_value && usgs_value<usgs_map.size() ) {
        return usgs_map[usgs_value-1];
    } else {
        return get_default_area_type();
    }
}

