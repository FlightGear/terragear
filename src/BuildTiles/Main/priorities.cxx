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
#include <simgear/io/iostreams/sgstream.hxx>

#include <fstream>
#include <vector>
#include <map>
#include <string>

#include <stdlib.h>

#include "priorities.hxx"

int TGAreaDefinitions::init( const std::string& filename )
{
    std::ifstream in ( filename.c_str() );
    unsigned int cur_priority = 0;

    if ( ! in ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Unable to open priorities file " << filename);
        return 0;
    }
    SG_LOG(SG_GENERAL, SG_ALERT, "Priorities file is " << filename);

    in >> skipcomment;
    std::string sa_name, da_name;

    in >> da_name;
    in >> skipcomment;
    in >> sa_name;
    in >> skipcomment;

    std::string name, category;

    while ( !in.eof() ) {
        in >> name;
        in >> category;
        in >> skipcomment;

        if ( name == sa_name ) {
            sliver_area_name     = sa_name;
            sliver_area_priority = cur_priority;
        }

        area_defs.push_back( TGAreaDefinition( name, category, cur_priority++ ) );
    }
    in.close();

    return 0;
}