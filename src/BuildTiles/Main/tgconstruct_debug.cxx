// tgconstruct_debug.cxx -- Class toimplement construct debug
//
// Written by Curtis Olson, started May 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: construct.cxx,v 1.4 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <sstream>

#include <simgear/debug/logstream.hxx>

#include "tgconstruct.hxx"

using std::string;

void TGConstruct::set_debug( std::string path, std::vector<string> a_defs, std::vector<string> s_defs )
{
    debug_path = path;

    debug_area_defs = a_defs;
    debug_shape_defs = s_defs;
}

void TGConstruct::get_debug( void )
{
    // clear out any previous entries
    debug_areas.clear();
    debug_shapes.clear();

    /* Find any ids for our tile */
    for (unsigned int i = 0; i < debug_area_defs.size(); ++i) {
        string dsd     = debug_area_defs[i];
        size_t d_pos   = dsd.find(":");
        string tile    = dsd.substr(0, d_pos);

        if ( tile == bucket.gen_index_str() ) {
            dsd.erase(0, d_pos + 1);

            if ( dsd == "all" ) {
                debug_all = true;
            } else {
                std::stringstream ss(dsd);
                int idx;
                while (ss >> idx)
                {
                    SG_LOG(SG_GENERAL, SG_ALERT, "Adding debug area " << idx);

                    debug_areas.push_back(idx);

                    if (ss.peek() == ',')
                        ss.ignore();
                }
            }
        }
    }

    for (unsigned int i = 0; i < debug_shape_defs.size(); ++i) {
        string dsd     = debug_shape_defs[i];
        size_t d_pos   = dsd.find(":");
        string tile    = dsd.substr(0, d_pos);

        if ( tile == bucket.gen_index_str() ) {
            dsd.erase(0, d_pos + 1);

            if ( dsd == "all" ) {
                debug_all = true;
            } else {
                std::stringstream ss(dsd);
                int idx;
                while (ss >> idx)
                {
                    SG_LOG(SG_GENERAL, SG_ALERT, "Adding debug shape " << idx);

                    debug_shapes.push_back(idx);

                    if (ss.peek() == ',')
                        ss.ignore();
                }
            }
        }
    }
}

bool TGConstruct::IsDebugShape( unsigned int id )
{
    bool is_debug = false;

    /* Check global flag */
    if ( debug_all ) {
        is_debug = true;
    } else {
        for (unsigned int i=0; i<debug_shapes.size(); i++) {
            if ( debug_shapes[i] == id ) {
                is_debug = true;
                break;
            }
        }
    }

    return is_debug;
}

bool TGConstruct::IsDebugArea( unsigned int area )
{
    bool is_debug = false;

    /* Check global flag */
    if ( debug_all ) {
        is_debug = true;
    } else {
        for (unsigned int i=0; i<debug_areas.size(); i++) {
            if ( debug_areas[i] == area ) {
                is_debug = true;
                break;
            }
        }
    }

    return is_debug;
}