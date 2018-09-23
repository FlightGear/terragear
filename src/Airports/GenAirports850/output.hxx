// output.hxx -- routines to output index files of an airport
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999 - 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
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


#ifndef _OUTPUT_HXX
#define _OUTPUT_HXX

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

// update index file (list of objects to be included in final scenery build)
void write_index_object( const std::string& base, const SGBucket& b, const std::string& name );

// update index file (list of shared objects to be included in final scenery build)
void write_index_object_shared( const std::string &base, const SGBucket &b,
                         const SGGeod &p, const std::string& name,
                         const double &heading );

// update index file (list of shared objects to be included in final scenery build)
void write_index_object_sign( const std::string &base, const SGBucket &b,
                        const SGGeod &p, const std::string& sign,
                        const double &heading, const int &size );

// purge the existing index file when it already exists
void truncate_index_file( const std::string& fileName );

// record index files that have been cleaned
std::set<std::string> cleanIndexFiles;

#endif
