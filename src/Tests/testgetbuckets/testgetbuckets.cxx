// main.cxx -- testGetBuckets
//
// Written by Peter Sadrozinski, started Dec 2014.
//
// Copyright (C) 2015  Peter Sadrozinski
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
#include <cstdio>
#include <iostream>
#include <simgear/bucket/newbucket.hxx>

int main(int argc, char* argv[])
{
    double minx, miny, maxx, maxy;
    
    if ( argc == 5 ) {
        std::sscanf( argv[1], "%lf", &minx );
        std::sscanf( argv[2], "%lf", &miny );
        std::sscanf( argv[3], "%lf", &maxx );
        std::sscanf( argv[4], "%lf", &maxy );
    
        SGGeod min = SGGeod::fromDeg( minx, miny );
        SGGeod max = SGGeod::fromDeg( maxx, maxy );
        
        std::vector<SGBucket> buckets;
        sgGetBuckets( min, max, buckets );
        
        for ( unsigned int i=0; i<buckets.size(); i++ ) {
            std::cout << buckets[i].gen_index_str() << std::endl;
        }
    }
    
    return 0;
}