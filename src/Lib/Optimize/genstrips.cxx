// genstrips.cxx -- Optimize a collection of individual triangles into
//                  strips.
//
// Written by Curtis Olson, started May 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: genstrips.cxx,v 1.3 2004-11-19 22:25:50 curt Exp $


#include <simgear/compiler.h>

#include "genstrips.hxx"


// make sure the list is expanded at least to hold "n" and then push
// "i" onto the back of the "n" list.
static void add_and_expand( opt_list& by_node, int n, int i ) {
    int_list empty;

    int size = (int)by_node.size();
    if ( size > n ) {
	// ok
    } else {
	// cout << "capacity = " << by_node.capacity() << endl;
	// cout << "size = " << size << "  n = " << n
	//      << " need to push = " << n - size + 1 << endl;
	for ( int i = 0; i < n - size + 1; ++i ) {
	    by_node.push_back(empty);
	}
    }

    by_node[n].push_back(i);
}


// generate a set of tri-strips from a random collection of triangles.
// Note, the triangle verticies are integer indices into a vertex list
// and not actual vertex data.
opt_list tgGenStrips( const triele_list tris ) {

    triele_list remaining = tris;
    opt_list by_node;
    int i;

    while ( remaining.size() ) {
	// traverse the triangle list and for each node, build a list of
	// triangles that attach to it.

	// cout << "building by_node list" << endl;
	by_node.clear();

	for ( i = 0; i < (int)remaining.size(); ++i ) {
	    int n1 = remaining[i].get_n1();
	    int n2 = remaining[i].get_n2();
	    int n3 = remaining[i].get_n3();

	    add_and_expand( by_node, n1, i );
	    add_and_expand( by_node, n2, i );
	    add_and_expand( by_node, n3, i );
	}

	// find the least connected triangle (this is a popular
	// heuristic for starting finding a good set of triangle
	// strips.)

	int min_size = 32768;
	int min_tri = -1;
	for ( i = 0; i < (int)remaining.size(); ++i ) {
	    int n1 = remaining[i].get_n1();
	    int n2 = remaining[i].get_n2();
	    int n3 = remaining[i].get_n3();
	    int metric = 0;

	    metric += by_node[n1].size();
	    metric += by_node[n2].size();
	    metric += by_node[n3].size();

	    if ( metric < min_size ) {
		min_size = metric;
		min_tri = i;
	    }
	}
    }

    return by_node;
}


