// contour_tree.hxx -- routines for building a contour tree showing
//                     which contours are inside if which contours
//
// Written by Curtis Olson, started June 2000.
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: contour_tree.hxx,v 1.5 2004-11-19 22:25:50 curt Exp $


#ifndef _CONTOUR_TREE_HXX
#define _CONTOUR_TREE_HXX


#ifndef __cplusplus
# error This library requires C++
#endif


#include <simgear/compiler.h>

#include <vector>
#include <cstddef>

// forward declaration
class TGContourNode;

typedef std::vector < TGContourNode * > contour_kids;
typedef contour_kids::iterator contour_kids_iterator;
typedef contour_kids::const_iterator const_contour_kids_iterator;


// a simple class for building a contour tree for a polygon.  The
// contour tree shows the hierarchy of which contour is inside which
// contour.

class TGContourNode {

private:

    int contour_num;        // -1 for the root node
    contour_kids kids;

public:

    TGContourNode();
    TGContourNode( int n );

    ~TGContourNode();

    inline int get_contour_num() const { return contour_num; }
    inline void set_contour_num( int n ) { contour_num = n; }

    inline int get_num_kids() const { return kids.size(); }
    inline TGContourNode *get_kid( int n ) const { return kids[n]; }
    inline void add_kid( TGContourNode *kid ) { kids.push_back( kid ); }
    inline void remove_kid( int n ) {
    // cout << "kids[" << n << "] = " << kids[n] << endl;
    delete kids[n];
    kids[n] = NULL;
    }
};


#endif // _CONTOUR_TREE_HXX


