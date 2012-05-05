// trieles.hxx -- "Triangle" element management class
//
// Written by Curtis Olson, started March 1999.
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
// $Id: trieles.hxx,v 1.4 2004-11-19 22:25:50 curt Exp $


#ifndef _TRIELES_HXX
#define _TRIELES_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>

#include <vector>

// a segment is two integer pointers into the node list
class TGTriEle {
    int n1, n2, n3;

    double attribute;

public:

    // Constructor and destructor
    inline TGTriEle( void ) { };
    inline TGTriEle( int i1, int i2, int i3, double a ) {
	n1 = i1; n2 = i2; n3 = i3; attribute = a;
    }

    inline ~TGTriEle( void ) { };

    inline int get_n1() const { return n1; }
    inline void set_n1( int i ) { n1 = i; }
    inline int get_n2() const { return n2; }
    inline void set_n2( int i ) { n2 = i; }
    inline int get_n3() const { return n3; }
    inline void set_n3( int i ) { n3 = i; }

    inline double get_attribute() const { return attribute; }
    inline void set_attribute( double a ) { attribute = a; }
};


typedef std::vector < TGTriEle > triele_list;
typedef triele_list::iterator triele_list_iterator;
typedef triele_list::const_iterator const_triele_list_iterator;


#endif // _TRIELES_HXX


