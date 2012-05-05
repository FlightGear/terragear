// trisegs.hxx -- "Triangle" segment management class
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
// $Id: trisegs.hxx,v 1.4 2004/11/19 22:25:50 curt Exp $


#ifndef _TRISEGS_HXX
#define _TRISEGS_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>

#include <vector>

#include "trinodes.hxx"

// a segment is two integer pointers into the node list
class TGTriSeg {
    int n1, n2;			// indices into point list
    int boundary_marker;	// flag if segment is a boundary
				// (i.e. shouldn't get split)

public:

    // Constructor and destructor
    inline TGTriSeg( void ) { };
    inline TGTriSeg( int i1, int i2, int b ) { 
	n1 = i1;
	n2 = i2;
	boundary_marker = b;
    }

    inline ~TGTriSeg( void ) { };

    inline int get_n1() const { return n1; }
    inline void set_n1( int i ) { n1 = i; }
    inline int get_n2() const { return n2; }
    inline void set_n2( int i ) { n2 = i; }
    inline int get_boundary_marker() const { return boundary_marker; }
    inline void set_boundary_marker( int b ) { boundary_marker = b; }

    friend bool operator == (const TGTriSeg& a, const TGTriSeg& b);

};

inline bool operator == (const TGTriSeg& a, const TGTriSeg& b)
{
    return ((a.n1 == b.n1) && (a.n2 == b.n2)) 
	|| ((a.n1 == b.n2) && (a.n2 == b.n1));
}


typedef std::vector < TGTriSeg > triseg_list;
typedef triseg_list::iterator triseg_list_iterator;
typedef triseg_list::const_iterator const_triseg_list_iterator;


class TGTriSegments {

private:

    triseg_list seg_list;

    // Divide segment if there are other points on it, return the
    // divided list of segments
    triseg_list divide_segment( const point_list& nodes, 
				const TGTriSeg& s );

public:

    // Constructor and destructor
    TGTriSegments( void );
    ~TGTriSegments( void );

    // delete all the data out of seg_list
    inline void clear() { seg_list.clear(); }

    // Add a segment to the segment list if it doesn't already exist.
    // Returns the index (starting at zero) of the segment in the
    // list.
    int unique_add( const TGTriSeg& s );

    // Add a segment to the segment list if it doesn't already exist.
    // Returns the index (starting at zero) of the segment in the list.
    void unique_divide_and_add( const point_list& node_list, 
				const TGTriSeg& s );

    // return the master segment list
    inline triseg_list& get_seg_list() { return seg_list; }
    inline const triseg_list& get_seg_list() const { return seg_list; }

    // return the ith segment
    inline TGTriSeg& get_seg( int i ) { return seg_list[i]; }
    inline const TGTriSeg& get_seg( int i ) const { return seg_list[i]; }
};


#endif // _TRISEGS_HXX


