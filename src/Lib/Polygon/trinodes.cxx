// trinodes.cxx -- "Triangle" nodes management class
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
// $Id: trinodes.cxx,v 1.7 2004-11-19 22:25:50 curt Exp $


#include "trinodes.hxx"


// Constructor 
TGTriNodes::TGTriNodes( void ) {
}


// Destructor
TGTriNodes::~TGTriNodes( void ) {
}


// Add a point to the point list if it doesn't already exist.  Returns
// the index (starting at zero) of the point in the list.
int TGTriNodes::unique_add( const Point3D& p ) {
    point_list_iterator current, last;
    int counter = 0;

    // cout << "unique add = " << p << endl;

    // see if point already exists
    current = node_list.begin();
    last = node_list.end();
    for ( ; current != last; ++current ) {
	if ( close_enough_2d(p, *current) ) {
	    // cout << "found an existing match!" << endl;

            // update elevation if new point has better info
            if ( current->z() < -9000 ) {
                current->setz( p.z() );
            }

	    return counter;
	}
	
	++counter;
    }

    // add to list
    node_list.push_back( p );

    return counter;
}


// Add a point to the point list if it doesn't already exist (checking
// all three dimensions.)  Returns the index (starting at zero) of the
// point in the list.
int TGTriNodes::unique_add_3d( const Point3D& p ) {
    point_list_iterator current, last;
    int counter = 0;

    // cout << p.x() << "," << p.y() << endl;

    // see if point already exists
    current = node_list.begin();
    last = node_list.end();
    for ( ; current != last; ++current ) {
	if ( close_enough_3d(p, *current) ) {
	    // cout << "found an existing match!" << endl;
	    return counter;
	}
	
	++counter;
    }

    // add to list
    node_list.push_back( p );

    return counter;
}


// Add the point with no uniqueness checking
int TGTriNodes::simple_add( const Point3D& p ) {
    node_list.push_back( p );

    return node_list.size() - 1;
}


// Add a point to the point list if it doesn't already exist.  Returns
// the index (starting at zero) of the point in the list.  Use a
// course proximity check
int TGTriNodes::course_add( const Point3D& p ) {
    point_list_iterator current, last;
    int counter = 0;

    // cout << p.x() << "," << p.y() << endl;

    // see if point already exists
    current = node_list.begin();
    last = node_list.end();
    for ( ; current != last; ++current ) {
	if ( course_close_enough(p, *current) ) {
	    // cout << "found an existing match!" << endl;

            // update elevation if new point has better info
            if ( current->z() < -9000 ) {
                current->setz( p.z() );
            }

	    return counter;
	}
	
	++counter;
    }

    // add to list
    node_list.push_back( p );

    return counter;
}


// Find the index of the specified point (compair to the same
// tolerance as unique_add().  Returns -1 if not found.
int TGTriNodes::find( const Point3D& p ) const {
    const_point_list_iterator current, last;
    int counter = 0;

    // cout << p.x() << "," << p.y() << endl;

    // see if point already exists
    current = node_list.begin();
    last = node_list.end();
    for ( ; current != last; ++current ) {
	if ( close_enough_2d(p, *current) ) {
	    // cout << "found an existing match!" << endl;
	    return counter;
	}
	
	++counter;
    }

    return -1;
}


