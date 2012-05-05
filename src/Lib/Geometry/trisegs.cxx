// trisegs.cxx -- "Triangle" segment management class
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
// $Id: trisegs.cxx,v 1.10 2004-11-19 22:25:50 curt Exp $

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>

#include <Geometry/point3d.hxx>

#include <iostream>

#include "trinodes.hxx"

#include "trisegs.hxx"

// Constructor 
TGTriSegments::TGTriSegments( void ) {
}


// Destructor
TGTriSegments::~TGTriSegments( void ) {
}


// Add a segment to the segment list if it doesn't already exist.
// Returns the index (starting at zero) of the segment in the list.
int TGTriSegments::unique_add( const TGTriSeg& s )
{
    triseg_list_iterator current, last;
    int counter = 0;

    // SG_LOG(SG_GENERAL, SG_DEBUG, s.get_n1() << "," << s.get_n2() );

    // check if segment has duplicated endpoints
    if ( s.get_n1() == s.get_n2() ) {
	    SG_LOG(SG_GENERAL, SG_ALERT, "WARNING: ignoring null segment with the same point for both endpoints" );
	return -1;
    }

    // check if segment already exists
    current = seg_list.begin();
    last = seg_list.end();
    for ( ; current != last; ++current ) {
	if ( s == *current ) {
	    // SG_LOG(SG_GENERAL, SG_DEBUG, "found an existing segment match" );
	    return counter;
	}
	
	++counter;
    }

    // add to list
    seg_list.push_back( s );

    return counter;
}


// Divide segment if there are other points on it, return the divided
// list of segments
void TGTriSegments::unique_divide_and_add( const point_list& nodes, 
					   const TGTriSeg& s )
{
    Point3D p0 = nodes[ s.get_n1() ];
    Point3D p1 = nodes[ s.get_n2() ];

    bool found_extra = false;
    int extra_index = 0;
    int counter;
    double m, m1, b, b1, y_err, x_err, y_err_min, x_err_min;
    const_point_list_iterator current, last;

    // bool temp = false;
    // if ( s == TGTriSeg( 170, 206 ) ) {
    //   SG_LOG(SG_GENERAL, SG_DEBUG, "this is it!" );
    //   temp = true;
    // }

    double xdist = fabs(p0.x() - p1.x());
    double ydist = fabs(p0.y() - p1.y());
    x_err_min = xdist + 1.0;
    y_err_min = ydist + 1.0;

    if ( xdist > ydist ) {
	// use y = mx + b

	// sort these in a sensible order
	if ( p0.x() > p1.x() ) {
	    Point3D tmp = p0;
	    p0 = p1;
	    p1 = tmp;
	}

	m = (p0.y() - p1.y()) / (p0.x() - p1.x());
	b = p1.y() - m * p1.x();

	// if ( temp ) {
	//   SG_LOG(SG_GENERAL, SG_DEBUG, "m = " << m << " b = " << b );
	// }

	current = nodes.begin();
	last = nodes.end();
	counter = 0;
	for ( ; current != last; ++current ) {
	    if ( (current->x() > (p0.x() + SG_EPSILON)) 
		 && (current->x() < (p1.x() - SG_EPSILON)) ) {

		// if ( temp ) {
		//   SG_LOG(SG_GENERAL, SG_DEBUG, counter );
		// }

		y_err = fabs(current->y() - (m * current->x() + b));

		if ( y_err < FG_PROXIMITY_EPSILON ) {
		    //SG_LOG(SG_GENERAL, SG_DEBUG, "FOUND EXTRA SEGMENT NODE (Y)" );
		    //SG_LOG(SG_GENERAL, SG_DEBUG,  p0 << " < " << *current << " < " << p1 );
		    found_extra = true;
		    if ( y_err < y_err_min ) {
			extra_index = counter;
			y_err_min = y_err;
		    }
		}
	    }
	    ++counter;
	}
    } else {
	// use x = m1 * y + b1

	// sort these in a sensible order
	if ( p0.y() > p1.y() ) {
	    Point3D tmp = p0;
	    p0 = p1;
	    p1 = tmp;
	}

	m1 = (p0.x() - p1.x()) / (p0.y() - p1.y());
	b1 = p1.x() - m1 * p1.y();

	// bool temp = true;
	// if ( temp ) {
	//   SG_LOG(SG_GENERAL, SG_DEBUG, "xdist = " << xdist << " ydist = " << ydist );
	//   SG_LOG(SG_GENERAL, SG_DEBUG, "  p0 = " << p0 << "  p1 = " << p1 );
	//   SG_LOG(SG_GENERAL, SG_DEBUG, "  m1 = " << m1 << " b1 = " << b1 );
	// }

	// SG_LOG(SG_GENERAL, SG_DEBUG, "  should = 0 = " << fabs(p0.x() - (m1 * p0.y() + b1)) );
	// SG_LOG(SG_GENERAL, SG_DEBUG, "  should = 0 = " << fabs(p1.x() - (m1 * p1.y() + b1)) );

	current = nodes.begin();
	last = nodes.end();
	counter = 0;
	for ( ; current != last; ++current ) {
	    if ( (current->y() > (p0.y() + SG_EPSILON)) 
		 && (current->y() < (p1.y() - SG_EPSILON)) ) {

		x_err = fabs(current->x() - (m1 * current->y() + b1));

		// if ( temp ) {
		//   SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << counter << ") x_err = " << x_err );
		// }

		if ( x_err < FG_PROXIMITY_EPSILON ) {
		    //SG_LOG(SG_GENERAL, SG_DEBUG, "FOUND EXTRA SEGMENT NODE (X)" );
		    //SG_LOG(SG_GENERAL, SG_DEBUG, p0 << " < " << *current << " < " << p1 );
		    found_extra = true;
		    if ( x_err < x_err_min ) {
			extra_index = counter;
			x_err_min = x_err;
		    }
		}
	    }
	    ++counter;
	}
    }

    if ( found_extra ) {
	// recurse with two sub segments
	SG_LOG(SG_GENERAL, SG_DEBUG, "dividing " << s.get_n1() << " " << extra_index << " " << s.get_n2() );
	unique_divide_and_add( nodes, TGTriSeg( s.get_n1(), extra_index, 
						s.get_boundary_marker() ) );
	unique_divide_and_add( nodes, TGTriSeg( extra_index, s.get_n2(), 
						s.get_boundary_marker() ) );
    } else {
	// this segment does not need to be divided, lets add it
	unique_add( s );
    }
}


