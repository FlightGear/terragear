// texparams.hxx -- A simple class to hold texture application
//                  parameters for sections of the runway
//
// Written by Curtis Olson, started August 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - curt@flightgear.org
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
// $Id$


#ifndef _TEXPARAMS_HXX
#define _TEXPARAMS_HXX


#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   


#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <vector>

SG_USING_STD(vector);


class FGTexParams {

private:

    Point3D ref;
    double width;
    double length;
    double heading;

public:

    // Constructor and destructor
    inline FGTexParams( void ) { }
    inline FGTexParams( const Point3D &r, const double w, const double l,
			const double h ) {
	ref = r;
	width = w;
	length = l;
	heading = h;
    }
    inline ~FGTexParams( void ) { }

    inline Point3D get_ref() const { return ref; }
    inline void set_ref( const Point3D &r ) { ref = r; }

    inline double get_width() const { return width; }
    inline void set_width( const double w ) { width = w; }

    inline double get_length() const { return length; }
    inline void set_length( const double l ) { length = l; }

    inline double get_heading() const { return heading; }
    inline void set_heading( const double h ) { heading = h; }
};


typedef vector < FGTexParams > texparams_list;
typedef texparams_list::iterator texparams_list_iterator;
typedef texparams_list::const_iterator const_texparams_list_iterator;


#endif // _TEXPARAMS_HXX
