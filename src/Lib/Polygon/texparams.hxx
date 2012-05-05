// texparams.hxx -- A simple class to hold texture application
//                  parameters for sections of the runway
//
// Written by Curtis Olson, started August 2000.
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
// $Id: texparams.hxx,v 1.7 2004-11-19 22:25:49 curt Exp $


#ifndef _TEXPARAMS_HXX
#define _TEXPARAMS_HXX


#ifndef __cplusplus
# error This library requires C++
#endif


#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <vector>

class TGTexParams {

private:

Point3D ref;
double width;
double length;
double heading;

double minu;
double maxu;
double minv;
double maxv;

public:

// Constructor and destructor
inline TGTexParams( void )
{
}
inline TGTexParams( const Point3D &r, const double w, const double l,
                    const double h )
{
    ref = r;
    width = w;
    length = l;
    heading = h;

    minu = minv = 0.0;
    maxu = maxv = 1.0;
}
inline ~TGTexParams( void )
{
}

inline Point3D get_ref() const
{
    return ref;
}
inline void set_ref( const Point3D &r )
{
    ref = r;
}

inline double get_width() const
{
    return width;
}
inline void set_width( const double w )
{
    width = w;
}

inline double get_length() const
{
    return length;
}
inline void set_length( const double l )
{
    length = l;
}

inline double get_heading() const
{
    return heading;
}
inline void set_heading( const double h )
{
    heading = h;
}

inline double get_minu() const
{
    return minu;
}
inline void set_minu( const double x )
{
    minu = x;
}

inline double get_maxu() const
{
    return maxu;
}
inline void set_maxu( const double x )
{
    maxu = x;
}

inline double get_minv() const
{
    return minv;
}
inline void set_minv( const double x )
{
    minv = x;
}

inline double get_maxv() const
{
    return maxv;
}
inline void set_maxv( const double x )
{
    maxv = x;
}
};


typedef std::vector < TGTexParams > texparams_list;
typedef texparams_list::iterator texparams_list_iterator;
typedef texparams_list::const_iterator const_texparams_list_iterator;


#endif // _TEXPARAMS_HXX
