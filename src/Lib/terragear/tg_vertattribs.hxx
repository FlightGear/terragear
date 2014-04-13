// tg_texparams.hxx -- texture parameters class
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
#ifndef _TG_VERTATTRIBS_HXX_
#define _TG_VERTATTRIBS_HXX_

#ifndef __cplusplus
# error This library requires C++
#endif

typedef enum {
    TG_VA_UNKNOWN,
    TG_VA_CONSTANT,
} tgVAttribMethod;

class tgIntVAttribParams
{
public:
    SGGeod ref;
    int    attrib;
    double width;
    double length;
    double heading;

    tgVAttribMethod method;
};

class tgFltVAttribParams
{
public:
    SGGeod ref;
    float  attrib;
    double width;
    double length;
    double heading;
    
    tgVAttribMethod method;
};

typedef boost::array<tgIntVAttribParams, 4> int_va_list;
typedef boost::array<tgFltVAttribParams, 4> flt_va_list;


#endif /* _TG_VERTATTRIBS_HXX_ */