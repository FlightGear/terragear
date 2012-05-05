// tri_support.h -- supporting routines for the triangulation library
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: tri_support.h,v 1.3 2004-11-19 22:25:50 curt Exp $


#ifndef _TRI_SUPPORT_H
#define _TRI_SUPPORT_H


#ifdef __cplusplus
extern "C" {
#endif


#define REAL double
#include "triangle.h"


void zero_triangulateio( struct triangulateio *in );
void print_tri_data( struct triangulateio *out );
void write_tri_data( struct triangulateio *out );


#ifdef __cplusplus
}
#endif


#endif // _TRI_SUPPORT_H


