// chop.hxx -- routine to chop a polygon up along tile boundaries and
//             write the individual pieces to the TG working polygon
//             file format.
//
// Written by Curtis Olson, started February 1999.
//
// Copyright (C) 1999-2004  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: chop.hxx,v 1.3 2004-11-19 22:25:50 curt Exp $


#ifndef _TG_CHOP_HXX
#define _TG_CHOP_HXX


#include <string>
#include "polygon.hxx"
#include "texparams.hxx"

// process polygon shape (chop up along tile boundaries and write each
// polygon piece to a file)
void tgChopNormalPolygon( const std::string& path, const std::string& poly_type,
                          const TGPolygon& shape, bool preserve3d );

void tgChopNormalPolygonsWithMask(const std::string& path, const std::string& poly_type, 
                                  const poly_list& segments, bool preserve3d );

// process polygon shape (chop up along tile boundaries and write each
// polygon piece to a file)
void tgChopNormalPolygonsWithTP( const std::string& path, const std::string& poly_type,
								 const poly_list& segments, const texparams_list& tps, bool preserve3d );

// process polygon shape (chop up along tile boundaries and write each
// polygon piece to a file) This has a front end to a crude clipper
// that doesn't handle holes so beware.  This routine is appropriate
// for breaking down really huge structures if needed.
void tgChopBigSimplePolygon( const std::string& path, const std::string& poly_type,
                             const TGPolygon& shape, bool preserve3d );

#endif // _TG_CHOP_HXX


