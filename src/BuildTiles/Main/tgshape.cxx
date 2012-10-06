// TGShape.cxx -- Class to handle polygons shapes generated in ogr-decode
//                A shape may consist of many polygons when it is generated
//                from a polyline.  They are kept together to speed up clipping
//                but also must be represented as seperate polygons in order to
//                keep track of textur parameters.
//
// Written by Curtis Olson, started May 1999.
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
// $Id: construct.cxx,v 1.4 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "tgshape.hxx"

void TGShape::GetName( char* name ) const
{
    sprintf( name, "%s_%d", get_area_name( (AreaType)area ).c_str(), id );
}

void TGShape::SetMask( TGPolygon mask )
{
    clip_mask = mask;
}

void TGShape::BuildMask( void )
{
    TGPolygon poly;
    clip_mask.erase();

    for (unsigned int i=0; i<sps.size(); i++)
    {
        poly = sps[i].get_poly();
        clip_mask = tgPolygonUnion( clip_mask, poly );
    }
}

void TGShape::IntersectPolys( void )
{
    if ( sps.size() > 1 ) {
        TGPolygon original, intersect;

        for (unsigned int i=0; i<sps.size(); i++)
        {
            original  = sps[i].get_poly();

            intersect = tgPolygonInt( clip_mask, original );

            sps[i].set_poly( intersect );
        }
    } else {
        sps[0].set_poly( clip_mask );
    }
}

// Serialization
// input from stream
std::istream& operator >> ( std::istream& in, TGShape& p)
{
    int i, count;

    // First, load the clipmask
    in >> p.clip_mask;

    // Then load superpolys
    in >> count;
    for (i=0; i<count; i++) {
        TGSuperPoly sp;
        in >> sp;
        p.sps.push_back( sp );
    }

    // Then load texparams
    in >> count;
    for (i=0; i<count; i++) {
        TGTexParams tp;
        in >> tp;
        p.tps.push_back( tp );
    }

    // Load the id, area type and textured flag
    in >> p.id;
    in >> p.area;
    in >> p.textured;

    return in;
}

std::ostream& operator<< ( std::ostream& out, const TGShape& p )
{
    int i, count;
    TGSuperPoly sp;
    TGTexParams tp;

    // First, save the clipmask
    out << p.clip_mask;

    // Then save superpolys
    count = p.sps.size();
    out << count << "\n";
    for (i=0; i<count; i++) {
        out << p.sps[i];
    }

    // Then save texparams
    count = p.tps.size();
    out << count << "\n";
    for (i=0; i<count; i++) {
        out << p.tps[i];
    }

    // Save the id, area type and textured flag
    out << p.id << " ";
    out << p.area << " ";
    out << p.textured << "\n";

    return out;
}
