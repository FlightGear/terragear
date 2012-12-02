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

void TGShape::SetMask( const tgPolygon& m )
{
    mask = m;
}

void TGShape::BuildMask( void )
{
    mask = tgPolygon::Union( polys );
}

void TGShape::IntersectPolys( void )
{
    if ( polys.size() > 1 ) {
        for (unsigned int i=0; i<polys.size(); i++)
        {
            SG_LOG(SG_GENERAL, SG_INFO, " before is " << polys[i].GetMaterial() );
            polys[i] = tgPolygon::Intersect( polys[i], mask );
            SG_LOG(SG_GENERAL, SG_INFO, " after is " << polys[i].GetMaterial() );
        }
    } else {
        std::string material = polys[0].GetMaterial();
        tgTexParams tp = polys[0].GetTexParams();
        polys[0] = mask;
        polys[0].SetMaterial( material );
        polys[0].SetTexParams( tp );
    }
}

void TGShape::LoadFromGzFile(gzFile& fp)
{
    int i, count;

    // First, load the clipmask
    SG_LOG(SG_GENERAL, SG_INFO, " load mask" );
    mask.LoadFromGzFile( fp );
    SG_LOG(SG_GENERAL, SG_INFO, " done" );

    // Then load individual polys
    sgReadInt( fp, &count );
    for (i=0; i<count; i++) {
        tgPolygon poly;
        poly.LoadFromGzFile( fp );
        polys.push_back( poly );
    }

    // Load the id, area type and textured flag
    sgReadUInt( fp, &id );
    sgReadInt(  fp, (int*)&area );
    sgReadInt(  fp, (int*)&textured );
}

std::ostream& operator<< ( std::ostream& out, const TGShape& p )
{
    int i, count;

    // First, save the clipmask
    out << p.mask;

    // Then save superpolys
    count = p.polys.size();
    out << count << "\n";
    for (i=0; i<count; i++) {
        out << p.polys[i];
    }

    // Save the id, area type and textured flag
    out << p.id << " ";
    out << p.area << " ";
    out << p.textured << "\n";

    return out;
}

void TGShape::SaveToGzFile(gzFile& fp)
{
    int i, count;

    // First, save the clipmask
    mask.SaveToGzFile( fp );

    // Then save superpolys
    count = polys.size();
    sgWriteInt( fp, count );
    for (i=0; i<count; i++) {
        polys[i].SaveToGzFile( fp );
    }

    // Save the id, area type and textured flag
    sgWriteUInt( fp, id );
    sgWriteInt( fp, (int)area );
    sgWriteInt( fp, (int)textured );
}