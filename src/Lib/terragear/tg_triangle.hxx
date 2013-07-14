// tg_triangle.hxx -- triangle management class
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
// $Id: polygon.hxx,v 1.17 2007-11-05 14:02:21 curt Exp $


#ifndef _TG_TRIANGLE_HXX
#define _TG_TRIANGLE_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <zlib.h> 

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include "tg_nodes.hxx"
#include "tg_rectangle.hxx"
#include "tg_misc.hxx"
#include "clipper.hpp"

class tgTriangle
{
public:
    tgTriangle() {
        node_list.resize( 3, SGGeod::fromDegM(0.0, 0.0, 0.0) );
        tc_list.resize(   3, SGVec2f(0.0, 0.0) );
        norm_list.resize( 3, SGVec3d(0.0, 0.0, 0.0) );
        idx_list.resize(  3, -1 );
    }

    tgTriangle( const SGGeod& p0, const SGGeod& p1, const SGGeod& p2 ) {
        node_list.push_back( p0 );
        node_list.push_back( p1 );
        node_list.push_back( p2 );

        tc_list.resize(   3, SGVec2f(0.0, 0.0) );
        norm_list.resize( 3, SGVec3d(0.0, 0.0, 0.0) );
        idx_list.resize(  3, -1 );
    }

    tgRectangle GetBoundingBox( void ) const;

    SGGeod const& GetNode( unsigned int i ) const {
        return node_list[i];
    }
    std::vector<SGGeod>& GetNodeList( void ) {
        return node_list;
    }
    void SetNode( unsigned int i, const SGGeod& n ) {
        node_list[i] = n;
    }

    SGVec2f GetTexCoord( unsigned int i ) const {
        return tc_list[i];
    }
    void SetTexCoord( unsigned int i, const SGVec2f tc ) {
        tc_list[i] = tc;
    }
    void SetTexCoordList( const std::vector<SGVec2f>& tcs ) {
        tc_list = tcs;
    }
    int GetIndex( unsigned int i ) const {
        return idx_list[i];
    }
    void SetIndex( unsigned int i, int idx ) {
        idx_list[i] = idx;
    }

    void    SetFaceNormal( const SGVec3f& n ) {
        face_normal = n;
    };
    SGVec3f const& GetFaceNormal( void ) const {
        return face_normal;
    }

    void    SetFaceArea( double a ) {
        face_area = a;
    }
    double  GetFaceArea( void ) const {
        return face_area;
    }

    static double area( const SGGeod& p1, const SGGeod& p2, const SGGeod& p3 ) {
        return fabs(0.5 * ( p1.getLongitudeDeg() * p2.getLatitudeDeg() - p2.getLongitudeDeg() * p1.getLatitudeDeg() +
                            p2.getLongitudeDeg() * p3.getLatitudeDeg() - p3.getLongitudeDeg() * p2.getLatitudeDeg() +
                            p3.getLongitudeDeg() * p1.getLatitudeDeg() - p1.getLongitudeDeg() * p3.getLatitudeDeg() ));
    }

    tgsegment_list ToSegments()
    {
        tgsegment_list result;

        result.push_back( tgSegment( node_list[0], node_list[1] ) );
        result.push_back( tgSegment( node_list[1], node_list[2] ) );
        result.push_back( tgSegment( node_list[2], node_list[0] ) );

        return result;
    }


    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgTriangle& );

private:
    std::vector<SGGeod>  node_list;
    std::vector<SGVec2f> tc_list;
    std::vector<SGVec3d> norm_list;
    std::vector<int>     idx_list;

    SGVec3f face_normal;
    double  face_area;
};

typedef std::vector <tgTriangle>  tgtriangle_list;
typedef tgtriangle_list::iterator tgtriangle_list_iterator;
typedef tgtriangle_list::const_iterator const_tgtriangle_list_iterator;

#endif /* _TG_TRIANGLE_HXX_ */