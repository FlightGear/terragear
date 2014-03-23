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
#include <simgear/debug/logstream.hxx>

#include "tg_nodes.hxx"
#include "tg_rectangle.hxx"
#include "tg_texparams.hxx"
#include "tg_misc.hxx"
#include "clipper.hpp"

typedef std::vector<double>  va_dbl_list;
typedef std::vector<int>     va_int_list;

typedef std::vector<va_dbl_list> vertex_attrib_double_list;
typedef std::vector<va_int_list> vertex_attrib_integer_list;

class tgPolygon;

class tgTriangle
{
public:
    tgTriangle() {
        // resize mandatory lists
        node_list.resize( 3, SGGeod::fromDegM(0.0, 0.0, 0.0) );
        pri_tc_list.resize(   3, SGVec2f(0.0, 0.0) );
        norm_list.resize( 3, SGVec3d(0.0, 0.0, 0.0) );
        idx_list.resize(  3, -1 );

        // clear optional lists
        sec_tc_list.clear();
        vas_dbl.clear();
        vas_int.clear();
        
        parent = NULL;
        
        secondaryTP.method = TG_TEX_UNKNOWN;
    }

    tgTriangle( const SGGeod& p0, const SGGeod& p1, const SGGeod& p2, const tgPolygon* p ) {
        node_list.push_back( p0 );
        node_list.push_back( p1 );
        node_list.push_back( p2 );

        // resize mandatory lists
        pri_tc_list.resize(   3, SGVec2f(0.0, 0.0) );
        norm_list.resize( 3, SGVec3d(0.0, 0.0, 0.0) );
        idx_list.resize(  3, -1 );

        // clear optional lists
        sec_tc_list.resize( 3, SGVec2f(0.0, 0.0) );
        vas_dbl.clear();
        vas_int.clear();
        
        parent = p;
    }

    const tgPolygon* GetParent() {
        return parent;
    }
    void SetParent( const tgPolygon* p ) {
        parent = p;
    }
    
    void SetSecondaryTexParams( tgTexParams tp ) {
        secondaryTP = tp;
    }
    tgTexParams GetSecondaryTexParams( void ) const {
        return secondaryTP;
    }
    
    tgRectangle GetBoundingBox( void ) const;
    SGGeod GetCentroid( void ) const;

    SGGeod const& GetNode( unsigned int i ) const {
        return node_list[i];
    }
    std::vector<SGGeod>& GetNodeList( void ) {
        return node_list;
    }
    void SetNode( unsigned int i, const SGGeod& n ) {
        node_list[i] = n;
    }

    SGVec2f GetPriTexCoord( unsigned int i ) const {
        return pri_tc_list[i];
    }
    void SetPriTexCoord( unsigned int i, const SGVec2f tc ) {
        pri_tc_list[i] = tc;
    }
    void SetPriTexCoordList( const std::vector<SGVec2f>& tcs ) {
        pri_tc_list = tcs;
    }

    SGVec2f GetSecTexCoord( unsigned int i ) const {
        return sec_tc_list[i];
    }
    void SetSecTexCoord( unsigned int i, const SGVec2f tc ) {
        sec_tc_list[i] = tc;
    }
    void SetSecTexCoordList( const std::vector<SGVec2f>& tcs ) {
        sec_tc_list = tcs;
    }
    
    int GetIndex( unsigned int i ) const {
        return idx_list[i];
    }
    void SetIndex( unsigned int i, int idx ) {
        idx_list[i] = idx;
    }

    void SetFaceNormal( const SGVec3f& n ) {
        face_normal = n;
    };
    SGVec3f const& GetFaceNormal( void ) const {
        return face_normal;
    }

    void SetFaceArea( double a ) {
        face_area = a;
    }
    double GetFaceArea( void ) const {
        return face_area;
    }

    static double area( const SGGeod& p1, const SGGeod& p2, const SGGeod& p3 ) {
        return fabs(0.5 * ( p1.getLongitudeDeg() * p2.getLatitudeDeg() - p2.getLongitudeDeg() * p1.getLatitudeDeg() +
                            p2.getLongitudeDeg() * p3.getLatitudeDeg() - p3.getLongitudeDeg() * p2.getLatitudeDeg() +
                            p3.getLongitudeDeg() * p1.getLatitudeDeg() - p1.getLongitudeDeg() * p3.getLatitudeDeg() ));
    }

    tgsegment_list ToSegments() const
    {
        tgsegment_list result;

        result.push_back( tgSegment( node_list[0], node_list[1] ) );
        result.push_back( tgSegment( node_list[1], node_list[2] ) );
        result.push_back( tgSegment( node_list[2], node_list[0] ) );

        return result;
    }

    bool IsPointInside( SGGeod& pt ) const
    {
        tgRectangle bb = GetBoundingBox();
        
        // SG_LOG(SG_TERRAIN, SG_ALERT, "TEST " << lon << "," << lat << " with BB " << minx << "," << miny << " - " << maxx << "," << maxy );
        if ( bb.isInside( pt ) ) {
            SGVec2d A = SGVec2d( node_list[0].getLongitudeDeg(), node_list[0].getLatitudeDeg() );
            SGVec2d B = SGVec2d( node_list[1].getLongitudeDeg(), node_list[1].getLatitudeDeg() );
            SGVec2d C = SGVec2d( node_list[2].getLongitudeDeg(), node_list[2].getLatitudeDeg() );
            SGVec2d P = SGVec2d( pt.getLongitudeDeg(),           pt.getLatitudeDeg()           );
            
            SGVec2d V0 = C - A;
            SGVec2d V1 = B - A;
            SGVec2d V2 = P - A;
            
            // Compute dot products
            double dot00 = dot(V0, V0);
            double dot01 = dot(V0, V1);
            double dot02 = dot(V0, V2);
            double dot11 = dot(V1, V1);
            double dot12 = dot(V1, V2);
            
            // Compute barycentric coordinates
            double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            double l1 = (dot11 * dot02 - dot01 * dot12) * invDenom;
            double l2 = (dot00 * dot12 - dot01 * dot02) * invDenom;
            double l3 = ( 1 - l1 - l2 );
            
            if ( (l1 >= 0) && (l2 >= 0) && (l3 >= 0) ) {
                return true;
            }
        }
        
        return false;
    }
    
    bool InterpolateHeight( SGGeod& pt ) const
    {
        tgRectangle bb = GetBoundingBox();
        
        // SG_LOG(SG_TERRAIN, SG_ALERT, "TEST " << lon << "," << lat << " with BB " << minx << "," << miny << " - " << maxx << "," << maxy );
        if ( bb.isInside( pt ) ) {
            SGVec2d A = SGVec2d( node_list[0].getLongitudeDeg(), node_list[0].getLatitudeDeg() );
            SGVec2d B = SGVec2d( node_list[1].getLongitudeDeg(), node_list[1].getLatitudeDeg() );
            SGVec2d C = SGVec2d( node_list[2].getLongitudeDeg(), node_list[2].getLatitudeDeg() );
            SGVec2d P = SGVec2d( pt.getLongitudeDeg(),           pt.getLatitudeDeg()           );
            
            SGVec2d V0 = C - A;
            SGVec2d V1 = B - A;
            SGVec2d V2 = P - A;
            
            // Compute dot products
            double dot00 = dot(V0, V0);
            double dot01 = dot(V0, V1);
            double dot02 = dot(V0, V2);
            double dot11 = dot(V1, V1);
            double dot12 = dot(V1, V2);
            
            // Compute barycentric coordinates
            double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            double l1 = (dot11 * dot02 - dot01 * dot12) * invDenom;
            double l2 = (dot00 * dot12 - dot01 * dot02) * invDenom;
            double l3 = ( 1 - l1 - l2 );
            
            if ( (l1 >= 0) && (l2 >= 0) && (l3 >= 0) ) {
                double h = node_list[2].getElevationM()*l1 + node_list[1].getElevationM()*l2 + node_list[0].getElevationM()*l3;
                pt.setElevationM( h );
                return true;
            }
        }
        
        return false;
    }
    
    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgTriangle& );

private:
    const tgPolygon* parent;
    
    std::vector<SGGeod>  node_list;
    std::vector<SGVec2f> pri_tc_list;
    std::vector<SGVec2f> sec_tc_list;
    std::vector<SGVec3d> norm_list;
    std::vector<int>     idx_list;
    
    tgTexParams secondaryTP;
    
    vertex_attrib_double_list   vas_dbl;
    vertex_attrib_integer_list  vas_int;
    
    SGVec3f face_normal;
    double  face_area;
};

typedef std::vector <tgTriangle>  tgtriangle_list;
typedef tgtriangle_list::iterator tgtriangle_list_iterator;
typedef tgtriangle_list::const_iterator const_tgtriangle_list_iterator;

#endif /* _TG_TRIANGLE_HXX_ */