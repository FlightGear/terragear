// polygon.hxx -- polygon (with holes) management class
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


#ifndef _POLYGON_HXX
#define _POLYGON_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <zlib.h> 

#include <boost/array.hpp>

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include "tg_nodes.hxx"
#include "clipper.hpp"

// WORK IN PROGRESS BEGIN ******************************************************
// Looking ato TGPolygon and TGSuperPoly, I think it would be best if we could
// templatize TGPolygon.  We really want 4 mirror shapes
// 1 with SGGeode as nodes - to represent Geodetic coordinates
// 1 with SGVec2d as nodes - to represent texture coordinates
// 1 with SGVec3d as nodes - to represent normals
// 1 with int as nodes     - to represent indicies into the node list
//
// The problem being, is that there are many type dependent implimenations needed
// For example:
// boolean operations, and calc_point_inside() should only be done on SGGeod polys
// most of the other formats are for data storage only.
//
// For the first attempt, I will just have TGPolygon with a list of contours.
// the extra data are stored in paralell vectors.
// This should also make TGSuperPoly obsolete

#include <simgear/bucket/newbucket.hxx>
#include <simgear/threads/SGThread.hxx>

#include "tg_rectangle.hxx"
#include "tg_contour.hxx"
#include "tg_texparams.hxx"
#include "tg_vertattribs.hxx"

// utilities - belong is simgear?
double CalculateTheta( const SGVec3d& dirCur, const SGVec3d& dirNext, const SGVec3d& cp );
SGGeod midpoint( const SGGeod& p0, const SGGeod& p1 );
SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, int& turn_dir );
SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by );
SGGeod OffsetPointFirst( const SGGeod& cur, const SGGeod& next, double offset_by );
SGGeod OffsetPointLast( const SGGeod& prev, const SGGeod& cur, double offset_by );

void OffsetPointsMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, double width, int& turn_dir, SGGeod& inner, SGGeod& outer );
void OffsetPointsMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, double width, SGGeod& inner, SGGeod& outer );
void OffsetPointsFirst( const SGGeod& cur, const SGGeod& next, double offset_by, double width, SGGeod& inner, SGGeod& outer );
void OffsetPointsLast( const SGGeod& prev, const SGGeod& cur, double offset_by, double width, SGGeod& inner, SGGeod& outer );

// what abount this?

// Save clipper to shapefile
void clipper_to_shapefile( ClipperLib::Paths polys, char* datasource );


// forward declaration
// Forward Declaration:
class tgPolygon;
class tgChoppedPolygons;
class TGNodes;
class TGNode;

typedef std::vector <tgPolygon>  tgpolygon_list;
typedef tgpolygon_list::iterator tgpolygon_list_iterator;
typedef tgpolygon_list::const_iterator const_tgpolygon_list_iterator;

class tgPolygon
{
public:
    tgPolygon() {
        static unsigned int cur_id = 0;
        
        preserve3d = false;
        tp.method = TG_TEX_UNKNOWN;
        for ( unsigned int i=0; i<4; i++ ) {
            int_vas[i].method = TG_VA_UNKNOWN;
        }
        va_int_mask = 0;
        
        for ( unsigned int i=0; i<4; i++ ) {
            flt_vas[i].method = TG_VA_UNKNOWN;
        }        
        va_flt_mask = 0;
        
        id = cur_id++;
    }
    ~tgPolygon() {
        contours.clear();
        triangles.clear();
    }

    void Erase( void ) {
        contours.clear();
        triangles.clear();
    }

    unsigned int Contours( void ) const {
        return contours.size();
    }
    unsigned int ContourSize( unsigned int c ) const {
        return contours[c].GetSize();
    }
    void AddContour( tgContour const& contour ) {
        contours.push_back(contour);
    }
    tgContour GetContour( unsigned int c ) const {
        return contours[c];
    }
    void DeleteContourAt( unsigned int idx ) {
        if ( idx < contours.size() ) {
            contours.erase( contours.begin() + idx );
        }
    }

    unsigned int TotalNodes( void ) const;

    SGGeod const& GetNode( unsigned int c, unsigned int i ) const {
        return contours[c][i];
    }
    void SetNode( unsigned int c, unsigned int i, const SGGeod& n ) {
        contours[c].SetNode( i, n );
    }

    void AddNode( unsigned int c, const SGGeod& n ) {
        // Make sure we have contour c.  If we don't add it
        while( contours.size() < c+1 ) {
            tgContour dummy;
            dummy.SetHole( false );
            contours.push_back( dummy );
        }

        contours[c].AddNode( n );
    }

    tgRectangle GetBoundingBox( void ) const;

    void InheritElevations( const tgPolygon& source );

    unsigned int Triangles( void ) const {
        return triangles.size();
    }
    void AddTriangle( tgTriangle& triangle ) {
        triangle.SetParent( this );
        triangles.push_back( triangle );
    }
    void AddTriangle( const SGGeod& p1, const SGGeod p2, const SGGeod p3 ) {
        triangles.push_back( tgTriangle( p1, p2, p3, this ) );
    }
    const tgTriangle& GetTriangle( unsigned int t ) const {
        return triangles[t];
    }    
    tgTriangle& GetTriangle( unsigned int t ) {
        return triangles[t];
    }    
    SGGeod GetTriNode( unsigned int c, unsigned int i ) const {
        return triangles[c].GetNode( i );
    }
    SGVec2f GetTriPriTexCoord( unsigned int c, unsigned int i ) const {
        return triangles[c].GetPriTexCoord( i );
    }
    SGVec2f GetTriSecTexCoord( unsigned int c, unsigned int i ) const {
        if ( c < triangles.size() ) {
            return triangles[c].GetSecTexCoord( i );
        } else {
            return SGVec2f(0.0, 0.0);
        }
    }
    unsigned int GetTriIntVA( unsigned int c, unsigned int i, unsigned int a  ) const {
        if ( c < triangles.size() ) {
            return triangles[c].GetIntVA( i, a );
        } else {
            return 0;
        }
    }
    float GetTriFltVA( unsigned int c, unsigned int i, unsigned int a  ) const {
        if ( c < triangles.size() ) {
            return triangles[c].GetFltVA( i, a );
        } else {
            return 0.0;
        }
    }
    
    void SetTriIdx( unsigned int c, unsigned int i, int idx ) {
        triangles[c].SetIndex( i, idx );
    }
    int GetTriIdx( unsigned int c, unsigned int i ) const {
        return triangles[c].GetIndex( i );
    }
    void SetTriFaceNormal( unsigned int c, const SGVec3f& n ) {
        triangles[c].SetFaceNormal( n );
    }
    SGVec3f const& GetTriFaceNormal( unsigned int c ) const {
        return triangles[c].GetFaceNormal();
    }
    void SetTriFaceArea( unsigned int c, double a ) {
        triangles[c].SetFaceArea( a );
    }
    double GetTriFaceArea( unsigned int c ) const {
        return triangles[c].GetFaceArea();
    }

    std::string const& GetMaterial( void ) const {
        return material;
    }
    void SetMaterial( const std::string& m ) {
        material = m;
    }

    std::string const& GetFlag( void ) const {
        return flag;
    }
    void SetFlag( const std::string& f ) {
        flag = f;
    }

    bool GetPreserve3D( void ) const {
        return preserve3d;
    }
    void SetPreserve3D( bool p ) {
        preserve3d = p;
    }
    void SetElevations( const TGNodes& nodes );

    unsigned int GetId( void ) const {
        return id;
    }
    void SetId( unsigned int i ) {
        id = i;
    }


    // Texturing
    void SetTexParams( const SGGeod& ref, double width, double length, double heading ) {
        tp.ref     = ref;
        tp.width   = width;
        tp.length  = length;
        tp.heading = heading;
    }
    void SetTexParams( const tgTexParams& t ) {
        tp = t;
    }
    const tgTexParams& GetTexParams( void ) const {
        return tp;
    }

    void SetTexLimits( double minu, double minv, double maxu, double maxv ) {
        tp.minu = minu;
        tp.minv = minv;
        tp.maxu = maxu;
        tp.maxv = maxv;
    }
    
    void SetTexMethod( tgTexMethod m ) {
        tp.method = m;
    }
    void SetTexMethod( tgTexMethod m, double min_cu, double min_cv, double max_cu, double max_cv ) {
        tp.method = m;
        tp.min_clipu = min_cu;
        tp.min_clipv = min_cv;
        tp.max_clipu = max_cu;
        tp.max_clipv = max_cv;
    }
    void SetTexMethod( tgTexMethod m, double cl ) {
        tp.method = m;
        tp.center_lat = cl;
    }
    tgTexMethod GetTexMethod( void ) const {
        return tp.method;
    }

    void SetVertexAttributeInt(tgVAttribMethod m, int index, int attrib)
    {
        if ( index < 4 ) {
            int_vas[index].method = m;
            int_vas[index].attrib = attrib;
            
            va_int_mask |= 1 << index;            
        }
    }
    void SetVertexAttributeFlt(tgVAttribMethod m, int index, float attrib)
    {
        if ( index < 4 ) {
            flt_vas[index].method = m;
            flt_vas[index].attrib = attrib;
            
            va_flt_mask |= 1 << index;            
        }
    }
    
    unsigned int GetNumIntVas( void ) {
        int num = 0;
        for ( unsigned int i=0; i<4; i++ ) {
            if ( (va_int_mask>>i) & 0x01 ) {
                num++;
            }
        }
        
        return num;
    }
    unsigned int GetNumFltVas( void ) {
        int num = 0;
        for ( unsigned int i=0; i<4; i++ ) {
            if ( (va_flt_mask>>i) & 0x01 ) {
                num++;
            }
        }
        return num;
    }
    
    void Texture( void );
    void TextureSecondary( void );

    // Tesselation
    void Tesselate( void );
    void Tesselate( const std::vector<SGGeod>& extra );

    // Straight Skeleton
    tgpolygon_list StraightSkeleton(void);
    
    // Boolean operations
    static void      SetClipperDump( bool dmp );
    static tgPolygon Union( const tgPolygon& subject, tgPolygon& clip );
    static tgPolygon Union( const tgpolygon_list& polys );
    static tgPolygon Diff( const tgPolygon& subject, tgPolygon& clip );
    static tgPolygon Intersect( const tgPolygon& subject, const tgPolygon& clip );

    // cleanup operations
    //static tgPolygon Snap( const tgPolygon& subject, double snap );
    static tgPolygon StripHoles( const tgPolygon& subject );
    static tgPolygon SplitLongEdges( const tgPolygon& subject, double dist );
    static tgPolygon RemoveCycles( const tgPolygon& subject );
    //static tgPolygon RemoveDups( const tgPolygon& subject );
    //static tgPolygon RemoveBadContours( const tgPolygon& subject );
    static tgPolygon Simplify( const tgPolygon& subject );
    static tgPolygon RemoveColinearNodes( const tgPolygon& subject );
    static tgPolygon RemoveTinyContours( const tgPolygon& subject );
    static tgPolygon RemoveSpikes( const tgPolygon& subject );
    static void           RemoveSlivers( tgPolygon& subject, tgcontour_list& slivers );
    static tgcontour_list MergeSlivers( tgpolygon_list& subjects, tgcontour_list& slivers );

    static tgPolygon Expand( const tgPolygon& subject, double offset );
    static tgPolygon Expand( const SGGeod& subject, double offset );

    // Conversions
    static ClipperLib::Paths ToClipper( const tgPolygon& subject );
    static tgPolygon FromClipper( const ClipperLib::Paths& subject );
    static void ToClipperFile( const tgPolygon& subject, const std::string& path, const std::string& filename );
    
    // T-Junctions and segment search
    static tgPolygon AddColinearNodes( const tgPolygon& subject, UniqueSGGeodSet& nodes );
    static tgPolygon AddColinearNodes( const tgPolygon& subject, std::vector<SGGeod>& nodes );
    static tgPolygon AddColinearNodes( const tgPolygon& subject, std::vector<TGNode*>& nodes );
    static bool      FindColinearLine( const tgPolygon& subject, SGGeod& node, SGGeod& start, SGGeod& end );
    static tgPolygon AddIntersectingNodes( const tgPolygon& subject, const tgtriangle_list& mesh );

    // NON STATIC CLEANING
    void         Snap( double snap );
    unsigned int RemoveDups( void );
    unsigned int RemoveBadContours( void );
    
    
    // IO
    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    friend std::ostream& operator<< ( std::ostream&, const tgPolygon& );

public:
    int_va_list     int_vas;
    flt_va_list     flt_vas;
    
    unsigned int    va_int_mask;
    unsigned int    va_flt_mask;
    
private:
    tgcontour_list  contours;
    tgtriangle_list triangles;

    std::string     material;
    std::string     flag;       // let's get rid of this....
    bool            preserve3d;
    unsigned int    id;         // unique polygon id for debug
    tgTexParams     tp;
};

#endif // _POLYGON_HXX
