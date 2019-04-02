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

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

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
#include <boost/concept_check.hpp>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/threads/SGThread.hxx>

#include "tg_rectangle.hxx"
#include "tg_contour.hxx"

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
class TGNode;

typedef std::vector <tgPolygon>  tgpolygon_list;
typedef tgpolygon_list::iterator tgpolygon_list_iterator;
typedef tgpolygon_list::const_iterator const_tgpolygon_list_iterator;

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

    tgRectangle GetBoundingBox( void ) const
    {
        SGGeod min, max;

        double minx =  std::numeric_limits<double>::infinity();
        double miny =  std::numeric_limits<double>::infinity();
        double maxx = -std::numeric_limits<double>::infinity();
        double maxy = -std::numeric_limits<double>::infinity();

        for (unsigned int i = 0; i < node_list.size(); i++) {
            SGGeod pt = GetNode(i);
            if ( pt.getLongitudeDeg() < minx ) { minx = pt.getLongitudeDeg(); }
            if ( pt.getLongitudeDeg() > maxx ) { maxx = pt.getLongitudeDeg(); }
            if ( pt.getLatitudeDeg()  < miny ) { miny = pt.getLatitudeDeg(); }
            if ( pt.getLatitudeDeg()  > maxy ) { maxy = pt.getLatitudeDeg(); }
        }

        min = SGGeod::fromDeg( minx, miny );
        max = SGGeod::fromDeg( maxx, maxy );

        return tgRectangle( min, max );        
    }

    SGGeod const& GetNode( unsigned int i ) const {
        return node_list[i];
    }
    std::vector<SGGeod>& GetNodeList( void ) {
        return node_list;
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

    static ClipperLib::Path ToClipper( const tgTriangle& subject );
    static tgPolygon Intersect( const tgTriangle& subject, const tgTriangle& clip );
    
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

typedef enum {
    TG_TEX_BY_GEODE,
    TG_TEX_BY_TPS_NOCLIP,
    TG_TEX_BY_TPS_CLIPU,
    TG_TEX_BY_TPS_CLIPV,
    TG_TEX_BY_TPS_CLIPUV,
    TG_TEX_BY_HORIZ_REF     //Tex coord calculated from ht and horiz reference
} tgTexMethod;

class tgTexParams
{
public:
    SGGeod ref;
    double width;
    double length;
    double heading;

    double minu;
    double maxu;
    double minv;
    double maxv;

    double min_clipu;
    double max_clipu;
    double min_clipv;
    double max_clipv;

    double custom_s;

    tgTexMethod method;

    double center_lat;

    void SaveToGzFile( gzFile& fp ) const;
    int LoadFromGzFile( gzFile& fp );

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgTexParams& );
};

class tgPolygon
{
public:
    tgPolygon() {
        preserve3d = false;
        closed = true;
        id = 0;
        tp.method = TG_TEX_BY_GEODE;
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
    void DelNode( unsigned int c, unsigned int n ) {
        contours[c].DelNode( n );
    }

    tgRectangle GetBoundingBox( void ) const;

    void InheritElevations( const tgPolygon& source );

    unsigned int Triangles( void ) const {
        return triangles.size();
    }
    void AddTriangle( const tgTriangle& triangle ) {
        triangles.push_back( triangle );
    }
    void AddTriangle( const SGGeod& p1, const SGGeod p2, const SGGeod p3 ) {
        triangles.push_back( tgTriangle( p1, p2, p3 ) );
    }
    tgTriangle GetTriangle( unsigned int t ) {
        return triangles[t];
    }

    SGGeod GetTriNode( unsigned int c, unsigned int i ) const {
        return triangles[c].GetNode( i );
    }
    SGVec2f GetTriTexCoord( unsigned int c, unsigned int i ) const {
        return triangles[c].GetTexCoord( i );
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

  void SetOpen( void ) {   //Do not try to close the contours
    closed = false;
  }

  void SetClosed (void) {
    closed = true;
  }

  bool IsClosed( void ) const {
    return closed;
  }   

    bool GetPreserve3D( void ) const {
        return preserve3d;
    }
    void SetPreserve3D( bool p ) {
        preserve3d = p;
    }

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

    void SetTexReference( SGGeod g, double tex_coord ) {
        tp.ref = g;
        tp.custom_s = tex_coord;
    }

    SGGeod GetTexRefPt() const {
        return tp.ref;
    }

    double GetRefTexCoord() const {
        return tp.custom_s;
    }

    tgTexMethod GetTexMethod( void ) const {
        return tp.method;
    }
    void Texture();
    void Texture( const std::vector<SGGeod>& geod_nodes );

    // Tesselation
    void Tesselate( void );
    void Tesselate( const std::vector<SGGeod>& extra );

    // Boolean operations
    static void      SetClipperDump( bool dmp );
    static tgPolygon Union( const tgPolygon& subject, tgPolygon& clip );
    static tgPolygon Union( const tgpolygon_list& polys );
    static tgPolygon Diff( const tgPolygon& subject, tgPolygon& clip );
    static tgPolygon Intersect( const tgPolygon& subject, const tgPolygon& clip );

    // cleanup operations
    static tgPolygon Snap( const tgPolygon& subject, double snap );
    static tgPolygon StripHoles( const tgPolygon& subject );
    static tgPolygon SplitLongEdges( const tgPolygon& subject, double dist );
    static tgPolygon RemoveCycles( const tgPolygon& subject );
    static tgPolygon RemoveDups( const tgPolygon& subject );
    static tgPolygon RemoveBadContours( const tgPolygon& subject );
    static tgPolygon Simplify( const tgPolygon& subject );
    static tgPolygon RemoveTinyContours( const tgPolygon& subject );
    static tgPolygon RemoveSpikes( const tgPolygon& subject );
    static void           RemoveSlivers( tgPolygon& subject, tgcontour_list& slivers );
    static tgcontour_list MergeSlivers( tgpolygon_list& subjects, tgcontour_list& slivers );

    static tgPolygon Expand( const tgPolygon& subject, double offset );
    static tgPolygon Expand( const SGGeod& subject, double offset );

    // Conversions
    static ClipperLib::Paths ToClipper( const tgPolygon& subject );
    static tgPolygon FromClipper( const ClipperLib::Paths& subject );
    static tgPolygon FromClipper( const ClipperLib::PolyTree& subject );
    static void ToClipperFile( const tgPolygon& subject, const std::string& path, const std::string& filename );
    
    // T-Junctions and segment search
    static tgPolygon AddColinearNodes( const tgPolygon& subject, UniqueSGGeodSet& nodes );
    static tgPolygon AddColinearNodes( const tgPolygon& subject, std::vector<SGGeod>& nodes );
    static bool      FindColinearLine( const tgPolygon& subject, SGGeod& node, SGGeod& start, SGGeod& end );

    static tgPolygon AddColinearNodes( const tgPolygon& subject, std::vector<TGNode*>& nodes );
    
    // IO
    void SaveToGzFile( gzFile& fp ) const;
    int LoadFromGzFile( gzFile& fp );

    friend std::ostream& operator<< ( std::ostream&, const tgPolygon& );

private:
    tgcontour_list  contours;
    tgtriangle_list triangles;

    std::string     material;
    std::string     flag;       // let's get rid of this....
    bool            preserve3d;
    unsigned int    id;         // unique polygon id for debug
    tgTexParams     tp;
    bool            closed;       // if we treat it as a closed shape
};

#endif // _POLYGON_HXX
