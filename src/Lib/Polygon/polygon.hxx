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
#include <string>
#include <vector>

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>

#include <Polygon/point3d.hxx>

// forward declaration
class TGPolygon;

#include "clipper.hpp"
#define FG_MAX_VERTICES 1500000

typedef std::vector < point_list > polytype;
typedef polytype::iterator polytype_iterator;
typedef polytype::const_iterator const_polytype_iterator;

class TGPolygon {

private:

    polytype poly;                  // polygons
    point_list inside_list;         // point inside list
    int_list hole_list;             // hole flag list

public:

    // Constructor and destructor
    TGPolygon( void );
    ~TGPolygon( void );

    // Add a contour
    inline void add_contour( const point_list contour, const int hole_flag )
    {
        poly.push_back( contour );
        inside_list.push_back( Point3D( 0.0 ) );
        hole_list.push_back( hole_flag );
    }

    // Get a contour
    inline point_list get_contour( const int i ) const
    {
        return poly[i];
    }

    // Delete a contour
    inline void delete_contour( const int i )
    {
        polytype_iterator start_poly = poly.begin();

        poly.erase( start_poly + i );

        point_list_iterator start_inside = inside_list.begin();
        inside_list.erase( start_inside + i );

        int_list_iterator start_hole = hole_list.begin();
        hole_list.erase( start_hole + i );
    }

    // Add the specified node (index) to the polygon
    inline void add_node( int contour, Point3D p )
    {
        if ( contour >= (int)poly.size() ) {
            // extend polygon
            point_list empty_contour;
            empty_contour.clear();
            for ( int i = 0; i < contour - (int)poly.size() + 1; ++i ) {
                poly.push_back( empty_contour );
                inside_list.push_back( Point3D(0.0) );
                hole_list.push_back( 0 );
            }
        }
        poly[contour].push_back( p );
    }

    // return size
    inline int contours() const
    {
        return poly.size();
    }
    inline int contour_size( int contour ) const
    {
        return poly[contour].size();
    }
    inline int total_size() const
    {
        int size = 0;

        for ( int i = 0; i < contours(); ++i )
            size += poly[i].size();
        return size;
    }

    // return the ith point from the specified contour
    inline Point3D get_pt( int contour, int i ) const
    {
        return poly[contour][i];
    }

    // update the value of a point
    inline void set_pt( int contour, int i, const Point3D& p )
    {
        poly[contour][i] = p;
    }

    void get_bounding_box( SGGeod& min, SGGeod& max ) const;

    // get and set an arbitrary point inside the specified polygon contour
    inline Point3D get_point_inside( const int contour ) const
    {
        return inside_list[contour];
    }
    inline void set_point_inside( int contour, const Point3D& p )
    {
        inside_list[contour] = p;
    }

    // get and set hole flag
    inline int get_hole_flag( const int contour ) const
    {
        return hole_list[contour];
    }
    inline void set_hole_flag( const int contour, const int flag )
    {
        hole_list[contour] = flag;
    }
    inline bool has_holes() const
    {
        for (int i = 0; i < contours(); i++)
            if (get_hole_flag(i))
                return true;
        return false;
    }

    // Set the elevations of points in the current polgyon based on
    // the elevations of points in source.  For points that are not
    // found in source, propogate the value from the nearest matching
    // point.
    void inherit_elevations( const TGPolygon &source );

    // Set the elevations of all points to the specified values
    void set_elevations( double elev );

    // shift every point in the polygon by lon, lat
    void shift( double lon, double lat );

    // erase
    inline void erase()
    {
        poly.clear();
    }

    // informational

    // return the area of a contour (assumes simple polygons,
    // i.e. non-self intersecting.)
    //
    // negative areas indicate counter clockwise winding
    // positive areas indicate clockwise winding.
    double area_contour( const int contour ) const;

    // return the smallest interior angle of the contour
    double minangle_contour( const int contour );

    // return true if contour B is inside countour A
    bool is_inside( int a, int b ) const;

    // output
    void write( const std::string& file ) const;

    // output
    void write_contour( const int contour, const std::string& file ) const;

    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    // Friends for serialization
    friend std::istream& operator>> ( std::istream&, TGPolygon& );
    friend std::ostream& operator<< ( std::ostream&, const TGPolygon& );
};


typedef std::vector < TGPolygon > poly_list;
typedef poly_list::iterator poly_list_iterator;
typedef poly_list::const_iterator const_poly_list_iterator;


// Calculate theta of angle (a, b, c)
double tgPolygonCalcAngle(SGVec2d a, SGVec2d b, SGVec2d c);


// canonify the polygon winding, outer contour must be anti-clockwise,
// all inner contours must be clockwise.
TGPolygon tgPolygonCanonify( const TGPolygon& in_poly );


// Traverse a polygon and split edges until they are less than max_len
// (specified in meters)
TGPolygon tgPolygonSplitLongEdges( const TGPolygon &poly, double max_len );


// Traverse a polygon and toss all the internal holes
TGPolygon tgPolygonStripHoles( const TGPolygon &poly );


// Wrapper for the fast Polygon Triangulation based on Seidel's
// Algorithm by Atul Narkhede and Dinesh Manocha
// http://www.cs.unc.edu/~dm/CODE/GEM/chapter.html

TGPolygon tgPolygon2tristrip( const TGPolygon& poly );

void tgPolygonFindSlivers( TGPolygon& in, poly_list& slivers );


// Difference
TGPolygon tgPolygonDiff( const TGPolygon& subject, const TGPolygon& clip );

// Intersection
TGPolygon tgPolygonInt( const TGPolygon& subject, const TGPolygon& clip );

// Exclusive or
TGPolygon tgPolygonXor( const TGPolygon& subject, const TGPolygon& clip );

// Union
TGPolygon tgPolygonUnion( const TGPolygon& subject, const TGPolygon& clip );
TGPolygon tgPolygonUnion( const poly_list& clips );

// wrapper for clipper clip routines

void tgPolygonInitClipperAccumulator( void );
void tgPolygonFreeClipperAccumulator( void );
void tgPolygonDumpAccumulator( char* ds, char* layer, char*name ); 
void tgPolygonAddToClipperAccumulator( const TGPolygon& subject, bool dump );
TGPolygon tgPolygonDiffClipperWithAccumulator( const TGPolygon& subject );

// Save clipper to shapefile
void clipper_to_shapefile( ClipperLib::Polygons polys, char* datasource );

// Difference
TGPolygon tgPolygonDiffClipper( const TGPolygon& subject, const TGPolygon& clip );

// Intersection
TGPolygon tgPolygonIntClipper( const TGPolygon& subject, const TGPolygon& clip );

// Union
TGPolygon tgPolygonUnionClipper( const TGPolygon& subject, const TGPolygon& clip );

// Expand / Shrink
TGPolygon tgPolygonExpand(const TGPolygon &poly, double delta);

// Simplify
TGPolygon tgPolygonSimplify(const TGPolygon &poly);

void tgPolygonDumpClipper(const TGPolygon &poly);

// Output
std::ostream &operator<<(std::ostream &output, const TGPolygon &poly);





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
#include <ogrsf_frmts.h>
#include <boost/concept_check.hpp>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/threads/SGThread.hxx>

#include <Polygon/trinodes.hxx>
#include <Polygon/rectangle.hxx>

#include "tg_unique_geod.hxx"

// utilities - belong is simgear?
double CalculateTheta( const SGVec3d& dirCur, const SGVec3d& dirNext, const SGVec3d& cp );
SGGeod midpoint( const SGGeod& p0, const SGGeod& p1 );
SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, int& turn_dir );
SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by );
SGGeod OffsetPointFirst( const SGGeod& cur, const SGGeod& next, double offset_by );
SGGeod OffsetPointLast( const SGGeod& prev, const SGGeod& cur, double offset_by );

// forward declaration
// Forward Declaration:
class tgPolygon;
class tgChoppedPolygons;

typedef std::vector <tgPolygon>  tgpolygon_list;
typedef tgpolygon_list::iterator tgpolygon_list_iterator;
typedef tgpolygon_list::const_iterator const_tgpolygon_list_iterator;

class tgContour
{
public:
    tgContour() {
        hole = false;
    }

    void Erase() {
        node_list.clear();
    }

    void SetHole( bool h ) {
        hole = h;
    }
    bool GetHole( void ) const {
        return hole;
    }

    unsigned int GetSize( void ) const {
        return node_list.size();
    }

    void Resize( int size ) {
        node_list.resize( size );
    }

    void AddNode( SGGeod n ) {
        node_list.push_back( n );
    }
    void SetNode( unsigned int i, SGGeod n ) {
        node_list[i] = n;
    }
    SGGeod GetNode( unsigned int i ) const {
        return node_list[i];
    }
    SGGeod const& operator[]( int index ) const {
        return node_list[index];
    }

    void RemoveNodeAt( unsigned int idx ) {
        if ( idx < node_list.size() ) {
            node_list.erase( node_list.begin() + idx );
        }
    }
    void RemoveNodeRange( unsigned int from, unsigned int to ) {
        if ( ( from < to ) && ( to < node_list.size() ) ) {
            node_list.erase( node_list.begin()+from,node_list.begin()+to );
        }
    }

    tg::Rectangle GetBoundingBox( void ) const;

    double GetMinimumAngle( void ) const;
    double GetArea( void ) const;

    static tgContour Snap( const tgContour& subject, double snap );
    static tgContour RemoveDups( const tgContour& subject );
    static tgContour RemoveCycles( const tgContour& subject );
    static tgContour SplitLongEdges( const tgContour& subject, double dist );
    static tgContour RemoveSpikes( const tgContour& subject );

    static tgPolygon Diff( const tgContour& subject, tgPolygon& clip );

    static tgContour AddColinearNodes( const tgContour& subject, TGTriNodes nodes );
    static tgContour AddColinearNodes( const tgContour& subject, std::vector<SGGeod>& nodes );
    static bool      FindColinearLine( const tgContour& subject, const SGGeod& node, SGGeod& start, SGGeod& end );

    // conversions
    static ClipperLib::Polygon ToClipper( const tgContour& subject );
    static tgContour FromClipper( const ClipperLib::Polygon& subject );

    static tgContour Expand( const tgContour& subject, double offset );
    static tgpolygon_list ExpandToPolygons( const tgContour& subject, double width );

    static void ToShapefile( const tgContour& subject, const std::string& datasource, const std::string& layer, const std::string& feature );

    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgContour& );

private:
    std::vector<SGGeod>  node_list;
    bool hole;
};

typedef std::vector <tgContour>  tgcontour_list;
typedef tgcontour_list::iterator tgcontour_list_iterator;
typedef tgcontour_list::const_iterator const_tgcontour_list_iterator;

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
    TG_TEX_BY_TPS_CLIPUV
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

    tgTexMethod method;

    double center_lat;

    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgTexParams& );
};

class tgPolygon
{
public:
    tgPolygon() {
        preserve3d = false;
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

    tg::Rectangle GetBoundingBox( void ) const;

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

    void Tesselate( void );
    void Tesselate( const std::vector<SGGeod>& extra );

    void Texture( void );

    // Boolean operations

    // TODO : Both should be constant
    // what we really need is multiple accumulators
    // init_accumulator should return a handle...
    static bool      ChopIdxInit( const std::string& path );

    static void      SetDump( bool dmp );
    static tgPolygon Union( const tgContour& subject, tgPolygon& clip );
    static tgPolygon Union( const tgPolygon& subject, tgPolygon& clip );
    static tgPolygon Union( const tgpolygon_list& polys );
    static tgPolygon Diff( const tgPolygon& subject, tgPolygon& clip );
    static tgPolygon Intersect( const tgPolygon& subject, const tgPolygon& clip );

    // Conversions
    static ClipperLib::Polygons ToClipper( const tgPolygon& subject );
    static tgPolygon FromClipper( const ClipperLib::Polygons& subject );

    static tgPolygon FromOGR( const OGRPolygon* subject );

    // other operations
    static tgPolygon Snap( const tgPolygon& subject, double snap );
    static tgPolygon StripHoles( const tgPolygon& subject );
    static tgPolygon SplitLongEdges( const tgPolygon& subject, double dist );

    static tgPolygon RemoveCycles( const tgPolygon& subject );
    static tgPolygon RemoveDups( const tgPolygon& subject );
    static tgPolygon RemoveBadContours( const tgPolygon& subject );
    static tgPolygon Simplify( const tgPolygon& subject );
    static tgPolygon RemoveTinyContours( const tgPolygon& subject );
    static tgPolygon RemoveSpikes( const tgPolygon& subject );

    static tgPolygon Expand( const tgPolygon& subject, double offset );
    static tgPolygon Expand( const SGGeod& subject, double offset );
    
    static void ToShapefile( const tgPolygon& subject, const std::string& datasource, const std::string& layer, const std::string& feature );

    static void Tesselate( const tgPolygon& subject );

    static tgPolygon AddColinearNodes( const tgPolygon& subject, TGTriNodes& nodes );
    static tgPolygon AddColinearNodes( const tgPolygon& subject, std::vector<SGGeod>& nodes );
    static bool      FindColinearLine( const tgPolygon& subject, SGGeod& node, SGGeod& start, SGGeod& end );

    static void RemoveSlivers( tgPolygon& subject, tgcontour_list& slivers );
    static tgcontour_list MergeSlivers( tgpolygon_list& subjects, tgcontour_list& slivers );

    void SaveToGzFile( gzFile& fp ) const;
    void LoadFromGzFile( gzFile& fp );

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgPolygon& );

private:
    tgcontour_list  contours;
    tgtriangle_list triangles;

    std::string     material;
    std::string     flag;       // let's get rid of this....
    bool            preserve3d;
    unsigned int    id;         // unique polygon id for debug
    tgTexParams     tp;
};

// for ogr-decode : generate a bunch of polygons, mapped by bucket id
typedef std::map<long int, tgpolygon_list> bucket_polys_map;
typedef bucket_polys_map::iterator bucket_polys_map_interator;

class tgChopper
{
public:
    tgChopper( const std::string& path ) {
        root_path = path;
    }

    void Add( const tgPolygon& poly, const std::string& type ); 
    void Save( void );

private:
    long int GenerateIndex( std::string path );
    void Clip( const tgPolygon& subject, const std::string& type, SGBucket& b );
    void Chop( const tgPolygon& subject, const std::string& type );

    std::string      root_path;
    bucket_polys_map bp_map;
    SGMutex          lock;
};

class tgLight
{
public:
    SGGeod      pos;
    SGVec3f     norm;
};

typedef std::vector <tgLight>  tglight_list;
typedef tglight_list::iterator tglight_list_iterator;
typedef tglight_list::const_iterator const_tglight_list_iterator;

class tgLightContour
{
public:
    unsigned int ContourSize( void ) const {
        return lights.size();
    }

    void AddLight( SGGeod p, SGVec3f n ) {
        tgLight light;
        light.pos  = p;
        light.norm = n;
        lights.push_back(light);
    }

    void SetElevation( unsigned int i, double elev ) {
        lights[i].pos.setElevationM( elev );
    }

    SGGeod GetNode( unsigned int i ) const {
        return lights[i].pos;
    }

    SGGeod GetPosition( unsigned int i ) const {
        return lights[i].pos;
    }

    SGVec3f GetNormal( unsigned int i ) const {
        return lights[i].norm;
    }

    std::string GetFlag( void ) const {
        return flag;
    }
    void SetFlag( const std::string f ) {
        flag = f;
    }

    std::string GetType( void ) const {
        return type;
    }
    void SetType( const std::string t ) {
        type = t;
    }

    // TEMP TEMP TEMP : need to redo Airport.cxx methods
    point_list TempGetPosListAsPoint3D( void ) {
        point_list p3dlist;

        for (unsigned int i=0; i<lights.size(); i++) {
            p3dlist.push_back( Point3D::fromSGGeod( lights[i].pos ) );
        }

        return p3dlist;
    }

    std::vector<SGGeod> GetPositionList( void ) {
        std::vector<SGGeod> positions;

        for (unsigned int i=0; i<lights.size(); i++) {
            positions.push_back( lights[i].pos );
        }

        return positions;
    }

    point_list TempGetNormalListAsPoint3D( void ) {
        point_list p3dlist;

        for (unsigned int i=0; i<lights.size(); i++) {
            p3dlist.push_back( Point3D::fromSGVec3( lights[i].norm ) );
        }

        return p3dlist;
    }

    std::vector<SGVec3f> GetNormalList( void ) {
        std::vector<SGVec3f> normals;

        for (unsigned int i=0; i<lights.size(); i++) {
            normals.push_back( lights[i].norm );
        }

        return normals;
    }

    // END TEMP TEMP TEMP

    // Friend for output
    friend std::ostream& operator<< ( std::ostream&, const tgLightContour& );

    std::string  type;
    std::string  flag;
    tglight_list lights;
};

typedef std::vector <tgLightContour>  tglightcontour_list;
typedef tglightcontour_list::iterator tglightcontour_list_iterator;
typedef tglightcontour_list::const_iterator const_tglightcontour_list_iterator;

typedef std::vector < ClipperLib::Polygons > clipper_polygons_list;

class tgAccumulator
{
public:
    tgPolygon Diff( const tgContour& subject );
    tgPolygon Diff( const tgPolygon& subject );

    void      Add( const tgContour& subject );
    void      Add( const tgPolygon& subject );

    void      ToShapefiles( const std::string& path, const std::string& layer );

private:
    clipper_polygons_list   accum;
};

class tgShapefile
{
public:
    static void Init( void );
    static void* OpenDatasource( const char* datasource_name );
    static void* OpenLayer( void* ds_id, const char* layer_name );
    static void  CreateFeature( void* ds_id, void* l_id, const TGPolygon &poly, const char* description );
    static void  CloseLayer( void* l_id );
    static void* CloseDatasource( void* ds_id );
};


#endif // _POLYGON_HXX


