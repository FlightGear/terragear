#ifndef _TG_NODES_HXX
#define _TG_NODES_HXX

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifndef __cplusplus
# error This library requires C++
#endif

#include <cstdlib>
#include <simgear/compiler.h>
#include <simgear/bucket/newbucket.hxx>
//#include <simgear/math/sg_types.hxx>
#include <simgear/io/lowlevel.hxx>

#define FG_PROXIMITY_EPSILON 0.000001
#define FG_COURSE_EPSILON 0.0001

// for each node, we'll need a vector to lookup all triangles the node
// is a member of.
struct TGFaceLookup {
    unsigned int    area;
    unsigned int    shape;
    unsigned int    seg;
    unsigned int    tri;
};
typedef std::vector < TGFaceLookup > TGFaceList;


class TGNode {
public:
    TGNode() {
        // constructor for serialization only
    }

    TGNode( SGGeod p ) {
        position    = p;
        CalcWgs84();

        fixed_position  = false;        // no matter what - don't move x, y, or z (likely a hole around an airport generated ny genapts)
        fixed_normal    = false;        // no matter what - don't modify the normal - likely on a normal generated on a shared edge 

        faces.clear();
    }

    inline void SetFixedPosition( bool fix )        
    { 
        if (!fixed_position) {
            fixed_position = fix;
        }
    }

    inline void CalcWgs84()
    {
        wgs84 = SGVec3d::fromGeod(position);
    }

    inline void AddFace( unsigned int area, unsigned int shape, unsigned int segment, unsigned int tri )
    {
        TGFaceLookup    face;
        face.area   = area;
        face.shape  = shape;
        face.seg    = segment;
        face.tri    = tri;

        faces.push_back( face );
    }

    inline TGFaceList GetFaces( void ) const        { return faces; }
    inline bool GetFixedPosition( void ) const      { return fixed_position; }
    inline void SetFixedNormal( bool fix )          { fixed_normal = fix; }
    inline bool GetFixedNormal( void ) const        { return fixed_normal; }
    inline SGVec3d GetWgs84( void ) const           { return wgs84; }

    inline void    SetPosition( const SGGeod& p )
    { 
        if (!fixed_position) {
            position = p; 
            CalcWgs84();
        }
    }

    inline void    SetElevation( double z )
    { 
        if (!fixed_position) {
            position.setElevationM( z );
            CalcWgs84();
        }
    }

    inline SGGeod  GetPosition( void ) const        { return position; }
    inline void    SetNormal( const SGVec3f& n )    { normal = n; }
    inline SGVec3f GetNormal( void ) const          { return normal; }

    void SaveToGzFile( gzFile& fp );
    void LoadFromGzFile( gzFile& fp );
    // Friends for serialization
    friend std::ostream& operator<< ( std::ostream&, const TGNode& );

private:
    SGGeod      position;
    SGVec3f     normal;
    SGVec3d     wgs84;

    bool        fixed_position;
    bool        fixed_normal;

    TGFaceList  faces;
};
typedef std::vector < TGNode > node_list;
typedef node_list::iterator node_list_iterator;
typedef node_list::const_iterator const_node_list_iterator;

/* This class handles ALL of the nodes in a tile : 3d nodes in elevation data, 2d nodes generated from landclass, etc) */
class TGNodes {
public:

    // Constructor and destructor
    TGNodes( void ) {
        sorted = false;
    }
    ~TGNodes( void )    {}

    // delete all the data out of node_list
    inline void clear() {
        tg_node_list.clear();
        sorted = false;
    }

    // Add a point to the point list if it doesn't already exist.
    // Returns the index (starting at zero) of the point in the list.
    void unique_add( const SGGeod& p ) {
        if ( !sorted ) {
            linear_unique_add( p );
        } else {
             SG_LOG(SG_GENERAL, SG_ALERT, "ADDING NODE AFTER SORT!");
             exit(0);
        }
    }

    // Add a point to the point list if it doesn't already exist
    // (checking all three dimensions.)  Returns the index (starting
    // at zero) of the point in the list.
    void unique_add_fixed_elevation( const SGGeod& p ) {
        if ( !sorted ) {
            linear_unique_add_fixed_elevation( p );
        } else {
             SG_LOG(SG_GENERAL, SG_ALERT, "ADDING NODE AFTER SORT!");
             exit(0);
        }
    }

    // Find the index of the specified point (compair to the same
    // tolerance as unique_add().  Returns -1 if not found.
    int find(  const SGGeod& p ) const {
        if ( sorted ) {
            return sorted_find( p );
        } else {
            return linear_find( p );
        }
    }

    void SortNodes( void );
    void SetElevation( int idx, double z )  { tg_node_list[idx].SetElevation( z ); }

    SGVec3f GetNormal( int idx ) const      { return tg_node_list[idx].GetNormal(); }
    void SetNormal( int idx, SGVec3f n )    { tg_node_list[idx].SetNormal( n ); }

     // return the master node list
    inline node_list& get_node_list() { return tg_node_list; }
    inline const node_list& get_node_list() const { return tg_node_list; }

    // return a point list of geodetic nodes
    void get_geod_nodes( std::vector<SGGeod>& points ) const;

    // Find all the nodes within a bounding box
    void get_geod_inside( const SGGeod& min, const SGGeod& max, std::vector<SGGeod>& points ) const;

    // Find a;; the nodes on the tile edges
    void get_geod_edge( const SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west ) const;

    // return a point list of wgs84 nodes
    void get_wgs84_nodes( std::vector<SGVec3d>& points ) const;

    // return a point list of normals
    void get_normals( std::vector<SGVec3f>& normals ) const;

    // return the ith point
    inline TGNode get_node( int i ) const { return tg_node_list[i]; }

    inline void AddFace( int i, unsigned int area, unsigned int shape, unsigned int segment, unsigned int tri )
    {
        tg_node_list[i].AddFace( area, shape, segment, tri );        
    }

    // return the size of the node list
    inline size_t size() const { return tg_node_list.size(); }

    void Dump( void );
    void SaveToGzFile( gzFile& fp );
    void LoadFromGzFile( gzFile& fp );

    // Friends for serialization
    friend std::ostream& operator<< ( std::ostream&, const TGNodes& );

private:
    void linear_unique_add( const SGGeod& p );
    void linear_unique_add_fixed_elevation( const SGGeod& p );

    int sorted_find( const SGGeod& p ) const;
    int linear_find( const SGGeod& p ) const;
    node_list tg_node_list;
    bool sorted;

    // return true of the two points are "close enough" as defined by
    // FG_PROXIMITY_EPSILON
    bool close_enough_2d( const SGGeod& p1, const SGGeod& p2 ) const;

    // return true of the two points are "close enough" as defined by
    // FG_PROXIMITY_EPSILON
    bool close_enough_3d( const SGGeod& p1, const SGGeod& p2 ) const;

    // return true of the two points are "close enough" as defined by
    // FG_COURSE_EPSILON
    bool course_close_enough( const SGGeod& p1, const SGGeod& p2 );
};


// return true of the two points are "close enough" as defined by
// FG_PROXIMITY_EPSILON checking just x and y dimensions
inline bool TGNodes::close_enough_2d( const SGGeod& p1, const SGGeod& p2 )
    const
{
    if ( ( fabs(p1.getLongitudeDeg() - p2.getLongitudeDeg()) < FG_PROXIMITY_EPSILON ) &&
         ( fabs(p1.getLatitudeDeg()  - p2.getLatitudeDeg()) < FG_PROXIMITY_EPSILON ) ) {
        return true;
    } else {
        return false;
    }
}

// return true of the two points are "close enough" as defined by
// FG_PROXIMITY_EPSILON check all three dimensions
inline bool TGNodes::close_enough_3d( const SGGeod& p1, const SGGeod& p2 )
    const
{
    if ( ( fabs(p1.getLongitudeDeg() - p2.getLongitudeDeg()) < FG_PROXIMITY_EPSILON ) &&
         ( fabs(p1.getLatitudeDeg()  - p2.getLatitudeDeg()) < FG_PROXIMITY_EPSILON ) &&
         ( fabs(p1.getElevationM()   - p2.getElevationM()) < FG_PROXIMITY_EPSILON ) ) {
        return true;
    } else {
        return false;
    }
}

// return true of the two points are "close enough" as defined by
// FG_COURSE_EPSILON
inline bool TGNodes::course_close_enough( const SGGeod& p1, const SGGeod& p2 )
{
    if ( ( fabs(p1.getLongitudeDeg() - p2.getLongitudeDeg()) < FG_COURSE_EPSILON ) &&
         ( fabs(p1.getLatitudeDeg()  - p2.getLatitudeDeg()) < FG_COURSE_EPSILON ) ) {
        return true;
    } else {
        return false;
    }
}

#endif // _TG_NODES_HXX
