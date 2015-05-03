#ifndef _TG_NODES_HXX
#define _TG_NODES_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include <cstdlib>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/algorithm.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>     /* Just use two dimensional lookup - elevation is a trait */
#include <boost/iterator/zip_iterator.hpp>

#include <simgear/compiler.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/io/lowlevel.hxx>

#include "tg_triangle.hxx"
//#include "tg_unique_tgnode.hxx"
#include "tg_surface.hxx"
#include "tg_array.hxx"

class tgSurface;

#define FG_PROXIMITY_EPSILON 0.000001
#define FG_COURSE_EPSILON 0.0001


typedef enum {
    TG_NODE_FIXED_ELEVATION = 0,    // elevation is fixed, and must not change - it's in the node
    TG_NODE_INTERPOLATED,           // calc elevation from interpolating height field data
    TG_NODE_SMOOTHED,               // elevation is calculated via a tgSurface
    TG_NODE_DRAPED                  // elevation is interpolated via a triangle list
} tgNodeType;

// for each node, we'll need a vector to lookup all triangles the node
// is a member of.
struct TGFaceLookup {
    unsigned int    area;
    unsigned int    poly;
    unsigned int    tri;
};
typedef std::vector < TGFaceLookup > TGFaceList;

class TGNode {
public:
    TGNode() {
        // constructor for serialization only
        type = TG_NODE_INTERPOLATED;
        used = false;
    }
    
    TGNode( SGGeod p, tgNodeType t = TG_NODE_INTERPOLATED ) {
        position    = p;
        CalcWgs84();
        
        type = t;
        faces.clear();
        used = false;
    }
    
    inline void SetType( tgNodeType t )
    {
        if (type != TG_NODE_FIXED_ELEVATION) {
            type = t;
        }
    }
    
    inline tgNodeType GetType() const {
        return type;
    }
    
    inline void CalcWgs84()
    {
        wgs84 = SGVec3d::fromGeod(position);
    }
    
    inline void AddFace( unsigned int area, unsigned int poly, unsigned int tri )
    {
        TGFaceLookup    face;
        face.area   = area;
        face.poly   = poly;
        face.tri    = tri;
        
        faces.push_back( face );
    }
    
    inline TGFaceList const& GetFaces( void ) const { return faces; }
    inline bool IsFixedElevation( void ) const      { return (type == TG_NODE_FIXED_ELEVATION); }
    inline SGVec3d const& GetWgs84( void ) const    { return wgs84; }
    
    inline void SetPosition( const SGGeod& p )
    {
        if (type != TG_NODE_FIXED_ELEVATION) {
            position = p;
            CalcWgs84();
        }
    }
    
    inline void SetElevation( double z )
    {
        if (type != TG_NODE_FIXED_ELEVATION) {
            position.setElevationM( z );
            CalcWgs84();
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "TGNode::SetElevations - REFUSE - type is " << type << " cur elevation is " << position.getElevationM() << " new would be " << z );
        }
    }
    
    inline bool IsUsed( void ) const {
        return used;
    }
    
    inline void SetUsed()
    {
        used = true;
    }
    
    inline SGGeod const&  GetPosition( void ) const        { return position; }
    inline void    SetNormal( const SGVec3f& n )    { normal = n; }
    inline SGVec3f GetNormal( void ) const          { return normal; }
    
    void SaveToGzFile( gzFile& fp ) {
        sgWriteGeod( fp, position );
        sgWriteInt( fp, (int)type );
        
        // Don't save the facelist per node
        // it's much faster to just redo the lookup
    }
    
    void LoadFromGzFile( gzFile& fp ) {
        int temp;
        
        sgReadGeod( fp, position );
        CalcWgs84();
        
        sgReadInt( fp, &temp );
        type = (tgNodeType)temp;        
    }
    
    // Friends for serialization
    friend std::ostream& operator<< ( std::ostream&, const TGNode& );
    
private:
    SGGeod      position;
    SGVec3f     normal;
    SGVec3d     wgs84;
    tgNodeType  type;
    bool        used;
    
    TGFaceList  faces;
};

typedef CGAL::Simple_cartesian<double>                                                                          TGNodeKernel;
typedef TGNodeKernel::Point_2                                                                                   TGNodePoint;
typedef boost::tuple<TGNodePoint,double,unsigned int,TGNode*>                                                   TGNodeData;
typedef CGAL::Search_traits_2<TGNodeKernel>                                                                     TGNodeTraitsBase;
typedef CGAL::Search_traits_adapter<TGNodeData,CGAL::Nth_of_tuple_property_map<0, TGNodeData>,TGNodeTraitsBase> TGNodeTraits;

typedef CGAL::Fuzzy_iso_box<TGNodeTraits>                                                                       TGNodeFuzzyBox;
typedef CGAL::Fuzzy_sphere<TGNodeTraits>                                                                        TGNodeFuzzyCir;
typedef CGAL::Kd_tree<TGNodeTraits>                                                                             TGNodeTree;


/* This class handles ALL of the nodes in a tile : 3d nodes in elevation data, 2d nodes generated from landclass, etc) */
class TGNodes {
public:

    // Constructor and destructor
    TGNodes( void )     {
        array         = NULL;
        tris          = NULL;
    }

    ~TGNodes( void )    {
        tg_node_list.clear();
        tg_kd_tree.clear();
    }

    // delete all the data out of node_list
    inline void clear() {
        tg_node_list.clear();
        tg_kd_tree.clear();
    }

    // Add a point to the point list if it doesn't already exist.
    // Returns the index (starting at zero) of the point in the list.
    unsigned int unique_add( const SGGeod& p, tgNodeType t = TG_NODE_INTERPOLATED ) {
        static unsigned int calls = 0;
        
        std::list<TGNodeData>   searchResults;
        unsigned int            index;
        
        // first - create a search node
        TGNodePoint     pt( p.getLongitudeDeg(), p.getLatitudeDeg() );
        TGNodeFuzzyCir  query_circle( pt, 0.0000001 );  // approx 1 cm
        
        // perform the query
        searchResults.clear();
        tg_kd_tree.search(std::back_inserter( searchResults ), query_circle);
        
        if ( searchResults.empty() ) {
            // no node here - add a new one
            index = tg_node_list.size();
            double e = p.getElevationM();
            
            tg_node_list.push_back( TGNode(p,t) );
            
            TGNodeData data(pt, e, index, &tg_node_list[index]);
            
            tg_kd_tree.insert(data);            
        } else {
            // we found a node - use it
            std::list<TGNodeData>::const_iterator it = searchResults.begin();
            index = boost::get<2>(*it);
        }
        
#if 0        
        calls++;
        if ( calls % 100 == 0 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "TGNode::unique_add called " << calls << " times ");
        }
#endif        
        
        return index;
    }

    // Find the index of the specified point (compair to the same
    // tolerance as unique_add().  Returns -1 if not found.
    int find(  const SGGeod& p ) const {
        std::list<TGNodeData>   searchResults;
        int index = -1;
        static unsigned int calls = 0;
        
        // first - create a search node
        TGNodePoint     pt( p.getLongitudeDeg(), p.getLatitudeDeg() );
        TGNodeFuzzyCir  query_circle( pt, 0.0000001 );  // approx 1 cm
        
        // perform the query
        searchResults.clear();
        tg_kd_tree.search(std::back_inserter( searchResults ), query_circle);
        
        if ( !searchResults.empty() ) {
            // we found a node - use it
            std::list<TGNodeData>::const_iterator it = searchResults.begin();
            index = (int)boost::get<2>(*it);
        }

#if 0        
        calls++;
        if ( calls % 100 == 0 ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "TGNode::find called " << calls << " times ");
        }
#endif

        return index;
    }

    void SetUsed( int idx ) {
        tg_node_list[idx].SetUsed();
    }
    
    void init_spacial_query( void );

    void SetArray( tgArray* a ) {
        array = a;
    }
    void SetTriangles( tgtriangle_list* t ) {
        tris = t;
    }

    void SetElevation( int idx, double z )  { tg_node_list[idx].SetElevation( z ); }
    void CalcElevations( tgNodeType type );
    void CalcElevations( tgNodeType type, const tgSurface& surf );
    void CalcElevations( tgNodeType type, const tgtriangle_list& mesh );

    void DeleteUnused( void );
    
    SGVec3f GetNormal( int idx ) const      { return tg_node_list[idx].GetNormal(); }
    void SetNormal( int idx, SGVec3f n )    { tg_node_list[idx].SetNormal( n ); }

    // return a point list of geodetic nodes
    void get_geod_nodes( std::vector<SGGeod>& points ) const;

    // Find all the nodes within a bounding box
    bool get_geod_inside( const SGGeod& min, const SGGeod& max, std::vector<SGGeod>& points ) const;

    // Find all the nodes within a bounding box
    bool get_nodes_inside( const SGGeod& min, const SGGeod& max, std::vector<TGNode*>& points ) const;
    
    // Find a;; the nodes on the tile edges
    bool get_geod_edge( const SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west ) const;

    // return a point list of wgs84 nodes
    void get_wgs84_nodes( std::vector<SGVec3d>& points ) const;

    // return a point list of normals
    void get_normals( std::vector<SGVec3f>& normals ) const;

    // return the ith point (constant)
    inline TGNode const& get_node( int i ) const { return tg_node_list[i]; }
    inline TGNode& get_node( int i ) { return tg_node_list[i]; }

    TGNode const& operator[]( int index ) const {
        return tg_node_list[index];
    }
    TGNode& operator[]( int index ) {
        return tg_node_list[index];
    }

    inline void AddFace( int i, unsigned int area, unsigned int poly, unsigned int tri )
    {
        tg_node_list[i].AddFace( area, poly, tri );
    }

    // return the size of the node list
    inline size_t size() const { return tg_node_list.size(); }

    void Dump( void );
    void ToShapefile( const std::string& datasourse );
    
    void SaveToGzFile( gzFile& fp );
    void LoadFromGzFile( gzFile& fp );

private:
    //UniqueTGNodeSet tg_node_list;
    std::vector<TGNode> tg_node_list;
    TGNodeTree          tg_kd_tree;
    //bool            kd_tree_valid;
    //double          tex_v;

    // temp pointers - not serialized
    tgArray*            array;      // for interpolated elevation
    tgtriangle_list*    tris;       // for draped elevation
};

#endif // _TG_NODES_HXX
