#ifndef _TG_UNIQUE_TGNODE_HXX
#define _TG_UNIQUE_TGNODE_HXX

#include <boost/unordered_set.hpp>
#include <boost/concept_check.hpp>

#include <simgear/math/SGGeod.hxx>
#include <simgear/math/SGMisc.hxx>

// Implement Unique TGNode list

// We use a hash to store the indices.  All iterators returned from find are
// constant, because modifying a value will modify the hash - rendering the
// set invalid.
// Selection of the bucket is tightly linked to the key equality function.
// If any value is considered equal to another, than the hash function MUST
// compute the same hash for each value.
// So close_enough_2d will remain our testo of equality.  It simply rounds to
// 6 decimal places.  Our hash function will round to 5, so at most 10 points in the
// same bucket.
// We could experiment with making it so 1 point per bucket, but I'm nervous...

#define PROXIMITY_MULTIPLIER (100000)
#define PROXIMITY_EPSILON    ((double) 1 / (double)( PROXIMITY_MULTIPLIER * 10 ) )

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
    }

    explicit TGNode( SGGeod p ) :
        position(p)
    {
        CalcWgs84();

        fixed_position  = false;        // no matter what - don't move x, y, or z (likely a hole around an airport generated ny genapts)
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

    inline void AddFace( unsigned int area, unsigned int poly, unsigned int tri )
    {
        TGFaceLookup    face;
        face.area   = area;
        face.poly   = poly;
        face.tri    = tri;

        faces.push_back( face );
    }

    inline TGFaceList const& GetFaces( void ) const { return faces; }
    inline bool GetFixedPosition( void ) const      { return fixed_position; }
    inline SGVec3d const& GetWgs84( void ) const    { return wgs84; }

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

    inline SGGeod const&  GetPosition( void ) const        { return position; }
    inline void    SetNormal( const SGVec3f& n )    { normal = n; }
    inline SGVec3f GetNormal( void ) const          { return normal; }

    void SaveToGzFile( gzFile& fp ) {
        sgWriteGeod( fp, position );
        sgWriteInt( fp, (int)fixed_position );

        // Don't save the facelist per node
        // it's much faster to just redo the lookup
    }

    void LoadFromGzFile( gzFile& fp ) {
        int temp;

        sgReadGeod( fp, position );
        CalcWgs84();

        sgReadInt( fp, &temp );
        fixed_position = (bool)temp;
    }

    // Friends for serialization
    friend std::ostream& operator<< ( std::ostream&, const TGNode& );
 
private:
    SGGeod      position;
    SGVec3f     normal;
    SGVec3d     wgs84;

    bool        fixed_position;

    TGFaceList  faces;
};

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4309)
#endif

class TGNodeIndex {
public:
    explicit TGNodeIndex( SGGeod g ) :
        geod(g)
    {
        std::size_t FNV_prime;
        std::size_t offset_basis;

        switch( sizeof( std::size_t ) ) {
            case 4: // 32 bit system
            default:
                FNV_prime =    16777619ULL;
                offset_basis = 2166136261ULL;
                break;

            case 8: // 64 bit system
                FNV_prime =    1099511628211ULL;
                offset_basis = 14695981039346656037ULL;
                break;
        }

        hash = (std::size_t)offset_basis;

        /* only hash lon, lat - we want to detect dups in 2d only */
        unsigned long long raw_pt[2];
        raw_pt[0] = (unsigned long long)( SGMisc<double>::round(geod.getLongitudeDeg() * PROXIMITY_MULTIPLIER) );
        raw_pt[1] = (unsigned long long)( SGMisc<double>::round(geod.getLatitudeDeg() * PROXIMITY_MULTIPLIER) );

        unsigned char* it = (unsigned char*)raw_pt;
        for ( unsigned i=0; i<sizeof( raw_pt ); i++ ) {
            hash = hash ^ *it++;
            hash = hash * FNV_prime;
        }
    }

    inline void         SetOrderedIndex( unsigned int i ) { ordered_index = i; }
    inline unsigned int GetOrderedIndex( void ) const     { return ordered_index; }
    inline std::size_t  GetHash( void ) const             { return hash; }

    friend bool operator == (const TGNodeIndex& a, const TGNodeIndex& b);
    friend std::ostream& operator<< ( std::ostream&, const TGNodeIndex& );

private:
    SGGeod       geod;
    std::size_t  hash;
    unsigned int ordered_index;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

inline bool operator == (const TGNodeIndex& a, const TGNodeIndex& b) {
    return (( fabs(a.geod.getLongitudeDeg() - b.geod.getLongitudeDeg()) < PROXIMITY_EPSILON ) &&
            ( fabs(a.geod.getLatitudeDeg()  - b.geod.getLatitudeDeg()) < PROXIMITY_EPSILON ));
}

struct TGNodeIndexHash : std::unary_function<TGNodeIndex, std::size_t>
{
    std::size_t operator()(TGNodeIndex const& gi) const {
        return gi.GetHash();
    }
};

typedef boost::unordered_set<TGNodeIndex, TGNodeIndexHash> unique_tgnode_set;
typedef unique_tgnode_set::iterator unique_tgnode_set_iterator;
typedef unique_tgnode_set::const_iterator const_unique_tgnode_set_iterator;

class UniqueTGNodeSet {
public:
    UniqueTGNodeSet() {}

    ~UniqueTGNodeSet() {
        index_list.clear();
        node_list.clear();
    }

    unsigned int add( const TGNode& n ) {
        unique_tgnode_set_iterator it;
        TGNodeIndex lookup( n.GetPosition() );

        it = index_list.find( lookup );
        if ( it == index_list.end() ) {
            lookup.SetOrderedIndex( node_list.size() );
            index_list.insert( lookup );

            node_list.push_back(n);
        } else {
            lookup = *it;
        }

        return lookup.GetOrderedIndex();
    }

    int find( const TGNode& n ) const {
        unique_tgnode_set_iterator it;
        TGNodeIndex lookup( n.GetPosition() );
        int index = -1;

        it = index_list.find( lookup );
        if ( it != index_list.end() ) {
            index = it->GetOrderedIndex();
        }

        return index;
    }

    void clear( void ) {
        index_list.clear();
        node_list.clear();
    }

    TGNode const& operator[]( int index ) const {
        return node_list[index];
    }

    TGNode& operator[]( int index ) {
        return node_list[index];
    }

    size_t size( void ) const {
        return node_list.size();
    }

    std::vector<TGNode>& get_list( void ) { return node_list; }

    void SaveToGzFile( gzFile& fp ) {
        // Just save the node_list - rebuild the index list on load
        sgWriteUInt( fp, node_list.size() );
        for (unsigned int i=0; i<node_list.size(); i++) {
            node_list[i].SaveToGzFile( fp );
        }
    }

    void LoadFromGzFile( gzFile& fp ) {
        unsigned int count;
        sgReadUInt( fp, &count );
        for (unsigned int i=0; i<count; i++) {
            TGNode node;
            node.LoadFromGzFile( fp );
            add( node );
        }
    }

private:
    unique_tgnode_set       index_list;
    std::vector<TGNode>     node_list;
};

#endif /* _TG_UNIQUE_TGNODE_HXX */