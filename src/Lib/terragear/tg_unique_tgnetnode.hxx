#ifndef _TG_UNIQUE_TGNETNODE_HXX
#define _TG_UNIQUE_TGNETNODE_HXX

#include <utility>
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

struct TGNetEdge {
    std::string     material;
    unsigned int    width;
    SGGeod          start_pos;
    SGGeod          end_pos;
};
typedef std::vector<TGNetEdge> TGNetEdgeList;

// add to a pair so we know if we (as a node) originate or terminate this edge
typedef std::pair<bool, TGNetEdge> TGDirectedNetEdge;
typedef std::vector<TGDirectedNetEdge> TGDirectedNetEdgeList;

class TGNetNode {
public:
    TGNetNode() {
    }

    TGNetNode( const SGGeod& p ) {
        position  = p;
        directed_edges.clear();
    }
    
    inline void AddEdgeOriginating( const TGNetEdge& edge, const SGGeod& terminating )
    {
        for ( unsigned int i=0; i<directed_edges.size(); i++ ) {
            // calculate the heading from this node
            double heading;
            if ( directed_edges[i].first ) {
                heading = SGGeodesy::courseDeg( position, directed_edges[i].second.end_pos );
            } else {
                heading = SGGeodesy::courseDeg( position, directed_edges[i].second.start_pos );
            }
        }
        // insert sorted by heading from our position
        directed_edges.push_back( std::make_pair(true, edge) );
    }

    inline TGDirectedNetEdgeList const& GetEdges( void ) const { return directed_edges; }
    inline SGGeod const& GetPosition( void )     const { return position; }

    // Friends for serialization
    friend std::ostream& operator<< ( std::ostream&, const TGNetNode& );
 
private:
    SGGeod                  position;
    TGDirectedNetEdgeList   directed_edges;    
};

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4309)
#endif

class TGNetNodeIndex {
public:
    TGNetNodeIndex( SGGeod g ) {
        geod = g;

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

    friend bool operator == (const TGNetNodeIndex& a, const TGNetNodeIndex& b);
    friend std::ostream& operator<< ( std::ostream&, const TGNetNodeIndex& );

private:
    SGGeod       geod;
    std::size_t  hash;
    unsigned int ordered_index;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

inline bool operator == (const TGNetNodeIndex& a, const TGNetNodeIndex& b) {
    return (( fabs(a.geod.getLongitudeDeg() - b.geod.getLongitudeDeg()) < PROXIMITY_EPSILON ) &&
            ( fabs(a.geod.getLatitudeDeg()  - b.geod.getLatitudeDeg()) < PROXIMITY_EPSILON ));
}

struct TGNetNodeIndexHash : std::unary_function<TGNetNodeIndex, std::size_t>
{
    std::size_t operator()(TGNetNodeIndex const& gi) const {
        return gi.GetHash();
    }
};

typedef boost::unordered_set<TGNetNodeIndex, TGNetNodeIndexHash> unique_tgnetnode_set;
typedef unique_tgnetnode_set::iterator unique_tgnetnode_set_iterator;
typedef unique_tgnetnode_set::const_iterator const_unique_tgnetnode_set_iterator;

class UniqueTGNetNodeSet {
public:
    UniqueTGNetNodeSet() {}

    ~UniqueTGNetNodeSet() {
        index_list.clear();
        node_list.clear();
    }

    unsigned int add( const TGNetNode& n ) {
        unique_tgnetnode_set_iterator it;
        TGNetNodeIndex lookup( n.GetPosition() );

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

    int find( const TGNetNode& n ) const {
        unique_tgnetnode_set_iterator it;
        TGNetNodeIndex lookup( n.GetPosition() );
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

    TGNetNode const& operator[]( int index ) const {
        return node_list[index];
    }

    TGNetNode& operator[]( int index ) {
        return node_list[index];
    }

    size_t size( void ) const {
        return node_list.size();
    }

    std::vector<TGNetNode>& get_list( void ) { return node_list; }

private:
    unique_tgnetnode_set    index_list;
    std::vector<TGNetNode>  node_list;
};

#endif /* _TG_UNIQUE_TGNETNODE_HXX */