#include <boost/unordered_set.hpp>
#include <simgear/math/SGMisc.hxx>

// Implement Unique SGGeod list

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

class SGGeodIndex {
public:
    SGGeodIndex( SGGeod g ) {
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
        raw_pt[0] = (unsigned long long)( SGMisc::round(geod.getLongitudeDeg() * PROXIMITY_MULTIPLIER) );
        raw_pt[1] = (unsigned long long)( SGMisc::round(geod.getLatitudeDeg() * PROXIMITY_MULTIPLIER) );

        unsigned char* it = (unsigned char*)raw_pt;
        for ( unsigned i=0; i<sizeof( raw_pt ); i++ ) {
            hash = hash ^ *it++;
            hash = hash * FNV_prime;
        }

        // SG_LOG(SG_GENERAL, SG_ALERT, " GetHash x: " << raw_pt[0] << " y: " << raw_pt[1] << " = " << hash );
    }

    inline void         SetOrderedIndex( unsigned int i ) { ordered_index = i; }
    inline unsigned int GetOrderedIndex( void ) const     { return ordered_index; }
    inline std::size_t  GetHash( void ) const             { return hash; }

    friend bool operator == (const SGGeodIndex& a, const SGGeodIndex& b);
    friend std::ostream& operator<< ( std::ostream&, const SGGeodIndex& );

private:
    SGGeod       geod;
    std::size_t  hash;
    unsigned int ordered_index;
};

inline bool operator == (const SGGeodIndex& a, const SGGeodIndex& b) {
    return (( fabs(a.geod.getLongitudeDeg() - b.geod.getLongitudeDeg()) < PROXIMITY_EPSILON ) &&
            ( fabs(a.geod.getLatitudeDeg()  - b.geod.getLatitudeDeg()) < PROXIMITY_EPSILON ));
}

struct SGGeodIndexHash : std::unary_function<SGGeodIndex, std::size_t>
{
    std::size_t operator()(SGGeodIndex const& gi) const {
        return gi.GetHash();
    }
};

typedef boost::unordered_set<SGGeodIndex, SGGeodIndexHash> unique_geod_set;
typedef unique_geod_set::iterator unique_geod_set_iterator;
typedef unique_geod_set::const_iterator const_unique_geod_set_iterator;

class UniqueSGGeodSet {
public:
    UniqueSGGeodSet() {}

    unsigned int add( const SGGeod& g ) {
        unique_geod_set_iterator it;
        SGGeodIndex lookup( g );

        it = index_list.find( lookup );
        if ( it == index_list.end() ) {
            lookup.SetOrderedIndex( geod_list.size() );
            index_list.insert( lookup );

            geod_list.push_back(g);
        } else {
            lookup = *it;
        }

        return lookup.GetOrderedIndex();
    }

    int find( const SGGeod& g ) const {
        unique_geod_set_iterator it;
        SGGeodIndex lookup( g );
        int index = -1;

        it = index_list.find( lookup );
        if ( it != index_list.end() ) {
            index = it->GetOrderedIndex();
        }

        return index;
    }

    std::vector<SGGeod>& get_list( void ) { return geod_list; }
    void get_node_list();

private:
    unique_geod_set         index_list;
    std::vector<SGGeod>     geod_list;
};
