#include <boost/unordered_set.hpp>

// Implement Unique SGVec2f list

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

class SGVec2fIndex {
public:
    SGVec2fIndex( SGVec2f v ) {
        vec = v;

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
        raw_pt[0] = (unsigned long long)( round(vec.x() * PROXIMITY_MULTIPLIER) );
        raw_pt[1] = (unsigned long long)( round(vec.y() * PROXIMITY_MULTIPLIER) );

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

    friend bool operator == (const SGVec2fIndex& a, const SGVec2fIndex& b);
    friend std::ostream& operator<< ( std::ostream&, const SGVec2fIndex& );

private:
    SGVec2f      vec;
    std::size_t  hash;
    unsigned int ordered_index;
};

inline bool operator == (const SGVec2fIndex& a, const SGVec2fIndex& b) {
    return (( fabs(a.vec.x() - b.vec.x()) < PROXIMITY_EPSILON ) &&
            ( fabs(a.vec.y() - b.vec.y()) < PROXIMITY_EPSILON ));
}

struct SGVec2fIndexHash : std::unary_function<SGVec2fIndex, std::size_t>
{
    std::size_t operator()(SGVec2fIndex const& vi) const {
        return vi.GetHash();
    }
};

typedef boost::unordered_set<SGVec2fIndex, SGVec2fIndexHash> unique_vec2f_set;
typedef unique_vec2f_set::iterator unique_vec2f_set_iterator;
typedef unique_vec2f_set::const_iterator const_unique_vec2f_set_iterator;

class UniqueSGVec2fSet {
public:
    UniqueSGVec2fSet() {}

    unsigned int add( const SGVec2f& v );
    int          find( const SGVec2f& v ) const;
    std::vector<SGVec2f>& get_list( void ) { return vector_list; }

private:
    unique_vec2f_set        index_list;
    std::vector<SGVec2f>    vector_list;
};

unsigned int UniqueSGVec2fSet::add( const SGVec2f& v ) {
    unique_vec2f_set_iterator it;
    SGVec2fIndex lookup( v );

    it = index_list.find( lookup );
    if ( it == index_list.end() ) {
        lookup.SetOrderedIndex( vector_list.size() );
        index_list.insert( lookup );

        vector_list.push_back(v);
    } else {
        lookup = *it;
    }

    return lookup.GetOrderedIndex();
}


int UniqueSGVec2fSet::find( const SGVec2f& v ) const {
    unique_vec2f_set_iterator it;
    SGVec2fIndex lookup( v );
    int index = -1;

    it = index_list.find( lookup );
    if ( it != index_list.end() ) {
        index = it->GetOrderedIndex();
    }

    return index;
}