#include <boost/unordered_set.hpp>

// Implement Unique SGVec3d list

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

class SGVec3dIndex {
public:
    SGVec3dIndex( SGVec3d v ) {
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
        unsigned long long raw_pt[3];
        raw_pt[0] = (unsigned long long)( round(vec.x() * PROXIMITY_MULTIPLIER) );
        raw_pt[1] = (unsigned long long)( round(vec.y() * PROXIMITY_MULTIPLIER) );
        raw_pt[2] = (unsigned long long)( round(vec.z() * PROXIMITY_MULTIPLIER) );

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

    friend bool operator == (const SGVec3dIndex& a, const SGVec3dIndex& b);
    friend std::ostream& operator<< ( std::ostream&, const SGVec3dIndex& );

private:
    SGVec3d      vec;
    std::size_t  hash;
    unsigned int ordered_index;
};

inline bool operator == (const SGVec3dIndex& a, const SGVec3dIndex& b) {
    return (( fabs(a.vec.x() - b.vec.x()) < PROXIMITY_EPSILON ) &&
            ( fabs(a.vec.y() - b.vec.y()) < PROXIMITY_EPSILON ) &&
            ( fabs(a.vec.z() - b.vec.z()) < PROXIMITY_EPSILON ));
}

struct SGVec3dIndexHash : std::unary_function<SGVec3dIndex, std::size_t>
{
    std::size_t operator()(SGVec3dIndex const& vi) const {
        return vi.GetHash();
    }
};

typedef boost::unordered_set<SGVec3dIndex, SGVec3dIndexHash> unique_vec3d_set;
typedef unique_vec3d_set::iterator unique_vec3d_set_iterator;
typedef unique_vec3d_set::const_iterator const_unique_vec3d_set_iterator;

class UniqueSGVec3dSet {
public:
    UniqueSGVec3dSet() {}

    unsigned int add( const SGVec3d& v );
    int          find( const SGVec3d& v ) const;
    std::vector<SGVec3d>& get_list( void ) { return vector_list; }

private:
    unique_vec3d_set        index_list;
    std::vector<SGVec3d>    vector_list;
};

unsigned int UniqueSGVec3dSet::add( const SGVec3d& v ) {
    unique_vec3d_set_iterator it;
    SGVec3dIndex lookup( v );

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


int UniqueSGVec3dSet::find( const SGVec3d& v ) const {
    unique_vec3d_set_iterator it;
    SGVec3dIndex lookup( v );
    int index = -1;

    it = index_list.find( lookup );
    if ( it != index_list.end() ) {
        index = it->GetOrderedIndex();
    }

    return index;
}