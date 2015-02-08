#ifndef _TGCONTOUR_HXX
#define _TGCONTOUR_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>
#include <boost/concept_check.hpp>

#include "tg_unique_geod.hxx"
#include "tg_rectangle.hxx"
#include "clipper.hpp"

/* forward declarations */
class TGNode;

class tgPolygon;
typedef std::vector <tgPolygon>  tgpolygon_list;

class tgContour;
typedef std::vector <tgContour>  tgcontour_list;
typedef tgcontour_list::iterator tgcontour_list_iterator;
typedef tgcontour_list::const_iterator const_tgcontour_list_iterator;

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
    void DelNode( unsigned int i ) {
        node_list.erase( node_list.begin()+i);
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

    tgRectangle GetBoundingBox( void ) const;

    double GetMinimumAngle( void ) const;
    double GetArea( void ) const;

    bool   operator==(const tgContour& other ) {
        bool isEqual = true;

        if ( GetSize() == other.GetSize() )
        {
            for (unsigned int i=0; i<GetSize(); i++) {
                if ( GetNode(i) != other.GetNode(i) ) {
                    isEqual = false;
                    break;
                }
            }
        } else {
            isEqual = false;
        }

        return isEqual; 
    }


    static tgContour Snap( const tgContour& subject, double snap );
    static tgContour RemoveDups( const tgContour& subject );
    static tgContour SplitLongEdges( const tgContour& subject, double dist );
    static tgContour RemoveSpikes( const tgContour& subject );
    static bool      RemoveCycles( const tgContour& subject, tgcontour_list& result );

    static tgPolygon Union( const tgContour& subject, tgPolygon& clip );
    static tgPolygon Diff( const tgContour& subject, tgPolygon& clip );
    static tgPolygon Intersect( const tgContour& subject, const tgContour& clip );

    static bool      IsInside( const tgContour& inside, const tgContour& outside );
    static tgContour AddColinearNodes( const tgContour& subject, UniqueSGGeodSet& nodes );
    static tgContour AddColinearNodes( const tgContour& subject, std::vector<SGGeod>& nodes );
    static tgContour AddColinearNodes( const tgContour& subject, bool preserve3d, std::vector<TGNode*>& nodes );
    static bool      FindColinearLine( const tgContour& subject, const SGGeod& node, SGGeod& start, SGGeod& end );

    // conversions
    static ClipperLib::Path ToClipper( const tgContour& subject );
    static tgContour FromClipper( const ClipperLib::Path& subject );

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

#endif // _TGCONTOUR_HXX