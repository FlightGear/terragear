#ifndef __TG_INTERSECTION_HXX__
#define __TG_INTERSECTION_HXX__

#include <set>

#include <simgear/debug/logstream.hxx>
#include <simgear/math/SGMath.hxx>

#include "tg_euclidean.hxx"
#include "tg_misc.hxx"

class tgIntersectionNode;
class tgIntersectionEdge;
class tgIntersectionEdgeInfo;

typedef std::list<tgIntersectionEdge*> tgintersectionedge_list;
typedef tgintersectionedge_list::iterator tgintersectionedge_it;

class tgIntersectionEdgeInfo
{
public:
    tgIntersectionEdgeInfo( bool orig, tgIntersectionEdge* e );
    
    bool operator< (const tgIntersectionEdgeInfo* e) const;
    
    const tgIntersectionEdge* GetEdge( void ) const {
        return edge;
    }
    tgIntersectionEdge* GetEdge( void ) {
        return edge;
    }
    
    double GetHeading(void) const {
        return heading;
    }
    
    bool IsOriginating(void) const {
        return originating; 
    }
    
    void SetMultiSegment( bool ms ) {
        multiSegment = ms;
    }

    bool IsMultiSegment( void ) const {
        return multiSegment;
    }
    
private:
    tgIntersectionEdge*     edge;
    bool                    originating;    // edge originates at this node
    double                  heading;        // edge heading from this node 
    bool                    multiSegment;   // edge intersection needs to traverse past single segment
};

struct EdgeInfoPtrComp
{
    bool operator()(const tgIntersectionEdgeInfo* lhs, const tgIntersectionEdgeInfo* rhs) const  { 
        bool result = true;
        if( lhs->GetHeading() >= rhs->GetHeading() ) {
            result = false;
        }
        return result;
    }
};

typedef std::set<tgIntersectionEdgeInfo *, EdgeInfoPtrComp> tgintersectionedgeinfo_list;
typedef tgintersectionedgeinfo_list::iterator tgintersectionedgeinfo_it;

class tgIntersectionNode
{
public:
    tgIntersectionNode( const SGGeod& pos ); 
    
    void AddEdge( bool originated, tgIntersectionEdge* edge );  
    
    void ConstrainEdges( void );
    void GenerateEdges( void );
    void FixMultisegmentIntersections( void );
    
    SGGeod GetPosition() const { return position; }

    tgIntersectionEdgeInfo* GetPrevEdgeInfo( tgIntersectionEdgeInfo* cur_info );
    tgIntersectionEdgeInfo* GetNextEdgeInfo( tgIntersectionEdgeInfo* cur_info );
    
private:
    void GenerateBisectRays( void );
    void IntersectBisectRays( void );
    void CompleteIntersection(tgIntersectionEdgeInfo* cur_info, tgIntersectionEdgeInfo* nxt_info);
    
    SGGeod                      position;
    tgintersectionedgeinfo_list edgeList;
};
typedef std::vector<tgIntersectionNode*> tgintersectionnode_list;

class tgIntersectionEdge
{
public:
    tgIntersectionEdge( tgIntersectionNode* s, tgIntersectionNode* e, double w, unsigned int t );
    
    bool operator==(const tgIntersectionEdge& e) { 
        return ((start == e.start) && (end == e.end)); 
    }

    void AddBottomRightConstraint( const tgRay& c ) {
        constrain_br.push_back( c );
    }
    void AddBottomLeftConstraint( const tgRay& c ) {
        constrain_bl.push_back( c );
    }
    void AddTopLeftConstraint( const tgRay& c ) {
        constrain_tl.push_back( c );
    }
    void AddTopRightConstraint( const tgRay& c ) {
        constrain_tr.push_back( c );
    }
    void AddLeftSegmentConstraint( const SGGeod& pt, bool originating, bool complete );
    void AddRightSegmentConstraint( const SGGeod& pt, bool originating, bool complete );
    
    void AddLeftContour( const SGGeod& pt, bool originating );
    void AddRightContour( const SGGeod& pt, bool originating );
    
    double GetHeading( bool originating ) const {
        if ( originating ) {
            return TGEuclidean::courseDeg( start->GetPosition(), end->GetPosition() );
        } else {
            return TGEuclidean::courseDeg( end->GetPosition(), start->GetPosition() );
        }
    }
    
    tgRay GetTopRightConstraint( bool originating ) const {
        if ( originating ) {
            return constrain_tr[0];
        } else {
            return constrain_bl[0];
        }
    }
    
    tgRay GetTopLeftConstraint( bool originating ) const {
        if ( originating ) {
            return constrain_tl[0];
        } else {
            return constrain_br[0];
        }
    }
    
    tgLine GetRightSide( bool originating ) const {
        if ( originating ) {
            return side_r;
        } else {
            return side_l; 
        }
    }

    tgLine GetLeftSide( bool originating ) const {
        if ( originating ) {
            return side_l;
        } else {
            return side_r; 
        }
    }

    void ToShapefile( void ) const;
        
    bool IntersectConstraintsAndSides( bool originating );
    tgPolygon CreatePolygon(int& type, double& heading, double& dist, double& width, SGGeod& texref, const char* dbg_layer);
    tgPolygon GetPoly(const char* prefix);
    
    tgIntersectionNode* start;
    tgIntersectionNode* end;
    double              width;
    unsigned int        type;
    
    SGGeod              botLeft, botRight;
    SGGeod              topLeft, topRight;
    
    tgLine              side_l;
    tgLine              side_r;
        
    tgray_list          constrain_br;
    tgray_list          constrain_bl;
    tgray_list          constrain_tl;
    tgray_list          constrain_tr;
    
    std::list<SGGeod>   constrain_msl;
    std::list<SGGeod>   constrain_msr;

    std::list<SGGeod>   left_contour;
    std::list<SGGeod>   right_contour;
    
    unsigned long int   id;
    
private:
    SGGeod IntersectCorner( const SGGeod& pos, tgray_list& constraint1, tgray_list& constraint2, 
                            tgLine& side, bool& constraint_nearest, 
                            const char* c1name, const char* c2name, const char* sname );
    std::list<SGGeod> ClipContour(std::list<SGGeod>&contour, std::list<SGGeod>& constraint);
    
    char                datasource[64];
};

class tgIntersectionNodeList {
public:
    tgIntersectionNodeList() {
        nodes.clear();
    }
    
    tgIntersectionNode* Get( const SGGeod& loc ) {
        tgIntersectionNode* node = NULL;
        
        for ( unsigned int i=0; i<nodes.size(); i++ ) {
            if ( SGGeod_isEqual2D(nodes[i]->GetPosition(), loc) ) {
                node = nodes[i];
                break;
            }
        }
        
        if ( node == NULL ) {
            node = new tgIntersectionNode( loc );
            nodes.push_back( node );
        }
        
        return node;
    }
    
    unsigned int size(void) const {
        return nodes.size();
    }
        
    tgIntersectionNode* operator[]( int index ) {
        return nodes[index];
    }
    
private:
    tgintersectionnode_list    nodes;    
};

#endif /* __TG_INTERSECTION_HXX__ */