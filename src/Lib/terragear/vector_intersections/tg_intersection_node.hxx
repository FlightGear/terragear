#ifndef __TG_INTERSECTION_NODE_HXX__
#define __TG_INTERSECTION_NODE_HXX__

#include <stack>

#include "tg_intersection_edge.hxx"

// forward declarations
class tgIntersectionEdge;
class tgIntersectionNode;
class tgIntersectionNodeList;

#define NODE_UNTEXTURED (-1.0f)
#define NO_HEADING      (-1000.0f)

class tgIntersectionNode
{
public:
    tgIntersectionNode( const SGGeod& pos ); 
    tgIntersectionNode( const edgeArrPoint& pos );
    
    void AddEdge( bool originated, tgIntersectionEdge* edge );
    void DelEdge( bool originated, tgIntersectionEdge* edge );
    int  Degree( void ) const { return edgeList.size(); }
    bool IsCap( void ) const { return (edgeList.size() == 1); }
    
    void AddCapEdges( tgIntersectionNodeList& nodelist, tgintersectionedge_list& edgelist );
    void ConstrainEdges( void );
    void GenerateEdges( void );
    void CompleteSpecialIntersections( void );
    
    SGGeod GetPosition() const { return position; }
    edgeArrPoint GetPosition2() const { return position2; }
    
    void   SetStartV( double sv ) { start_v = sv; }
    double GetStartV( void ) const { return start_v; }
    
    tgIntersectionEdgeInfo* GetNextEdgeInfo( tgIntersectionEdge* cur_edge );
    
    tgIntersectionEdgeInfo* GetFirstEdgeInfo( void ) { return (*edgeList.begin()); }
    tgIntersectionEdgeInfo* GetPrevEdgeInfo( tgIntersectionEdgeInfo* cur_info, const tgConstraint& bisector, const edgeArrPoint& bisect_pos, const char* prefix );
    tgIntersectionEdgeInfo* GetNextEdgeInfo( tgIntersectionEdgeInfo* cur_info, const tgConstraint& bisector, const edgeArrPoint& bisect_pos, const char* prefix );
    double                  CalcDistanceToNextEndpoint( tgIntersectionEdgeInfo* cur_info, unsigned int& num_edges );
    void                    TextureToNextEndpoint( tgIntersectionEdgeInfo* cur_info, tgIntersectionGeneratorTexInfoCb texInfoCb, double ratio );
    void                    TextureEdges( tgIntersectionGeneratorTexInfoCb texInfoCb );
    void                    CheckEndpoint( void );
    bool                    IsEndpoint( void ) const { return endpoint; }
    
private:
    void GenerateBisectRays( void );
    void GeneratePrimaryBisectRays( double width, const tgintersectionedgeinfo_vector& edges );
    void GenerateSecondaryBisectRays( double width, const tgintersectionedgeinfo_vector& edges );

    void GenerateCapRays( void );
    void IntersectBisectRays( void );

    bool IntersectCurRightSideWithNextLeftSide( tgIntersectionEdgeInfo* cur_info, tgIntersectionEdgeInfo* nxt_info, edgeArrPoint& intersectionLocation );

    tgIntersectionEdgeInfo* FindPrevInfo( tgIntersectionEdgeInfo* cur_info );
    tgIntersectionEdgeInfo* FindNextInfo( tgIntersectionEdgeInfo* cur_info );
    
    //void Complete(tgIntersectionEdgeInfo* cur_info);
    void CompleteMultiSegmentIntersections(tgIntersectionEdgeInfo* cur_info, tgIntersectionEdgeInfo* nxt_info);
    void CompleteCap(tgIntersectionEdgeInfo* cur_info);
    
    SGGeod                      position;
    edgeArrPoint                position2;
    tgintersectionedgeinfo_set  edgeList;
    double                      start_v;
    bool                        endpoint;
    unsigned int                id;
};
typedef std::vector<tgIntersectionNode*> tgintersectionnode_list;

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

    tgIntersectionNode* Add( const SGGeod& loc ) {
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

    tgIntersectionNode* Add( const edgeArrPoint& loc ) {
        tgIntersectionNode* node = NULL;
        
        SGGeod gPos = SGGeod::fromDeg( CGAL::to_double(loc.x()), CGAL::to_double(loc.y()) );
        for ( unsigned int i=0; i<nodes.size(); i++ ) {
            if ( SGGeod_isEqual2D(nodes[i]->GetPosition(), gPos) ) {
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
    
    bool IsNode( const SGGeod& loc ) {
        bool isnode = false;
        
        for ( unsigned int i=0; i<nodes.size(); i++ ) {
            if ( SGGeod_isEqual2D(nodes[i]->GetPosition(), loc) ) {
                isnode = true;
                break;
            }
        }
        
        return isnode;
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

#endif /* __TG_INTERSECTION_NODE_HXX__ */