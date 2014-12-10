#ifndef __TG_INTERSECTION_NODE_HXX__
#define __TG_INTERSECTION_NODE_HXX__

#include "tg_intersection_edge.hxx"

// forward declarations
class tgIntersectionEdge;
class tgIntersectionNode;
class tgIntersectionNodeList;

class tgIntersectionNode
{
public:
    tgIntersectionNode( const SGGeod& pos ); 
    
    void AddEdge( bool originated, tgIntersectionEdge* edge );
    void DelEdge( bool originated, tgIntersectionEdge* edge );
    
    void AddCapEdges( tgIntersectionNodeList& nodelist, tgintersectionedge_list& edgelist );
    void ConstrainEdges( void );
    void GenerateEdges( void );
    void CompleteSpecialIntersections( void );
    
    SGGeod GetPosition() const { return position; }

    tgIntersectionEdgeInfo* GetPrevEdgeInfo( tgIntersectionEdgeInfo* cur_info, const tgRay& bisector, const SGGeod& bisect_pos, const char* prefix );
    tgIntersectionEdgeInfo* GetNextEdgeInfo( tgIntersectionEdgeInfo* cur_info, const tgRay& bisector, const SGGeod& bisect_pos, const char* prefix );
    
private:
    void GenerateBisectRays( void );
    void GenerateCapRays( void );
    void IntersectBisectRays( void );
    //void Complete(tgIntersectionEdgeInfo* cur_info);
    void CompleteMultiSegmentIntersections(tgIntersectionEdgeInfo* cur_info, tgIntersectionEdgeInfo* nxt_info);
    void CompleteCap(tgIntersectionEdgeInfo* cur_info);
    
    SGGeod                      position;
    tgintersectionedgeinfo_list edgeList;
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