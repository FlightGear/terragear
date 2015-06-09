#ifndef __TG_INTERSECTION_GENERATOR_HXX__
#define __TG_INTERSECTION_GENERATOR_HXX__

#include "tg_intersection_node.hxx"
#include "tg_intersection_edge.hxx"
#include "tg_segmentnetwork.hxx"

#define IG_DEBUG_COMPLETE       (0x01)

class tgIntersectionGenerator {
public:
    tgIntersectionGenerator(const char* dr, unsigned int f, tgIntersectionGeneratorTexInfoCb cb) : segNet(dr), texInfoCb(cb), flags(f)  {
        strcpy(  debugRoot, dr );
        sprintf( datasource, "./edge_dbg/%s", debugRoot );
    }
    
    void                                Insert( const SGGeod& s, const SGGeod& e, double w, unsigned int t );
    void                                Execute( bool clean );
    tgintersectionedge_it               edges_begin( void )  { return edgelist.begin(); }
    tgintersectionedge_it               edges_end( void )    { return edgelist.end(); }
    int                                 edges_size( void )   { return edgelist.size(); }
    
private:
    void                                ToShapefile( const char* prefix );

    tgSegmentNetwork                    segNet;
    tgIntersectionNodeList              nodelist;
    tgintersectionedge_list             edgelist;
        
    tgIntersectionGeneratorTexInfoCb    texInfoCb;
    char                                debugRoot[64];
    char                                datasource[64];
    unsigned int                        flags;
    
};

#endif /* __TG_INTERSECTION_GENERATOR_HXX__ */