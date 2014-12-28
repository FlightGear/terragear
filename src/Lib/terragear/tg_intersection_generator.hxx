#ifndef __TG_INTERSECTION_GENERATOR_HXX__
#define __TG_INTERSECTION_GENERATOR_HXX__

#include "tg_intersection_node.hxx"
#include "tg_intersection_edge.hxx"
#include "tg_segmentnetwork.hxx"

class tgIntersectionGenerator {
public:
    tgIntersectionGenerator(const std::string& dr, tgIntersectionGeneratorTexInfoCb cb) : segNet(dr), texInfoCb(cb), debugRoot(dr)  {
        sprintf( datasource, "./edge_dbg/%s", debugRoot.c_str() );
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
    std::string                         debugRoot;
    
    char                                datasource[64];
    
};

#endif /* __TG_INTERSECTION_GENERATOR_HXX__ */