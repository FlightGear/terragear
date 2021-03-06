#ifndef __TG_INTERSECTION_GENERATOR_HXX__
#define __TG_INTERSECTION_GENERATOR_HXX__

#include "tg_intersection_node.hxx"
#include "tg_intersection_edge.hxx"
#include "tg_segmentnetwork.hxx"

// intersection generator and segment network flags
#define IG_DEBUG_COMPLETE       (0x01)

class tgIntersectionGenerator {
public:
    tgIntersectionGenerator(const char* dbg, unsigned int cln_f, unsigned int int_f, tgIntersectionGeneratorTexInfoCb cb) : segNet(cln_f, dbg), texInfoCb(cb), flags(int_f)  {
        strcpy(  debugDatabase, dbg );
    }
    
    void                                Insert( const SGGeod& s, const SGGeod& e, double w, int z, unsigned int t );
    void                                Execute( void );
    tgintersectionedge_it               edges_begin( void )  { return edgelist.begin(); }
    tgintersectionedge_it               edges_end( void )    { return edgelist.end(); }
    int                                 edges_size( void )   { return edgelist.size(); }
    
private:
    void                                ToShapefile( const char* prefix );

    tgSegmentNetwork                    segNet;
    tgIntersectionNodeList              nodelist;
    tgintersectionedge_list             edgelist;
        
    tgIntersectionGeneratorTexInfoCb    texInfoCb;
    char                                debugDatabase[256];
    unsigned int                        flags;    
};

#endif /* __TG_INTERSECTION_GENERATOR_HXX__ */