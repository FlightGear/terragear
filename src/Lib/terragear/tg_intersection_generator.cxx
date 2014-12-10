#include "tg_segmentnetwork.hxx"
#include "tg_polygon.hxx"

#include "tg_intersection_generator.hxx"

#define DEBUG_INTERSECTIONS (0)
#define LOG_INTERSECTION    (SG_DEBUG)

void tgIntersectionGenerator::Insert( const SGGeod& s, const SGGeod& e, double w, unsigned int t )
{    
    segNet.Add( s, e, w, t );
}

void tgIntersectionGenerator::Execute( void )
{
    // Segnet has all of the edges and nodes in an arrangement : clean it
    SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Clean network " );
    segNet.Clean();
        
    // now retreive the cleaned edges 
    for ( segnetedge_it snit = segNet.output_begin(); snit != segNet.output_end(); snit++ ) {
        // get the start and end nodes
        tgIntersectionNode* start = nodelist.Add( snit->start );
        tgIntersectionNode* end   = nodelist.Add( snit->end );

        if ( start != end ) {
            tgIntersectionEdge* newEdge = new tgIntersectionEdge( start, end, snit->width, snit->type, debugRoot );
            edgelist.push_back( newEdge );
        }
    }

    // add end cap segments
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: ConstrainEdges at node " << i << " of " << nodelist.size() );
        nodelist[i]->AddCapEdges( nodelist, edgelist );
    }
    
    //find shared edge constraints at each node
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: ConstrainEdges at node " << i << " of " << nodelist.size() );
        nodelist[i]->ConstrainEdges();
    }

    // Generate the edge from each node
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: GenerateEdges at node " << i << " of " << nodelist.size() );
        nodelist[i]->GenerateEdges();
    }

    // fix multisegment intersections - happens when segments do not diverge from each other before the end of the edge
    for (unsigned int i=0; i<nodelist.size(); i++) {
        SG_LOG(SG_GENERAL, LOG_INTERSECTION, "tgIntersectionGenerator::Execute: FixSpecialIntersections at node " << i << " of " << nodelist.size() );
        nodelist[i]->CompleteSpecialIntersections();
    }
    
    // complete the edges
    for (tgintersectionedge_it it = edgelist.begin(); it != edgelist.end(); it++) {
        (*it)->Complete();
    }
}