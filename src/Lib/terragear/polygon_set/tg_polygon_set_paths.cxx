#include "tg_polygon_set.hxx"

#define DEBUG_PATHS 0

tgPolygonSetPaths::tgPolygonSetPaths( const cgalPoly_Arrangement& arr )
{
    identifyFaces( arr  );
    dumpFaces();

    cgalPoly_FaceConstHandle outsideFaceH = arr.unbounded_face();
    
    // get the number of holes in the unbounded face
    cgalPoly_HoleConstIterator holeIt;
    unsigned int numHoles = 0;
    for (holeIt = outsideFaceH->holes_begin(); holeIt != outsideFaceH->holes_end(); holeIt++) {
        numHoles++;
    }
    
    if ( numHoles > 1 ) {
        // pretty sure this is impossible.  even a line segment is part of the hole...
        SG_LOG(SG_GENERAL, SG_ALERT, "tgPolygonSetPaths::tgPolygonSetPaths - unbounded face has more than one hole" );
        exit(0);
    }
    
    // add polys / holes from the single hole
    holeIt = outsideFaceH->holes_begin();
    
    // value of hole iterator is a ccb
    cgalPoly_CcbHeConstCirculator ccb = *holeIt; 
    cgalPoly_He                   he = *ccb;
    cgalPoly_FaceConstHandle      insideFaceH = (cgalPoly_FaceConstHandle)he.twin()->face();    

    if ( insideFaceH == outsideFaceH ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSetPaths::tgPolygonSetPaths - insideFH == ousideFH" );
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSetPaths::tgPolygonSetPaths - insideFH != ousideFH" );        
    }

    // create the primary path
    tgPolygonSetPath* path  = new tgPolygonSetPath( insideFaceH->outer_ccb(), false );
    
    paths.push_back( path );
};

tgPolygonSetPaths::~tgPolygonSetPaths()
{
    for( unsigned int i=0; i<paths.size(); i++ ) {
        delete paths[i];
    }
}

// when we hit a junction, we can add a child ( or children ) and continue
// with the current face.

// how?
// it's complicated...

// first a picture:
/******************************************************************************
 *                                                                            *
 *                                                                            *
 *   -----------------------------------------------------------------------  *
 *   |                                                                     |  * 
 *   |    -------------------------------------------------------          |  * 
 *   |    |                                                      \         |  * 
 *   |    |                                                       \        |  * 
 *   |    |                                                        \       |  * 
 *   |    |                                                         \      |  * 
 *   |    |                                                          \     |  * 
 *   |  J3|______________________                                     \    |  * 
 *   |    |\                    /                                      \   |  * 
 *   |    | \                  /                                        \  |  * 
 *   |    |  \                /                                          \ |  * 
 *   |    |   \              /                                          J1\|  * 
 *   |    |    \     F3     /                  H1                         /|  * 
 *   |    |     \          /                                             / |  * 
 *   |    |      \        /                                             /  |  * 
 *   |    |       \      /                                             /   |  * 
 *   |    |        \    /                                             /    |  * 
 *   |    |         \  /                                             /     |  * 
 *   |    |          \/_________________                            /      |  * 
 *   |    |         J4\                /                           /       |  * 
 *   |    |            \              /                           /        |  * 
 *   |    |             \            /                           /         |  * 
 *   |    |              \    F2    /                           /          |  *
 *   |    |     H2        \        /                           /           |  * 
 *   |    |                \      /                           /            |  * 
 *   |    |                 \    /                           /             |  * 
 *   |    |                  \  /                           /              |  * 
 *   |    |                   \/J2                         /               |  * 
 *   |    -------------------------------------------------                |  *
 *   |                                                                     |  *
 *   |                              F1                                     |  *
 *   ------start->----------------------------------------------------------  *
 *                                                                            *
 *                             unbounded face                                 *
 *                                                                            *
 ******************************************************************************/

/* The above diagram shows a rectangle polygon with various holes and 
 * filled area inside.  This is a valid shapefile containing a SINGLE polyline, 
 * and can be represented by a valid CGAL PolygonSet
 */

/* the algorith is as follows:
 *
 * 1) from start, create a new path with inner face F1, and outer face as the unbounded face
 * 2) traverse and add all points until hitting Junction J1
 * 3) two ( of four ) incident halfedges are valid:
 *    First at 12:00 - same inner and outer faces, so we'll continue with that one
 *    Second at 7:30 - same inner face, different outer face.  We'll create a new
 *                     path for that one with inner face F1 and outer face H1.
 *                     Because the inner face is not a hole, the outer face MUST be. 
 * 4) continue to traverse from J1 at 12:00 and reach the CCB end == start condition.  
 *    This path is complete.  Find an incomplete one - we just have the one at 7:30
 *    at J1.
 * 5) traverse new path to J2.
 * 6) Again, we have 2 ( of four ) valid incident halfedges.
 *    First at 9:00  - same inner base (F1 - we are traversing clockwise, now)
 *    Second at 1:30 - different inner face, same outer face
 * 7) pick 9:00, as it has more in common that 1:30.  add new path for 1:30 with
 *    outer face H1 and inner face F2 ( must be a face, because outer is a hole )
 * 8) continue to traverse from J2 at 9:00 to J3 
 * 9) We have 2 ( of four ) valid halfedges
 *    First at 12:00 - same inner face, different outer face ( same condition as J2 )
 *    Second at 4:30 - same outer face
 * 10) pick 12:00 ( same inner base is higher priority than same outer base )
 *     Add new path for 4:30 with outer face H2, and inner face F3
 * 11) continue to traverse from J3 at 12:00 and reach J1, which is the end condition
 *     for this path.  Mark it complete.  Find an incomplete 1 ( 1:30 from J2 )
 * 12) traverse 1:30 from J2 to J4.
 * 13) two valid halfedges 
 *     First at 4:30  - same inner and outer face
 *     Second at 1:30 - same outer face, different inner face
 * 14) pick 4:30 - both faces are the same.  Add new path at 1:30 with outer face H1, 
 *     and inner face F3.
 * 15) continue to traverse from J4 to J2, which is the end condition for this path.
 *     Mark it complete, find an incomplete path ( 1:30 from J4)
 * 16) traverse 1:30 from J4 to J3
 * 17) two valid halfedges
 *     First at 12:00  - same outer face, different inner face
 *     Second at 4:30 - same inner face, different outer face
 * 18) pick 4:30 - same inner face higher priority than same outer face.
 *     ( don't add 12:00 - it's already been traversed)
 * 19) traverse 4:30 from J3 to J4, which is the end condition. Mark complete
 * 20) shouldn't be any more incomplete paths.  We are done.
 * 
 */

/* Summary: 
 * Which halfedges are valid at a junction:
 * - Halfedge and halfedge twin are not in the added/completed list 
 *   ( updated whenever we are at a junction. )
 * - If there exists an edge with the inner AND outer faces
 *      then any other halfedges with the same inner face are valid
 *   else if there exists an edge with the same inner face but different outer face
 *      then any other halfedges with the same outer face are valid
 * 
 * Which halfedge should we continue on? 
 * PRIORITY list:
 * - any halfedge with same inner AND outer face
 * - any halfedge with same inner face
 * - anu halfedge with same outer face
 */

void tgPolygonSetPaths::traversePaths( void )
{
    tgPolygonSetPath*                   curPath = NULL;

    do {
        // find an unfinished path, and traverse until completed, or we hit a junction
        std::vector<tgPolygonSetPath*>::iterator pit = paths.begin();
        curPath = NULL;
        while ( pit != paths.end() ) {
            if ( !(*pit)->complete ) {
                curPath = *pit;
            }
            pit++;
        }
        
        if ( curPath ) {
            cgalPoly_CcbHeConstCirculator curCcb = curPath->startCcb;
            cgalPoly_HeConstHandle        curHe  = curCcb;

            // get inner and outer face IDs
            unsigned long innerId = lookupFace( curPath->inner.face );
            unsigned long outerId = lookupFace( curPath->outer.face );
            
            // some geometries can create duplicate paths - make sure this halfedge hasn't been visited.
            if ( isVisited( curHe ) ) {
                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths IGNORE/REMOVE duplicate path " << curPath->id << " at " << curHe->source()->point() << " inner: " << innerId << ", outer: " << outerId );
                curPath->nodes.clear();
                curPath->complete = true;
            } else {
#if DEBUG_PATHS                
                // open a datasource for the path debug
                char datasetname[128];
                sprintf(datasetname, "./Paths/path_%03lu", curPath->id );
                GDALDataset* poDS = tgPolygonSet::openDatasource(datasetname);
            
                // open Point layer
                OGRLayer* poNodeLayer = tgPolygonSet::openLayer(poDS, wkbPoint25D, tgPolygonSet::LF_DEBUG, "nodes");
            
                // save start node
                tgPolygonSet::toDebugShapefile( poNodeLayer, curHe->source()->point(), "start" );
#endif
                
                SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::traversePaths Start path " << curPath->id << " at " << curHe->source()->point() << " inner: " << innerId << ", outer: " << outerId );

                do
                {
                    // ignore inner antenna
                    if ( curHe->face() != curHe->twin()->face() ) { 
#if DEBUG_PATHS                
                        // add the node
                        char nodelabel[32];
                        sprintf( nodelabel, "node_%04lu", curPath->nodes.size() );
                        tgPolygonSet::toDebugShapefile( poNodeLayer, curHe->source()->point(), nodelabel );
#endif

                        setVisited( curHe );
                        curPath->nodes.push_back( curHe->source()->point() );
                    
                        // check for junction
                        // NOTE: if we contiue working on a path, it's always the case that we are at a junction.just pick the next route
 
                        if ( (curHe->target()->degree() > 2) ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::traversePaths Path " << curPath->id << " found a junction at " << curHe->target()->point() );
                        
                            // 1.  find all valid HEs
                            // ( same inner or outer face, and is NOT in visited list )
                            std::vector<tgPathsJunction> validPaths;
                        
                            // curculate this vertex, and add new paths for all HEs with it as their source ( except our curHE )
                            // the circulator returns hes with vertex as target - look at the twins...
                            cgalPoly_Arrangement::Halfedge_around_vertex_const_circulator vCurCirc = curHe->target()->incident_halfedges();
                            cgalPoly_Arrangement::Halfedge_around_vertex_const_circulator vEndCirc = vCurCirc;
                        
                            do {
                                // get the twin halfedge ( leaving the vertex )
                                cgalPoly_HeConstHandle curChildHe = vCurCirc->twin();
                                if ( !isVisited( curChildHe ) ) {
                                    unsigned long curInnerId = lookupFace( curHe->face() );
                                    unsigned long curOuterId = lookupFace( curHe->twin()->face() );
                                    unsigned long childInnerId = lookupFace( curChildHe->face() );
                                    unsigned long childOuterId = lookupFace( curChildHe->twin()->face() );
                                
                                    if ( curHe->face() == curChildHe->face() && curHe->twin()->face() == curChildHe->twin()->face() ) {
                                        // same inner and outer face
                                        validPaths.push_back( tgPathsJunction(curChildHe, tgPathsJunction::SAME_IN_AND_OUT) );

                                        SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has valid path to " << curChildHe->target()->point() << " with same inside and outside faces - in: " << curInnerId << " out: " << curOuterId );
                                    } else if ( curHe->face() == curChildHe->face() ) {
                                        // same inner face
                                        validPaths.push_back( tgPathsJunction(curChildHe, tgPathsJunction::SAME_IN) );

                                        SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has valid path to " << curChildHe->target()->point() << " with same inside face: " << curInnerId << " but different out - cur: " << curOuterId << ", path: " << childOuterId );
                                    } else if ( curHe->twin()->face() == curChildHe->twin()->face() ) {
                                        // same outer face
                                        validPaths.push_back( tgPathsJunction(curChildHe, tgPathsJunction::SAME_OUT) );
                                        
                                        SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has valid path to " << curChildHe->target()->point() << " with same outside face: " << curOuterId << " but different in - cur: " << curInnerId << ", path: " << childInnerId );
                                    }
                                } else {
                                    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has path to already visited halfedge to " << curChildHe->target()->point() );
                                }

                                vCurCirc++;
                            } while ( vCurCirc != vEndCirc );
                        
                            // get the highest priority path
                            // sort?
                        
                            tgPathsJunction::pathPriority_e highestPri = tgPathsJunction::UNKNOWN;
                            for ( unsigned int i=0; i<validPaths.size(); i++ ) {
                                if ( validPaths[i].priority > highestPri ) {
                                    highestPri = validPaths[i].priority;
                                }
                            }
                        
                            // traverse again and add paths to work queue
                            for ( unsigned int i=0; i<validPaths.size(); i++ ) {
                                if ( validPaths[i].priority < highestPri ) {
                                    // determine if this CCBs inner face is a hole or not...
                                    bool isHole = false;
                                
                                    paths.push_back( new tgPolygonSetPath( validPaths[i].startCcb, isHole) );
                                }
                            }
                        
                            // now continue past junction with highest priority
                            for ( unsigned int i=0; i<validPaths.size(); i++ ) {
                                if ( validPaths[i].priority == highestPri ) {
                                    curCcb = validPaths[i].startCcb;
                                    curHe = curCcb;

#if DEBUG_PATHS                
                                    char nodelabel[32];
                                    sprintf( nodelabel, "node_%04lu", curPath->nodes.size() );
                                    tgPolygonSet::toDebugShapefile( poNodeLayer, curHe->source()->point(), nodelabel );
#endif

                                    setVisited( curHe  );
                                    curPath->nodes.push_back( curHe->source()->point() );
                                    break;
                                }
                            }
                        }
                    } else {
                        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Path " << curPath->id << " - drop antenna node " );
                    }
                
                    ++curCcb;
                    curHe = curCcb;
                    if ( curHe->source() == curPath->endVertex ) {
                        curPath->complete = true;
                    }
                } while ( !curPath->complete );

#if DEBUG_PATHS                                
                GDALClose( poDS );
#endif
                
            }
        }
    } while( curPath != NULL );
}

void tgPolygonSetPaths::getPolys( std::vector<cgalPoly_Polygon>& boundaries, std::vector<cgalPoly_Polygon>& holes ) const
{
    std::vector<tgPolygonSetPath *>::const_iterator pit = paths.begin();
    while ( pit != paths.end() ) {
        if ( ((*pit)->complete) && ((*pit)->nodes.size()) ) {
            tgPolygonSetPath* curPath = *pit;
            
            if ( curPath->inner.isHole ) {
                holes.push_back( cgalPoly_Polygon( curPath->nodes.begin(), curPath->nodes.end() ) );
            } else {
                boundaries.push_back( cgalPoly_Polygon( curPath->nodes.begin(), curPath->nodes.end() ) );                
            }
        }
        pit++;
    }
}

void tgPolygonSetPaths::setVisited( cgalPoly_HeConstHandle he )
{
    visited.push_back( he );
}

bool tgPolygonSetPaths::isVisited( cgalPoly_HeConstHandle he )
{
    // check the visited array
    bool found = false;
    
    for ( unsigned int i=0; i<visited.size(); i++ ) {
        if ( he == visited[i] || he->twin() == visited[i] ) {
            found = true;
            break;
        }
    }
    
    return found;
}

void tgPolygonSetPaths::identifyFaces( const cgalPoly_Arrangement& arr )
{
    // traverse faces list, and add an identifier, so we can debug
    unsigned long faceId = 1;
    cgalPoly_FaceConstIterator fit;
    for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
        cgalPoly_FaceConstHandle h = (cgalPoly_FaceConstHandle)fit;
        if ( h->is_unbounded() ) {
            faces.push_back( tgPathsFaceID(h, 0) );            
        } else {
            faces.push_back( tgPathsFaceID(h, faceId++) );
        }
    }
}

void tgPolygonSetPaths::dumpFaces( void ) const
{
#if DEBUG_PATHS
    std::vector<tgPathsFaceID>::const_iterator fit;
    char faceId[32];

    for( fit = faces.begin(); fit != faces.end(); fit++ ) {
        sprintf( faceId, "face_%03lu", fit->id );
        printFace( faceId, fit->face );
    }
#endif
}

void tgPolygonSetPaths::printFace( const char* layer, cgalPoly_FaceConstHandle fh ) const
{    
    if( fh->has_outer_ccb() ) {
        // generate Polygon from face, and join wuth polygon set
        cgalPoly_CcbHeConstCirculator ccb = fh->outer_ccb();
        cgalPoly_CcbHeConstCirculator cur = ccb;
        cgalPoly_HeConstHandle        he;
        std::vector<cgalPoly_Point>   nodes;
        
        GDALDataset* poDS = tgPolygonSet::openDatasource("Faces");
        
        char layer_node_name[64];
        sprintf( layer_node_name, "%s_badnodes", layer );
        
        char layer_edge_name[64];
        sprintf( layer_edge_name, "%s_edges", layer );
        
        OGRLayer*    poLayerNodes = tgPolygonSet::openLayer(poDS, wkbPoint25D, tgPolygonSet::LF_DEBUG, layer_node_name);
        OGRLayer*    poLayerEdges = tgPolygonSet::openLayer(poDS, wkbLineString25D, tgPolygonSet::LF_DEBUG, layer_edge_name);
        
        do
        {
            he = cur;
            
            // ignore inner antenna
            if ( he->face() != he->twin()->face() ) { 
                if ( he->source()->degree() > 2 ) {
                    // dump the bad nodes to a shapefile
                    tgPolygonSet::toDebugShapefile( poLayerNodes, he->source()->point(), "node" );
                }
                nodes.push_back( he->source()->point() );
            } else {
                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::printFace - drop antenna node  " );                            
            }
            
            ++cur;
        } while (cur != ccb);
        
        cgalPoly_Polygon poly( nodes.begin(), nodes.end()  );
        tgPolygonSet::toDebugShapefile( poLayerEdges, poly, "desc" );
        
        GDALClose( poDS );
        
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::printFace - face has no outer ccb " );
    }    
}

unsigned long tgPolygonSetPaths::lookupFace( cgalPoly_FaceConstHandle h ) const
{
    std::vector<tgPathsFaceID>::const_iterator fit;
    unsigned long id = 0;
    
    for( fit = faces.begin(); fit != faces.end(); fit++ ) {
        if ( h == fit->face ) {
            id = fit->id;
        }
    }
    
    return id;
}

unsigned long tgPolygonSetPath::cur_id = 1;
tgPolygonSetPath::tgPolygonSetPath( cgalPoly_CcbHeConstCirculator sccb, bool isHole ) : id(tgPolygonSetPath::cur_id++)
{
    startCcb = sccb;
    cgalPoly_HeConstHandle tmp = startCcb; 
    
    endVertex  = tmp->source();

    inner.face = tmp->face();
    inner.isHole = isHole;
    
    outer.face = tmp->twin()->face();
    outer.isHole = !isHole;
    
    complete = false;
}

void tgPolygonSetPath::toShapefile( const char* ds )
{
#if 0    
    char datasetName[128];
    sprintf( datasetName, "%s/path_%04lu", ds, id );
    GDALDataset* poDS = openDatasource(datasetName);
    
    // open three layers per path.  
    // a point layer for nodes, and a point layer for junctions
    // and a line layer for the path
    OGRLayer*    poLayerNodes      = openLayer(poDS, wkbPoint25D, "nodes");
    OGRLayer*    poLayerJunctions  = openLayer(poDS, wkbPoint25D, "junctions");
    OGRLayer*    poLayerPath       = openLayer(poDS, wkbLineString25D, "path");
    
    // save Nodes
    
    // save Junctions
    
    // save Path

    
    GDALClose(poDS);
#endif    
}