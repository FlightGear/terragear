#include <algorithm>
#include <functional>

#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

#define THRESHOLD_SAME      (0.0000000000001)
#define THRESHOLD_TOO_CLOSE (0.00000001)

#define DEBUG_SHARED_EDGE   (0)

/* This will add or move nodes to match a neighbor edge.  Algorithm is designed to give the same result for both tiles.  
 * (it is run twice - once for each tile on the shared edge )
 */
void tgMeshTriangulation::matchNodes( edgeType edge, std::vector<meshVertexInfo>& curVertexes, std::vector<meshVertexInfo>& neighVertexes, std::vector<meshTriPoint>& addedNodes, std::vector<movedNode>& movedNodes )
{
    // we'll build a search tree of all nodes on the shared edge - flag which ones are current, which are neighbor, and which are both.
    // to determine if they are on both, we'll create 2 kd-trees to search first.
    // CGAL kd trees cannot remove items, so merging in tree is innefficient.
    nodeMembershipTree  curTree, neighTree;
    std::vector<meshVertexInfo>::iterator viIt;

    for ( viIt = curVertexes.begin(); viIt != curVertexes.end(); viIt++ ) {
        curTree.insert( nodeMembershipData( viIt->getPoint(), NODE_CURRENT, viIt->getId() ) );        
    }

    for ( viIt = neighVertexes.begin(); viIt != neighVertexes.end(); viIt++ ) {
        neighTree.insert( nodeMembershipData( viIt->getPoint(), NODE_NEIGHBOR, -1 ) );        
    }

    // does this need to be a tree as well?
    nodeMembershipTree  nodeTree;

    const char *edgestr[4] = {
        "north",
        "south",
        "east",
        "west"
    };

    SG_LOG(SG_GENERAL, SG_DEBUG, "edge matching bucket " << mesh->getBucket().gen_index_str() << " edge " << edgestr[edge] << ".  current - " << curTree.size() << ", neighbor - " << neighTree.size() );           

    // traverse neighbor tile - nodes are either both, or neighbor
    for( viIt = neighVertexes.begin(); viIt != neighVertexes.end(); viIt++ ) {
        nodeMembershipSearch           currentSearch( curTree, viIt->getPoint(), 1 ); 

        if( currentSearch.begin() != currentSearch.end() ) {
            nodeMembershipSearch::iterator csit = currentSearch.begin();
            nodeMembershipData             csdata  = csit->first;
            double                         dist_sq = csit->second;

            if ( dist_sq < THRESHOLD_SAME ) {
                // use the current info id - from the tree
                nodeTree.insert( nodeMembershipData(viIt->getPoint(), NODE_BOTH, boost::get<2>(csdata)) );
            } else {
                nodeTree.insert( nodeMembershipData(viIt->getPoint(), NODE_NEIGHBOR, -1) );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "ERROR: search returned no results 1" );
        }
    }

    // now traverse current tile - nodes are either both, current ( or duplicate )
    for( viIt = curVertexes.begin(); viIt != curVertexes.end(); viIt++ ) {
        if ( !neighTree.empty() ) {
            nodeMembershipSearch           neighborSearch( neighTree, viIt->getPoint(), 1 );

            if( neighborSearch.begin() != neighborSearch.end() ) {
                nodeMembershipSearch::iterator nsit = neighborSearch.begin();
                nodeMembershipData             nsdata  = nsit->first;
                double                         dist_sq = nsit->second;
                meshTriPoint                   pt = boost::get<0>(nsdata);

                if ( dist_sq < THRESHOLD_SAME ) {
                    // only add nodes in BOTH onece - is it already in nodeTree?
                    nodeMembershipSearch duplicateSearch( nodeTree, pt, 1 );
                    if ( duplicateSearch.begin() != duplicateSearch.end() ) {
                        double               dupDist_sq = duplicateSearch.begin()->second;

                        if ( dupDist_sq > THRESHOLD_SAME ) {
                            // use the current info ID - from the original array
                            nodeTree.insert( nodeMembershipData(viIt->getPoint(), NODE_BOTH, viIt->getId() ) );                    
                        } else {
                            // duplicate - already in the membership tree as both
                        }
                    } else {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "ERROR: search returned no results 2" );
                    }
                } else {
                    // no neighbor near - just us
                    nodeTree.insert( nodeMembershipData(viIt->getPoint(), NODE_CURRENT, viIt->getId() ) );
                }
            } else {
                // no neighbors at all - just add current
                nodeTree.insert( nodeMembershipData(viIt->getPoint(), NODE_CURRENT, viIt->getId() ) );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "ERROR: search returned no results 3" );
        }
    }

    // we now have a search tree with all nodes on the shared edge.  each node is marked cur, neigh, or both.
    // now we look for cur and neigh nodes that are very close to an opposite neigh or current to merge them.
    // first - let's dump the debug output
    nodeMembershipTree::const_iterator mit;
    unsigned int nodeCnt = 1;

    // now traverse the tree again, and either add or merge nodes.  
    // This tree should be exactly the same for each tile sharing the edge.
    // If it isn't we'll get T-Junctions at tile boundaries...

    // we can't actively search the tree while traversing it...
    // make a copy of the nodeDatas
    std::vector<nodeMembershipData> queryPoints;
    for ( mit = nodeTree.begin(); mit != nodeTree.end(); mit++ ) {
        queryPoints.push_back( *mit );
    }

    std::vector<nodeMembershipData>::const_iterator qpit = queryPoints.begin();
    while ( qpit != queryPoints.end() ) {
        // Only worry about nodes that are NOT on both edges - either add them or merge them
        if ( boost::get<1>(*qpit) != NODE_BOTH ) {

            // do a search for the closest node to this one 
            // - search for 2 nodes, as the closest one will be this one :)
            meshTriPoint qp = boost::get<0>(*qpit);
            nodeMembershipSearch memberSearch( nodeTree, qp, 2 );

            if ( memberSearch.begin() != memberSearch.end() ) {
                nodeMembershipSearch::iterator rit = memberSearch.begin(); 

                nodeMembershipData thisNode = rit->first;
                rit++;
                nodeMembershipData nextNode = rit->first;
                double             distSq   = rit->second;

                // if the distance between nodes is less than the merge threshold - see if we can merge them.
                if ( distSq < THRESHOLD_TOO_CLOSE ) {
                    std::string debugInfo = "dist is within merge thresh.  ";
                    switch( boost::get<1>(thisNode) ) {
                        case NODE_BOTH:     debugInfo += " this is BOTH, ";       break;
                        case NODE_CURRENT:  debugInfo += " this is CURRENT, ";    break;
                        case NODE_NEIGHBOR: debugInfo += " this is NEIGHBOR, ";   break;
                    }
                    switch( boost::get<1>(nextNode) ) {
                        case NODE_BOTH:     debugInfo += " next is BOTH";         break;
                        case NODE_CURRENT:  debugInfo += " next is CURRENT";      break;
                        case NODE_NEIGHBOR: debugInfo += " next is NEIGHBOR";     break;
                    }

                    // merge these points only if they are in opposite tiles
                    if ( boost::get<1>(nextNode) != NODE_BOTH ) {
                        int index;

                        // NOTE - we're going to add the moved node twice ( once from curent, and once from neighbot.
                        // This is OK, as we will lookup and find just the one - from current
                        if ( (boost::get<1>(thisNode) == NODE_CURRENT) && (boost::get<1>(nextNode) == NODE_NEIGHBOR) ) {
                            debugInfo += ": MERGE";

                            // moving thisNode to midpoint of this and next
                            index = boost::get<2>(thisNode);
                            if ( vertexIndexToHandleMap.find(index) != vertexIndexToHandleMap.end() ) {
                                meshTriTDS::Vertex_handle vHand = vertexIndexToHandleMap[index];
                                movedNodes.push_back( movedNode(vHand, boost::get<0>(thisNode), CGAL::midpoint( boost::get<0>(thisNode), boost::get<0>(nextNode)) ) );
                            } else {
                                SG_LOG(SG_GENERAL, SG_INFO, "Can't find index " << index << " map size is " << vertexIndexToHandleMap.size() );
                            }
                        } else if ( (boost::get<1>(thisNode) == NODE_NEIGHBOR) && (boost::get<1>(nextNode) == NODE_CURRENT) ) {
                            debugInfo += ": MERGE";

                            // moving nextNode to midpoint of this and next
                            index = boost::get<2>(nextNode);
                            if ( vertexIndexToHandleMap.find(index) != vertexIndexToHandleMap.end() ) {
                                meshTriTDS::Vertex_handle vHand = vertexIndexToHandleMap[index];
                                movedNodes.push_back( movedNode(vHand, boost::get<0>(nextNode), CGAL::midpoint( boost::get<0>(thisNode), boost::get<0>(nextNode)) ) );
                            } else {
                                SG_LOG(SG_GENERAL, SG_INFO, "Can't find index " << index << " map size is " << vertexIndexToHandleMap.size() );
                            }
                        } else {
                            // we've found 2 points that are very close in the same tile - if it's the neighbor tile, go ahead and add it
                            if ( boost::get<1>(thisNode) == NODE_NEIGHBOR ) {
                                debugInfo += ": CAN'T MERGE - both points neighbor - addding neighbor node";
                                addedNodes.push_back( boost::get<0>(thisNode) );
                            } else {
                                debugInfo += ": CAN'T MERGE - both points current - ignore";
                            }
                        }
                    } else {
                        // current node is on just one edge, but next is on both 
                        // if it is on the neighbor edge, add it to current
                        if ( boost::get<1>(thisNode) == NODE_NEIGHBOR ) {
                            debugInfo += ": CAN'T MERGE - next closest on both edges - adding neighbor node";
                            addedNodes.push_back( boost::get<0>(thisNode) );
                        } else {
                            // current node is by definition, on current tile edge.
                            debugInfo += ": CAN'T MERGE - next closest on both edges - skipping current node";
                        }
                    }

                    SG_LOG( SG_GENERAL, SG_DEBUG, debugInfo );
                } else {
                    // distance between nodes is too large to merge 
                    // - if cur is on neighbor edge, add it to current
                    if ( boost::get<1>(thisNode) == NODE_NEIGHBOR ) {
                        addedNodes.push_back( boost::get<0>(thisNode) );
                    } else {
                        // current node is by definition, on current tile edge.
                    }
                }
            } else {
                SG_LOG(SG_GENERAL, SG_DEBUG, "ERROR: search returned no results 4" );
            }
        } else {
            // node is already on both edges - ignore.
        }

        // try next node ( or next  next, in case of merge )
        if ( qpit != queryPoints.end() ) {
            nodeCnt++;
            qpit++;
        }
    }
}

void tgMeshTriangulation::loadStage1SharedEdge( const std::string& p, const SGBucket& bucket, edgeType edge, std::vector<meshVertexInfo>& points ) 
{
    const char *edgestr[4] = {
        "north",
        "south",
        "east",
        "west"
    };

    char filename[64];
    sprintf( filename, "stage1_%s.shp", edgestr[edge] );
    std::string filePath = p + bucket.gen_base_path() + "/" + bucket.gen_index_str() + "/" + filename;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Loading Bucket " << bucket.gen_index_str() << " edge " << edgestr[edge] << " from " << filePath );           
    fromShapefile( filePath, points );

    SG_LOG(SG_GENERAL, SG_DEBUG, "Loaded " << points.size() << " nodes on edge " << edgestr[edge] );        
}

// load stage1 triangulation - translate nodes on edges if we merged nodes with a shared edge
bool tgMeshTriangulation::loadTriangulation( const std::string& basePath, const SGBucket& bucket )
{
    std::string bucketPath = basePath + "/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
    bool hasLand = loadTds( bucketPath );

    if ( hasLand ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - Triangulation valid? " << meshTriangulation.is_valid() );

        // match edges
        std::vector<meshVertexInfo> currentNorth, currentSouth, currentEast, currentWest;
        std::vector<meshVertexInfo> neighborNorth, neighborSouth, neighborEast, neighborWest;

        // we sort nodes on load, rather than save, as north / south edges MAY
        // have multiple buckets involved...

        // TODO - if neighbor has larger border than us, skip them.
        // I need some data to test this.
        // Ask Martin...

        // load our shared edge data
        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - north edge " );

        loadStage1SharedEdge( basePath, bucket, NORTH_EDGE, currentNorth );
        sortByLon( currentNorth );

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - south edge " );

        loadStage1SharedEdge( basePath, bucket, SOUTH_EDGE, currentSouth );
        sortByLon( currentSouth );

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - east edge " );

        loadStage1SharedEdge( basePath, bucket, EAST_EDGE, currentEast );
        sortByLat( currentEast );

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - west edge " );

        loadStage1SharedEdge( basePath, bucket, WEST_EDGE, currentWest );
        sortByLat( currentWest );

        // load southern edge(s) of northern neighbor(s)
        std::vector<SGBucket> northBuckets;
        bucket.siblings( 0, 1, northBuckets );
        for ( unsigned int i=0; i<northBuckets.size(); i++ ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - neigbor south " );

            loadStage1SharedEdge( basePath, northBuckets[i], SOUTH_EDGE, neighborNorth );
        }
        sortByLon( neighborNorth );        

        // load northern edge(s) of southern neighbor(s)
        std::vector<SGBucket> southBuckets;
        bucket.siblings( 0, -1, southBuckets );
        for ( unsigned int i=0; i<southBuckets.size(); i++ ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - neigbor north " );

            loadStage1SharedEdge( basePath, southBuckets[i], NORTH_EDGE, neighborSouth );
        }
        sortByLon( neighborSouth );        

        // load eastern edge of western neighbor
        SGBucket westBucket = bucket.sibling(-1, 0);

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - neigbor east " );

        loadStage1SharedEdge( basePath, westBucket, EAST_EDGE, neighborWest );
        sortByLat( neighborWest );

        // load western edge of eastern neighbor
        SGBucket eastBucket = bucket.sibling( 1, 0);

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - neigbor west " );

        loadStage1SharedEdge( basePath, eastBucket, WEST_EDGE, neighborEast );
        sortByLat( neighborEast );

        // match edges - add corrected locations into search tree, and new nodes into array
        std::vector<meshTriPoint> addedNodes;
        std::vector<movedNode>    movedNodes;

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - match north " );
        matchNodes( NORTH_EDGE, currentNorth, neighborNorth, addedNodes, movedNodes );

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - match south " );
        matchNodes( SOUTH_EDGE, currentSouth, neighborSouth, addedNodes, movedNodes );

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - match east " );
        matchNodes( EAST_EDGE,  currentEast,  neighborEast,  addedNodes, movedNodes );

        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - match west " );
        matchNodes( WEST_EDGE,  currentWest,  neighborWest,  addedNodes, movedNodes );

#if DEBUG_SHARED_EDGE
        // dump the added and moved nodes for the tile
        toShapefile( mesh->getDebugPath(), "added_nodes", addedNodes );
        toShapefile( mesh->getDebugPath(), "moved_nodes", movedNodes );
#endif

        // remesh with the moved and added nodes to the triangulation
        SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation - remesh " );

        constrainedTriangulateWithoutEdgeModification( movedNodes, addedNodes );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTriangulation complete.  hasLand " << hasLand );

    return hasLand;
}

const double fgPoint3_Epsilon = 0.00000005;
void tgMeshTriangulation::getEdgeNodes( std::vector<const meshVertexInfo *>& north, std::vector<const meshVertexInfo *>& south, std::vector<const meshVertexInfo *>& east, std::vector<const meshVertexInfo *>& west ) const {
    SGBucket b = mesh->getBucket();

    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();

    meshTriPoint            ll, lr, ul, ur;
    findVertexInfoFuzzyBox  exact_bb;

    std::list<findVertexInfoData>           result;
    std::list<findVertexInfoData>::iterator it;

    north.clear();
    south.clear();
    east.clear();
    west.clear();

    // generate the search tree
    const meshTriTDS& tds = meshTriangulation.tds();
    findVertexInfoTree tree;

    for( meshTriTDS::Vertex_iterator vit= tds.vertices_begin(); vit != tds.vertices_end() ; ++vit) {
        if ( vit != meshTriTDS::Vertex_handle() ) {
            if ( !vertexHandleToIndexMap.is_defined(vit) ) {
                SG_LOG(SG_GENERAL, SG_INFO, " vit not defioned in map " );
            } else {
                int idx = vertexHandleToIndexMap[vit];

                findVertexInfoData data( vit->point(), idx );
                tree.insert( data );
            }
        }
    }

    // find northern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, north_compare - fgPoint3_Epsilon );
    lr = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    ul = meshTriPoint( west_compare - fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
#if DEBUG_SHARED_EDGE
    saveEdgeBoundingBox( lr, ll, ul, ur, "edgebb_north" );
#endif

    exact_bb = findVertexInfoFuzzyBox(ll, ur);

    result.clear();
    tree.search(std::back_inserter( result ), exact_bb);

    for ( it = result.begin(); it != result.end(); it++ ) {
        int idx = boost::get<1>(*it);
        const meshVertexInfo* vip = &vertexInfo[idx];

        north.push_back( vip );
    }

    // find southern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    lr = meshTriPoint( east_compare + fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, south_compare + fgPoint3_Epsilon );
    ul = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare + fgPoint3_Epsilon );
#if DEBUG_SHARED_EDGE
    saveEdgeBoundingBox( lr, ll, ul, ur, "edgebb_north" );
#endif

    exact_bb = findVertexInfoFuzzyBox(ll, ur);

    result.clear();
    tree.search(std::back_inserter( result ), exact_bb);

    for ( it = result.begin(); it != result.end(); it++ ) {
        int idx = boost::get<1>(*it);
        const meshVertexInfo* vip = &vertexInfo[idx];

        south.push_back( vip );
    }

    // find eastern points
    ll = meshTriPoint( east_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    lr = meshTriPoint( east_compare + fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    ul = meshTriPoint( east_compare - fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
#if DEBUG_SHARED_EDGE
    saveEdgeBoundingBox( lr, ll, ul, ur, "edgebb_north" );
#endif

    exact_bb = findVertexInfoFuzzyBox(ll, ur);

    result.clear();
    tree.search(std::back_inserter( result ), exact_bb);

    for ( it = result.begin(); it != result.end(); it++ ) {
        int idx = boost::get<1>(*it);
        const meshVertexInfo* vip = &vertexInfo[idx];

        east.push_back( vip );
    }

    // find western points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    lr = meshTriPoint( west_compare + fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( west_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    ul = meshTriPoint( west_compare - fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
#if DEBUG_SHARED_EDGE
    saveEdgeBoundingBox( lr, ll, ul, ur, "edgebb_north" );
#endif

    exact_bb = findVertexInfoFuzzyBox(ll, ur);

    result.clear();
    tree.search(std::back_inserter( result ), exact_bb);

    for ( it = result.begin(); it != result.end(); it++ ) {
        int idx = boost::get<1>(*it);
        const meshVertexInfo* vip = &vertexInfo[idx];

        west.push_back( vip );
    }
}

static bool lessLatitude(const meshVertexInfo& a, const meshVertexInfo& b)
{
    return a.getY() < b.getY();
}

static bool lessLongitude(const meshVertexInfo& a, const meshVertexInfo& b)
{
    return a.getX() < b.getX();
}

void tgMeshTriangulation::sortByLat( std::vector<meshVertexInfo>& points ) const
{
    std::sort( points.begin(), points.end(), lessLatitude );
}

void tgMeshTriangulation::sortByLon( std::vector<meshVertexInfo>& points ) const
{
    std::sort( points.begin(), points.end(), lessLongitude );    
}

void tgMeshTriangulation::saveSharedEdgeNodes( const std::string& path ) const
{
    std::vector<const meshVertexInfo *> north;
    std::vector<const meshVertexInfo *> south;
    std::vector<const meshVertexInfo *> east;
    std::vector<const meshVertexInfo *> west;

    getEdgeNodes( north, south, east, west );

    // save these arrays in a point layer
    toShapefile( path, "stage1_north", north );
    toShapefile( path, "stage1_south", south );
    toShapefile( path, "stage1_east",  east );
    toShapefile( path, "stage1_west",  west );
}

void tgMeshTriangulation::saveIncidentFaces( const std::string& path, const char* layer, const std::vector<const meshVertexInfo *>& edgeVertexes ) const
{
#if 0
    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer    = NULL;
    OGRLayer*     poTriangleLayer = NULL;

    poDS = mesh->openDatasource( path );
    if ( poDS ) {
        char layerName[128];

        sprintf( layerName, "%s_points", layer );
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_TDS_VERTEX, layerName );

        sprintf( layerName, "%s_triangles", layer );
        poTriangleLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_TDS_FACE, layerName );

        SG_LOG(SG_GENERAL, SG_INFO, "Saving incident faces for " << edgeVertexes.size() << " vertexes" );

        // gather just the vertices and faces we want.
        std::vector<meshTriVertexHandle>                        vertexes;
        CGAL::Unique_hash_map<meshTriVertexHandle,unsigned int> V;
        unsigned int                                            vNum = 0;

        std::vector<meshTriFaceHandle>                          faces;
        CGAL::Unique_hash_map<meshTriFaceHandle,unsigned int>   F;
        unsigned int                                            fNum = 0;

        // first, save the vertices for this shared edge
        for (unsigned int i=0; i<edgeVertexes.size(); i++) {
            meshTriFaceCirculator ccf = meshTriangulation.incident_faces( edgeVertexes[i] );
            meshTriFaceCirculator cur = ccf;

            do {
                meshTriFaceHandle curFace = cur;
                if ( !meshTriangulation.is_infinite( curFace ) ) {
                    if ( !F.is_defined( curFace ) ) {
                        faces.push_back(curFace);
                        F[curFace] = fNum++;

                        // save the vertices
                        for (unsigned int j=0; j<3; j++ ) {
                            meshTriVertexHandle vit = curFace->vertex(j);
                            if ( !V.is_defined(vit) ) {
                                vertexes.push_back( vit );
                                V[vit] = vNum++;
                            }
                        }
                    }
                }
                cur++;
            } while ( cur != ccf );
        }

        // save the vertices
        for ( unsigned int i=0; i<vertexes.size(); i++ ) {
            // toShapefile( poPointLayer, vertexes[i]->point(), vertexes[i]->info().getElevation(), V[vertexes[i]] );
            toShapefile( poPointLayer, vertexes[i], V );
        }

        // save the faces 
        for ( unsigned int i=0; i<faces.size(); i++ ) {
            toShapefile( poTriangleLayer, faces[i], V, F );
        }

        GDALClose( poDS );  
    }
#endif
}

void tgMeshTriangulation::saveSharedEdgeFaces( const std::string& path ) const
{
    std::vector<const meshVertexInfo *> north;
    std::vector<const meshVertexInfo *> south;
    std::vector<const meshVertexInfo *> east;
    std::vector<const meshVertexInfo *> west;

    getEdgeNodes( north, south, east, west );

    // save all faces containing these points
    saveIncidentFaces( path, "stage2_north", north );
    saveIncidentFaces( path, "stage2_south", south );
    saveIncidentFaces( path, "stage2_east",  east );
    saveIncidentFaces( path, "stage2_west",  west );
}
