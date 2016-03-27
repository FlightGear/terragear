#include <algorithm>
#include <functional>

#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

#define THRESHOLD_SAME      (0.0000000000001)
#define THRESHOLD_TOO_CLOSE (0.00000001)

/* This will add or move nodes to match a neighbor edge.  Algorithm is designed to give the same result for both tiles.  
 * (it is run twice - once for each tile on the shared edge )
 */
void tgMeshTriangulation::matchNodes( edgeType edge, std::vector<meshVertexInfo>& curVertexes, std::vector<meshVertexInfo>& neighVertexes, std::vector<meshTriPoint>&addedNodes, moveNodeTree& movedNodes )
{        
    // we'll build a search tree of all nodes on the shared edge - flag which ones are current, which are neighbor, and which are both.
    // to determine if they are on both, we'll create 2 kd-trees to search first.
    edgeNodeTree        curTree, neighTree;
    std::vector<meshVertexInfo>::iterator viIt;
    
    for ( viIt = curVertexes.begin(); viIt != curVertexes.end(); viIt++ ) {
        curTree.insert( viIt->getPoint() );        
    }
    
    for ( viIt = neighVertexes.begin(); viIt != neighVertexes.end(); viIt++ ) {
        neighTree.insert( viIt->getPoint() );        
    }
    
    nodeMembershipTree  nodeTree;    
    // std::vector<meshTriPoint>::iterator nodeIt;
    
    const char *edgestr[4] = {
        "north",
        "south",
        "east",
        "west"
    };
    
    SG_LOG(SG_GENERAL, SG_ALERT, "edge matching bucket " << mesh->getBucket().gen_index_str() << " edge " << edgestr[edge] << ".  current - " << curTree.size() << ", neighbor - " << neighTree.size() );           
    
    int cur_neigh = 10000;
    for( viIt = neighVertexes.begin(); viIt != neighVertexes.end(); viIt++ ) {
        edgeNodeSearch  neighborSearch( curTree, viIt->getPoint(), 1 );        
        double          dist_sq = neighborSearch.begin()->second;
        
        if ( dist_sq < THRESHOLD_SAME ) {
            nodeMembershipData data( viIt->getPoint(), NODE_BOTH, cur_neigh );
            nodeTree.insert(data);
        } else {
            nodeMembershipData data( viIt->getPoint(), NODE_NEIGHBOR, cur_neigh );
            nodeTree.insert(data);
        }
        
        cur_neigh++;
    }
    
    // now traverse current tile
    int cur_curr = 20000;
    for( viIt = curVertexes.begin(); viIt != curVertexes.end(); viIt++ ) {
        if ( !neighTree.empty() ) {
            edgeNodeSearch  neighborSearch( neighTree, viIt->getPoint(), 1 );
            
            meshTriPoint    pt      = neighborSearch.begin()->first;
            double          dist_sq = neighborSearch.begin()->second;
            
            if ( dist_sq < THRESHOLD_SAME ) {
                // only add nodes in BOTH one - is it already in nodeTree?
                nodeMembershipSearch memberSearch( nodeTree, pt, 1 );
                double               dupDist_sq = memberSearch.begin()->second;
                
                if ( dupDist_sq > THRESHOLD_SAME ) {
                    nodeMembershipData data( viIt->getPoint(), NODE_BOTH, cur_curr );
                    nodeTree.insert(data);
                } else {
                    // duplicate - already in the membership tree
                }
            } else {
                nodeMembershipData data( viIt->getPoint(), NODE_CURRENT, cur_curr );
                nodeTree.insert(data);
            }
        } else {
            nodeMembershipData data( viIt->getPoint(), NODE_CURRENT, cur_curr );
            nodeTree.insert(data);            
        }
        cur_curr++;
    }
    
    // we now have a search tree with all nodes on the shared edge.  each node is marked cur, neigh, or both.
    // now we look for cur and neigh nodes that are very close to an opposite neigh otr current to merge them.
    // first - let's dump the debug output
    
    // open debug layer
    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer = NULL;
    char          layerName[64];
    
    sprintf( layerName, "shared_edge_matching_%s", edgestr[edge] );
    
    poDS = mesh->openDatasource( mesh->getDebugPath() );
    if ( poDS ) {
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, layerName );
    }
    
    nodeMembershipTree::const_iterator mit;
    for ( mit=nodeTree.begin(); mit !=nodeTree.end(); mit++ ) {
        char desc[64];
        
        switch( boost::get<1>(*mit) ) {
            case NODE_CURRENT:
                sprintf( desc, "current_%d", boost::get<2>(*mit) );
                break;
                
            case NODE_NEIGHBOR:
                sprintf( desc, "neighbor_%d", boost::get<2>(*mit) );
                break;
                
            case NODE_BOTH:
                sprintf( desc, "both_%d", boost::get<2>(*mit) );
                break;
        }
        toShapefile( poPointLayer, boost::get<0>(*mit), desc );
    }
    
    // close datasource
    GDALClose( poDS );    
    
    SG_LOG(SG_GENERAL, SG_ALERT, "completed edge matching bucket " << mesh->getBucket().gen_index_str() << " edge " << edgestr[edge] << " tree size is " << nodeTree.size() ); 
    
    // list get's corrupted - is it already corrupt?
    mit=nodeTree.begin(); 
    
    unsigned int numNodes = nodeTree.size();
    unsigned int nodeCnt = 1;
    while ( mit != nodeTree.end() ) {
        switch( boost::get<1>(*mit) ) {
            case NODE_BOTH:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START MATCH " << nodeCnt << " of " << numNodes << ": cur node " <<  boost::get<2>(*mit) << " is BOTH " ); 
                break;
                
            case NODE_CURRENT:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START MATCH " << nodeCnt << " of " << numNodes << ": cur node " <<  boost::get<2>(*mit) << " is CURRENT " ); 
                break;
                
            case NODE_NEIGHBOR:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START_MATCH " << nodeCnt << " of " << numNodes << ": cur node " <<  boost::get<2>(*mit) << " is NEIGHBOR " ); 
                break;
                
            default:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START_MATCH " << nodeCnt << " of " << numNodes << ": cur node is UNKNOWN " << boost::get<1>(*mit) ); 
                break;
        }
        
        nodeCnt++;
        mit++;
    }
    
    toShapefile( mesh->getDebugPath(), "membership", nodeTree );
    
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
        switch( boost::get<1>(*qpit) ) {
            case NODE_BOTH:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START MATCH " << nodeCnt << " of " << numNodes << ": cur node " <<  boost::get<2>(*qpit) << " is BOTH " ); 
                break;
                
            case NODE_CURRENT:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START MATCH " << nodeCnt << " of " << numNodes << ": cur node " <<  boost::get<2>(*qpit) << " is CURRENT " ); 
                break;
                
            case NODE_NEIGHBOR:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START_MATCH " << nodeCnt << " of " << numNodes << ": cur node " <<  boost::get<2>(*qpit) << " is NEIGHBOR " ); 
                break;
                
            default:
                SG_LOG(SG_GENERAL, SG_DEBUG, "START_MATCH " << nodeCnt << " of " << numNodes << ": cur node is UNKNOWN " << boost::get<1>(*qpit) ); 
                break;
        }
        
        if ( boost::get<1>(*qpit) != NODE_BOTH ) {
            
            // do a search for the closest node to this one 
            // - search for 2 nodes, as the closest one will be this one :)
            meshTriPoint qp = boost::get<0>(*qpit);
            nodeMembershipSearch memberSearch( nodeTree, qp, 2 );
            nodeMembershipSearch::iterator rit = memberSearch.begin(); 
            
            nodeMembershipData thisNode = rit->first;
            rit++;
            nodeMembershipData nextNode = rit->first;
            double             distSq   = rit->second;
            
            switch( boost::get<1>(thisNode) ) {
                case NODE_BOTH:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "cur not both - this is BOTH " ); 
                    break;
                    
                case NODE_CURRENT:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "cur not both - this is CURRENT " ); 
                    break;
                    
                case NODE_NEIGHBOR:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "cur not both - this is NEIGBOR " ); 
                    break;
            }
            
            switch( boost::get<1>(nextNode) ) {
                case NODE_BOTH:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "cur not both - next is BOTH " ); 
                    break;
                    
                case NODE_CURRENT:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "cur not both - next is CURRENT " ); 
                    break;
                    
                case NODE_NEIGHBOR:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "cur not both - next is NEIGBOR " ); 
                    break;
            }
            
            // if the distance between nodes is less than the merge threshold - see if we can merge them.
            if ( distSq < THRESHOLD_TOO_CLOSE ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "dist is too close " << distSq );
                
                // merge these points only if they are in opposite tiles
                if ( boost::get<1>(nextNode) != NODE_BOTH ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "next is NOT both " );
                    
                    // NOTE - we're going to add the moved node twice ( once from curent, and once from neighbot.
                    // This is OK, as we will lookup and find just the one - from current
                    if ( (boost::get<1>(thisNode) == NODE_CURRENT) && (boost::get<1>(nextNode) == NODE_NEIGHBOR) ) {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "this is CUR, next is NEIGHBOR - merge " );
                        
                        // moving thisNode to midpoint of this and next
                        moveNodeData movedNode( boost::get<0>(thisNode), CGAL::midpoint( boost::get<0>(thisNode), boost::get<0>(nextNode) ) );
                        movedNodes.insert( movedNode );                    
                    } else if ( (boost::get<1>(thisNode) == NODE_NEIGHBOR) && (boost::get<1>(nextNode) == NODE_CURRENT) ) {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "this is NEIGH, next is CUR - merge " );
                        
                        // moving nextNode to midpoint of this and next
                        moveNodeData movedNode( boost::get<0>(nextNode), CGAL::midpoint( boost::get<0>(thisNode), boost::get<0>(nextNode) ) );
                        movedNodes.insert( movedNode );                        
                    } else {
                        // we've found 2 points that are very close in the same tile - if it's the neighbor tile, go ahead and add it
                        if ( boost::get<1>(thisNode) == NODE_NEIGHBOR ) {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "Can't merge - bith points on neighbor - adding this neighbor node" );
                            addedNodes.push_back( boost::get<0>(thisNode) );
                        } else {
                            SG_LOG(SG_GENERAL, SG_DEBUG, "very close points, but but both are in current tile - ignore " );
                        }
                    }
                } else {
                    // current node is on just one edge, but next is on both 
                    // if it is on the neighbor edge, add it to current
                    if ( boost::get<1>(thisNode) == NODE_NEIGHBOR ) {
                        SG_LOG(SG_GENERAL, SG_DEBUG, "Can't merge - next closest on both edges - adding this neighbor node" );
                        addedNodes.push_back( boost::get<0>(thisNode) );
                    } else {
                        // current node is by definition, on current tile edge.
                        SG_LOG(SG_GENERAL, SG_DEBUG, "Can't merge - next closest on both edges - skipping node on current tile" );
                    }
                }
            } else {
                // distance between nodes is too large to merge 
                // - if cur is on neighbor edge, add it to current
                if ( boost::get<1>(thisNode) == NODE_NEIGHBOR ) {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Don't merge - next closest too far away - adding this neighbor node" );
                    addedNodes.push_back( boost::get<0>(thisNode) );
                } else {
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Don't merge - next closest too far away - skipping node on current tile" );
                    // current node is by definition, on current tile edge.
                }
            }
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "ignore node - already in both tiles" );
            // node is already on both edges - ignore.
        }
        
        // try next node ( or next  next, in case of merge )
        if ( qpit != queryPoints.end() ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "   get next node" );
            nodeCnt++;
            qpit++;
        } else {
            SG_LOG(SG_GENERAL, SG_DEBUG, "   don't increment to next node - done" );            
        }
    }
    SG_LOG(SG_GENERAL, SG_DEBUG, "EDGE MATCH DONE" );    
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
    
    SG_LOG(SG_GENERAL, SG_ALERT, "Loading Bucket " << bucket.gen_index_str() << " edge " << edgestr[edge] << " from " << filePath );           
    fromShapefile( filePath, points );
}


void tgMeshTriangulation::loadTds(std::vector<meshVertexInfo>& points, std::vector<meshFaceInfo>& faces)
{
    meshTriTDS& tds = meshTriangulation.tds();
    tds.clear();
    
    unsigned int n = points.size();
    unsigned int m = faces.size();
    unsigned int i;
    int          d = 2;

    SG_LOG(SG_GENERAL, SG_ALERT, "LoadTDS - begin" );
    
    if ( n != 0 ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "set dimension " );

        tds.set_dimension(d);
    
        std::vector<meshTriTDS::Vertex_handle> V(n);
        std::vector<meshTriTDS::Face_handle>   F(m);
        
        // read the rest
        for( i = 0; i < n; i++ ) {
            int index = points[i].getId();
            // SG_LOG(SG_GENERAL, SG_ALERT, "set point at index " << index << " max is " << n+1 );
            
            V[index] = tds.create_vertex();
            
            // index 0 is infinite vertex
            if (index) {
                V[index]->set_point(points[i].getPoint());
            }
        }
    
        // Creation of the faces
        int dim = (tds.dimension() == -1 ? 1 :  tds.dimension() + 1);
        SG_LOG(SG_GENERAL, SG_ALERT, "dim is" << dim );
        
        for( i = 0; i < m; i++ ) {
            int fid = faces[i].getFid();
            
            F[fid] = tds.create_face();
            for(int j = 0; j < dim ; j++){
                int vid = faces[i].getVid(j);
                F[fid]->set_vertex(j, V[vid]);
                // The face pointer of vertices is set too often,
                // but otherwise we had to use a further map
                V[vid]->set_face( F[fid] );
            }
        }
        
        // Setting the neighbor pointers 
        for( i = 0; i < m; i++ ) {
            int fid = faces[i].getFid();
            
            for( int j = 0; j < tds.dimension()+1; j++ ){
                int nid = faces[i].getNid(j);
                F[fid]->set_neighbor(j, F[nid]);
            }
        }
        
        // setting constraints
        for( i = 0; i < m; i++ ) {
            int fid = faces[i].getFid();
            
            for( int j = 0; j < tds.dimension()+1; j++ ){
                int con = faces[i].getConstrained(j);
                F[fid]->set_constraint(j, con?true:false);
            }
        }
    }    
}

// load stage1 triangulation - translate nodes on edges if we merged nodes with a shared edge
void tgMeshTriangulation::loadTriangulation( const std::string& basePath, const SGBucket& bucket )
{
    SG_LOG(SG_GENERAL, SG_ALERT, "LoadTriangulation - begin" );
    
    std::vector<meshVertexInfo> currentNorth, currentSouth, currentEast, currentWest;
    std::vector<meshVertexInfo> neighborNorth, neighborSouth, neighborEast, neighborWest;
 
    std::string bucketPath = basePath + "/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
    
    // we sort nodes on load, rather than save, as north / south edges MAY
    // have multiple buckets involved...
    
    // TODO - if neighbor has larger border than us, skip them.
    // I need some data to test this.
    // Ask Martin...
#if 0    
    // load our shared edge data
    loadStage1SharedEdge( basePath, bucket, NORTH_EDGE, currentNorth );
    sortByLon( currentNorth );
    
    loadStage1SharedEdge( basePath, bucket, SOUTH_EDGE, currentSouth );
    sortByLon( currentSouth );
    
    loadStage1SharedEdge( basePath, bucket, EAST_EDGE, currentEast );
    sortByLat( currentEast );
    
    loadStage1SharedEdge( basePath, bucket, WEST_EDGE, currentWest );
    sortByLat( currentWest );
    
    // load southern edge(s) of northern neighbor(s)
    std::vector<SGBucket> northBuckets;
    bucket.siblings( 0, 1, northBuckets );
    for ( unsigned int i=0; i<northBuckets.size(); i++ ) {
        loadStage1SharedEdge( basePath, northBuckets[i], SOUTH_EDGE, neighborNorth );
    }
    sortByLon( neighborNorth );        
    
    // load northern edge(s) of southern neighbor(s)
    std::vector<SGBucket> southBuckets;
    bucket.siblings( 0, -1, southBuckets );
    for ( unsigned int i=0; i<southBuckets.size(); i++ ) {
        loadStage1SharedEdge( basePath, southBuckets[i], NORTH_EDGE, neighborSouth );
    }
    sortByLon( neighborSouth );        
    
    // load eastern edge of western neighbor
    SGBucket westBucket = bucket.sibling(-1, 0);
    loadStage1SharedEdge( basePath, westBucket, EAST_EDGE, neighborWest );
    sortByLat( neighborWest );
    
    // load western edge of eastern neighbor
    SGBucket eastBucket = bucket.sibling( 1, 0);
    loadStage1SharedEdge( basePath, eastBucket, WEST_EDGE, neighborEast );
    sortByLat( neighborEast );
    
    // match edges - add corrected locations into search tree, and new nodes into array
    std::vector<meshTriPoint> addedNodes;
    moveNodeTree              movedNodes;
    
    matchNodes( NORTH_EDGE, currentNorth, neighborNorth, addedNodes, movedNodes );
    matchNodes( SOUTH_EDGE, currentSouth, neighborSouth, addedNodes, movedNodes );
    matchNodes( EAST_EDGE,  currentEast,  neighborEast,  addedNodes, movedNodes );
    matchNodes( WEST_EDGE,  currentWest,  neighborWest,  addedNodes, movedNodes );
    
    // dump the added and moved nodes for the tile
    //toShapefile( mesh->getDebugPath(), "added_nodes", addedNodes );
    //toShapefile( mesh->getDebugPath(), "moved_nodes", movedNodes );
#endif

    // load the tile points
    std::vector<meshVertexInfo>   points;
    std::vector<meshFaceInfo>     faces;
    std::string                   filePath;
    
    // load vertices, and save their handles in V
    filePath = bucketPath + "/stage1_triangles_points.shp"; 
    fromShapefile( filePath, points );
    
    filePath = bucketPath + "/stage1_triangles_faces.shp"; 
    fromShapefile( filePath, faces );

    // update point positions of moved nodes
    //moveTileNodes( points, movedNodes );

    SG_LOG(SG_GENERAL, SG_ALERT, "LoadTriangulation - load TDS from " << points.size() << " points and " << faces.size() << " faces" );
    
    // load the TDS ( with moved nodes... )
    loadTds( points, faces );
        
    // add the new nodes to the triangulation
    

    // remesh ( without adding points to the edges... )
    
#if 0    
    
    
    // add the added nodes
    for ( unsigned int i=0; i<addedNodes.size(); i++ ) {
        points.push_back( addedNodes[i] );
    }
    
    // and the constraints
    for ( unsigned int i=0; i<constraints.size(); i++ ) {
        std::list<moveNodeData> searchResults;
        meshTriPoint src, trg;
        
        // first - create a search node
        searchResults.clear();
        moveNodeFuzzyCir  query_circle_source( constraints[i].source(), 0.0000001 );        
        movedNodes.search(std::back_inserter( searchResults ), query_circle_source );
        
        if ( !searchResults.empty() ) {
            // move this node
            std::list<moveNodeData>::const_iterator it = searchResults.begin();
            src = boost::get<1>(*it);
        } else {
            src = constraints[i].source();
        }
        
        searchResults.clear();
        moveNodeFuzzyCir  query_circle_target( constraints[i].target(), 0.0000001 );
        movedNodes.search(std::back_inserter( searchResults ), query_circle_target );
        
        if ( !searchResults.empty() ) {
            // move this node
            std::list<moveNodeData>::const_iterator it = searchResults.begin();
            trg = boost::get<1>(*it);
        } else {
            trg = constraints[i].target();
        }
        
        constraints[i] = meshTriSegment( src, trg );
    }

    // create the triangulation
    constrainedTriangulateWithoutEdgeModification( points, constraints );
#endif
}

const double fgPoint3_Epsilon = 0.00000001;
void tgMeshTriangulation::getEdgeNodes( std::vector<meshVertexInfo>& north, std::vector<meshVertexInfo>& south, std::vector<meshVertexInfo>& east, std::vector<meshVertexInfo>& west ) const {
#if 0 // todo
    SGBucket b = mesh->getBucket();

    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();
    
    meshTriPoint            ll;
    meshTriPoint            ur;
    findVertexInfoFuzzyBox  exact_bb;
    
    std::list<findVertexInfoData>           result;
    std::list<findVertexInfoData>::iterator it;
    
    north.clear();
    south.clear();
    east.clear();
    west.clear();
    
    // generate the search tree
    findVertexInfoTree tree;
    for (meshTriCDTPlus::Finite_vertices_iterator vit=meshTriangulation.finite_vertices_begin(); vit!=meshTriangulation.finite_vertices_end(); ++vit) {
        findVertexInfoData data( vit->point(), vertexInfo[vertexMap[vit]] );
        tree.insert( data );
    }
    
    // find northern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, north_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = findVertexInfoFuzzyBox(ll, ur);
    
    result.clear();
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        north.push_back( boost::get<1>(*it) );
    }
    
    // find southern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, south_compare + fgPoint3_Epsilon );
    exact_bb = findVertexInfoFuzzyBox(ll, ur);
    result.clear();
    
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        south.push_back( boost::get<1>(*it) );
    }
    
    // find eastern points
    ll = meshTriPoint( east_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = findVertexInfoFuzzyBox(ll, ur);
    result.clear();
    
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        east.push_back( boost::get<1>(*it) );
    }
    
    // find western points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( west_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = findVertexInfoFuzzyBox(ll, ur);
    result.clear();
    
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        west.push_back( boost::get<1>(*it) );
    }
#endif    
}

void tgMeshTriangulation::getEdgeVertices( std::vector<meshTriVertexHandle>& north, std::vector<meshTriVertexHandle>& south, std::vector<meshTriVertexHandle>& east, std::vector<meshTriVertexHandle>& west ) const {
    SGBucket b = mesh->getBucket();

    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();
    
    meshTriPoint        ll;
    meshTriPoint        ur;
    findVertexFuzzyBox  exact_bb;
    
    std::list<findVertexData>           result;
    std::list<findVertexData>::iterator it;
    
    north.clear();
    south.clear();
    east.clear();
    west.clear();
    
    // generate the search tree
    findVertexTree tree;
    for (meshTriCDTPlus::Finite_vertices_iterator vit=meshTriangulation.finite_vertices_begin(); vit!=meshTriangulation.finite_vertices_end(); ++vit) {
        // insert the location, and the handle into the tree
        findVertexData data( vit->point(), vit );
        tree.insert( data );
    }
    
    // find northern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, north_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = findVertexFuzzyBox(ll, ur);
    
    result.clear();
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        north.push_back( boost::get<1>(*it) );
    }
    
    // find southern points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, south_compare + fgPoint3_Epsilon );
    exact_bb = findVertexFuzzyBox(ll, ur);
    result.clear();
    
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        south.push_back( boost::get<1>(*it) );
    }
    
    // find eastern points
    ll = meshTriPoint( east_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = findVertexFuzzyBox(ll, ur);
    result.clear();
    
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        east.push_back( boost::get<1>(*it) );
    }
    
    // find western points
    ll = meshTriPoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = meshTriPoint( west_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = findVertexFuzzyBox(ll, ur);
    result.clear();
    
    tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        west.push_back( boost::get<1>(*it) );
    }
}

static bool lessLatitude(meshVertexInfo& a, meshVertexInfo& b)
{
    return a.getY() < b.getY();
}

static bool lessLongitude(meshVertexInfo& a, meshVertexInfo& b)
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
    std::vector<meshVertexInfo> north;
    std::vector<meshVertexInfo> south;
    std::vector<meshVertexInfo> east;
    std::vector<meshVertexInfo> west;
    
    getEdgeNodes( north, south, east, west );
    
    // save these arrays in a point layer
    toShapefile( path, "stage1_north", north );
    toShapefile( path, "stage1_south", south );
    toShapefile( path, "stage1_east",  east );
    toShapefile( path, "stage1_west",  west );
}

void tgMeshTriangulation::saveIncidentFaces( const std::string& path, const char* layer, const std::vector<meshTriVertexHandle>& edgeVertexes ) const
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
    std::vector<meshTriVertexHandle> north;
    std::vector<meshTriVertexHandle> south;
    std::vector<meshTriVertexHandle> east;
    std::vector<meshTriVertexHandle> west;
    
    getEdgeVertices( north, south, east, west );
    
    // save all faces containing these points
    saveIncidentFaces( path, "stage2_north", north );
    saveIncidentFaces( path, "stage2_south", south );
    saveIncidentFaces( path, "stage2_east",  east );
    saveIncidentFaces( path, "stage2_west",  west );
}