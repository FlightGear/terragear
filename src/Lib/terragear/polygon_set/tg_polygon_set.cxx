#include <simgear/debug/logstream.hxx>

#include "tg_polygon_set.hxx"

#include <simgear/misc/sg_path.hxx> // for file i/o


// every polygon set (metadata) gets its own unique identifier
unsigned long tgPolygonSetMeta::cur_id = 1;

void tgPolygonSet::polygonToSegmentList( const cgalPoly_Polygon& p, std::vector<cgalPoly_Segment>& segs ) const
{
    cgalPoly_Polygon::Vertex_const_iterator   src, trg;

    src = p.vertices_begin();
    trg = src; trg++;
    while( trg != p.vertices_end() ) {
        segs.push_back( cgalPoly_Segment(*src++, *trg++) );
    }
    trg = p.vertices_begin();
    segs.push_back( cgalPoly_Segment(*src, *trg) );    
}

void tgPolygonSet::toSegments( std::vector<cgalPoly_Segment>& segs, bool withHoles ) const
{
    std::list<cgalPoly_PolygonWithHoles>                 pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator it;

    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::toShapefile: got " << pwh_list.size() << " polys with holes ");
    
    // save each poly with holes to the layer
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        cgalPoly_PolygonWithHoles pwh = (*it);
        polygonToSegmentList( pwh.outer_boundary(), segs );
        
        if ( withHoles ) {
        }
    }
}

void tgPolygonSet::clusterNodes( const tgCluster& clusteredNodes )
{
    std::list<cgalPoly_PolygonWithHoles>                    pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator    pwhit;
    std::vector<cgalPoly_Polygon>                           boundaries;
    std::vector<cgalPoly_Polygon>                           holes;
    cgalPoly_PolygonSet                                     holesUnion;
    
    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::clusterNodes: got " << pwh_list.size() << " polys with holes ");

    // create faces from boundaries and holes
    for (pwhit = pwh_list.begin(); pwhit != pwh_list.end(); ++pwhit) {
        cgalPoly_PolygonWithHoles                           pwh = (*pwhit);
        cgalPoly_PolygonWithHoles::Hole_const_iterator      hit;
        cgalPoly_Polygon::Vertex_const_iterator             vit;        
        std::vector<cgalPoly_Point>                         nodes;
        cgalPoly_Polygon                                    poly;
        
        // get boundary face(s)
        nodes.clear();
        poly = pwh.outer_boundary();
        for ( vit = poly.vertices_begin(); vit != poly.vertices_end(); vit++ ) {
            // lookup clustered location
            nodes.push_back( clusteredNodes.Locate( *vit ) );
        }
        facesFromUntrustedNodes( nodes, boundaries );
        
        // get hole face(s)
        for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
            nodes.clear();
            poly = *hit;
            for ( vit = poly.vertices_begin(); vit != poly.vertices_end(); vit++ ) {
                // lookup clustered location
                nodes.push_back( clusteredNodes.Locate( *vit ) );
            }
            facesFromUntrustedNodes( nodes, holes );
        }
    }

    ps.clear();
    
    // join all the boundaries
    ps.join( boundaries.begin(), boundaries.end() );

    // join all the holes
    holesUnion.join( holes.begin(), holes.end() );

    // perform difference
    ps.difference( holesUnion );
}

void tgPolygonSet::facesFromUntrustedNodes( std::vector<cgalPoly_Point> nodes, std::vector<cgalPoly_Polygon>& faces )
{
    cgalPoly_Arrangement            arr;
    std::vector<cgalPoly_Segment>   segs;

    for (unsigned int i = 0; i < nodes.size(); i++) {
        cgalPoly_Point src = nodes[i];
        cgalPoly_Point trg;
        
        if ( i < nodes.size()-1 ) {
            // target is the next point
            trg = nodes[i+1];
        } else {
            // target is the first point
            trg = nodes[0];
        }

        if ( src != trg ) {
            segs.push_back( cgalPoly_Segment( src, trg ) );
        }
    }

    insert( arr, segs.begin(), segs.end() );

    // return the union of all bounded faces
    cgalPoly_FaceConstIterator fit;
    for( fit = arr.faces_begin(); fit != arr.faces_end(); fit++ ) {
        cgalPoly_Arrangement::Face face = (*fit);
        if( face.has_outer_ccb() ) {
            // generate Polygon from face, and join wuth polygon set
            cgalPoly_CcbHeConstCirculator ccb = face.outer_ccb();
            cgalPoly_CcbHeConstCirculator cur = ccb;
            cgalPoly_HeConstHandle        he;
            std::vector<cgalPoly_Point>   nodes;

            do
            {
                he = cur;

                // ignore inner antenna
                if ( he->face() != he->twin()->face() ) {                    
                    nodes.push_back( he->source()->point() );
                }
                
                ++cur;
            } while (cur != ccb);

            // check the orientation - outer boundaries should be CCW
            faces.push_back( cgalPoly_Polygon( nodes.begin(), nodes.end()  ));
        }
    }    
}




// a face does not mean we have a simple polygon - check if all nodes are degree 2
void tgPolygonSet::addBoundary( cgalPoly_FaceConstHandle& curFace, std::vector<cgalPoly_Polygon>& boundaries )
{
    cgalPoly_Arrangement::Face face = (*curFace);
    if( face.has_outer_ccb() ) {
        // generate Polygon from face, and join wuth polygon set
        cgalPoly_CcbHeConstCirculator ccb = face.outer_ccb();
        cgalPoly_CcbHeConstCirculator cur = ccb;
        cgalPoly_HeConstHandle        he;
        std::vector<cgalPoly_Point>   nodes;
        
        do
        {
            he = cur;
            
            // ignore inner antenna
            if ( he->face() != he->twin()->face() ) { 
                if ( he->source()->degree() > 2 ) {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - uhoh - we have a node with degree > 2" );
                }
                nodes.push_back( he->source()->point() );
            } else {
                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - drop antenna node  " );                            
            }
            
            ++cur;
        } while (cur != ccb);
        
        cgalPoly_Polygon poly( nodes.begin(), nodes.end()  );
        
        if ( !poly.is_simple() ) {
            SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - Poly is not simple " );                        
        }
        cgalPoly_PolygonSet test(poly);
        
        if ( !test.is_valid() ) {
            SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - face Creates invalid polygon set " );            
        }
 
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary " << boundaries.size() );
        
        // check the orientation - outer boundaries should be CCW
        boundaries.push_back( poly );
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - face has no outer ccb " );
    }        
}

void tgPolygonSet::printFace( const char* layer, cgalPoly_FaceConstHandle fh )
{    
    if( fh->has_outer_ccb() ) {
        // generate Polygon from face, and join wuth polygon set
        cgalPoly_CcbHeConstCirculator ccb = fh->outer_ccb();
        cgalPoly_CcbHeConstCirculator cur = ccb;
        cgalPoly_HeConstHandle        he;
        std::vector<cgalPoly_Point>   nodes;
        
        GDALDataset* poDS = openDatasource("Faces");
        
        char layer_node_name[64];
        sprintf( layer_node_name, "%s_badnodes", layer );
        
        char layer_edge_name[64];
        sprintf( layer_edge_name, "%s_edges", layer );
        
        OGRLayer*    poLayerNodes = openLayer(poDS, wkbPoint25D, layer_node_name);
        OGRLayer*    poLayerEdges = openLayer(poDS, wkbLineString25D, layer_edge_name);
        
        do
        {
            he = cur;
            
            // ignore inner antenna
            if ( he->face() != he->twin()->face() ) { 
                if ( he->source()->degree() > 2 ) {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - uhoh - we have a node with degree > 2" );
                    
                    // dump the bad nodes to a shapefile
                    toShapefile( poLayerNodes, he->source()->point(), "node" );
                }
                nodes.push_back( he->source()->point() );
            } else {
                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - drop antenna node  " );                            
            }
            
            ++cur;
        } while (cur != ccb);
        
        cgalPoly_Polygon poly( nodes.begin(), nodes.end()  );
        toShapefile( poLayerEdges, poly, false );
        
        GDALClose( poDS );
        
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - face has no outer ccb " );
    }    
}

void tgPolygonSet::addHole( cgalPoly_FaceConstHandle& curFace, std::vector<cgalPoly_Polygon>& holes )
{
    cgalPoly_Arrangement::Face face = (*curFace);
    if( face.has_outer_ccb() ) {
        // generate Polygon from face, and join wuth polygon set
        cgalPoly_CcbHeConstCirculator ccb = face.outer_ccb();
        cgalPoly_CcbHeConstCirculator cur = ccb;
        cgalPoly_HeConstHandle        he;
        std::vector<cgalPoly_Point>   nodes;
        
        do
        {
            he = cur;
            
            // ignore inner antenna
            if ( he->face() != he->twin()->face() ) {                    
                nodes.push_back( he->source()->point() );
            }
            
            ++cur;
        } while (cur != ccb);

        cgalPoly_Polygon poly( nodes.begin(), nodes.end()  );
        
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addHole " << holes.size() );
        
        // check the orientation - outer boundaries should be CCW
        holes.push_back( poly );
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addHole - face has no outer ccb " );
    }        
}

void tgPolygonSet::addBoundaries( cgalPoly_FaceConstHandle& curFace, std::vector<cgalPoly_Polygon>& boundaries, std::vector<cgalPoly_Polygon>& holes )
{
    // every hole in this face is a boundary
    cgalPoly_HoleConstIterator holeIt;
    for (holeIt = curFace->holes_begin(); holeIt != curFace->holes_end(); holeIt++) {
        // value of hole iterator is a ccb
        cgalPoly_CcbHeConstCirculator ccb = *holeIt; 
        cgalPoly_He he = *ccb;  
        
        cgalPoly_FaceConstHandle holeH = (cgalPoly_FaceConstHandle)he.twin()->face();
        
        addBoundary( holeH, boundaries);
        
        // then, add any interior holes as holes
        addHoles( holeH, boundaries, holes );
    }
}

void tgPolygonSet::addHoles( cgalPoly_FaceConstHandle& curFace, std::vector<cgalPoly_Polygon>& boundaries, std::vector<cgalPoly_Polygon>& holes )
{
    // every hole in this face is a hole
    cgalPoly_HoleConstIterator holeIt;
    for (holeIt = curFace->holes_begin(); holeIt != curFace->holes_end(); holeIt++) {
        // value of hole iterator is a ccb
        cgalPoly_CcbHeConstCirculator ccb = *holeIt; 
        cgalPoly_He he = *ccb;  
        cgalPoly_FaceConstHandle holeH = (cgalPoly_FaceConstHandle)he.twin()->face();
        
        addHole( holeH, holes );
        
        // then, add any interior holes as boundaries
        addBoundaries( holeH, boundaries, holes );
    }
}

unsigned int faceNum = 1;


// breakthrough!
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
 * 1) from start, create a new path with inner face F1, and outer face F2
 * 2) traverse and add all points until hitting Junction J1
 * 3) two ( of four ) incident halfedges are valid:
 *    First at 12:00 - same inner and outer faces, so we'll continue with that one
 *    Second at 7:30 - same inner face, different outer face.  We'll create a new
 *                     path for that one with inner face F1 and outer face H1.
 *                     Because the inner face is not a hole, the outer face MUST be. 
 * 4) continue to traverse from J1 at 12:00 and reach the CCB end == start condition.  
 *    This path is complete.  Find an incomplete 1 - we just have the one at 7:30
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



cgalPoly_CcbHeConstCirculator tgPolygonSet::addPolys( const cgalPoly_Arrangement& arr, bool isHole,
                             cgalPoly_FaceConstHandle insideFaceH, cgalPoly_CcbHeConstCirculator insideCcb, 
                             std::vector<cgalPoly_Polygon>& boundaries, std::vector<cgalPoly_Polygon>& holes,
                             GDALDataset* poDS )
{
    cgalPoly_CcbHeConstCirculator cur = insideCcb;
    
    if( insideFaceH->has_outer_ccb() ) {
        // generate Polygon from inside face
        cgalPoly_HeConstHandle        he = cur;

        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addPolys Start face " << faceNum << " at " << he->source()->point() );
        
        // if the hole is not simple, we'll have multiple polygons, so nodes are on the stack
        std::vector<cgalPoly_Point> nodes;
                
        char layer_node_name[64];
        sprintf( layer_node_name, "%02u_badnodes", faceNum );
        
        char layer_edge_name[64];
        sprintf( layer_edge_name, "%02u_edges", faceNum );

        faceNum++;
        
        OGRLayer*    poLayerNodes = openLayer(poDS, wkbPoint25D, layer_node_name);
        OGRLayer*    poLayerEdges = openLayer(poDS, wkbLineString25D, layer_edge_name);
        
        // exit criteria is the he source pointed to by cur is the same as pointer to by insideCcb
        
        cgalPoly_VertexConstHandle endVertex = he->source();
        do
        {            
            // ignore inner antenna
            if ( he->face() != he->twin()->face() ) { 
                // check for degenerate node 
                // ( but ok, it it's the first one, as all recursive entries atart at such verticies )
                if ( (he->source()->degree() > 2) && (he->source() != endVertex ) ) {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addPolys face " << faceNum << " - uhoh - we have a node with degree > 2 at " << he->source()->point() );
                    
                    // dump the bad nodes to a shapefile
                    toShapefile( poLayerNodes, he->source()->point(), "node" );
                    
                    // recurse into the inner poly - need to find the outside and inside faces...
                    if ( he->face() == insideFaceH ) {
                        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addPolys face " << faceNum << " - incident face is inside face - this is a hole " );
                        cur = addPolys( arr, true, insideFaceH, cur, boundaries, holes, poDS );
                    } else {
                        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addPolys face " << faceNum << " - incident face is not inside face - this is a a boundary " );                        
                        cur = addPolys( arr, false, insideFaceH, cur, boundaries, holes, poDS );
                    }
                }
                nodes.push_back( he->source()->point() );
            } else {
                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - drop antenna node  " );                            
            }
            
            ++cur;
            he = cur;
        } while (he->source() != endVertex);
        
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addPolys face " << faceNum << " - complete at " << he->source()->point() );
        
        cgalPoly_Polygon poly( nodes.begin(), nodes.end()  );
        toShapefile( poLayerEdges, poly, false );
                
        if ( isHole ) {
            holes.push_back( poly );
        } else {
            boundaries.push_back( poly );
        }
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::addBoundary - face has no outer ccb " );
    }    
    
    return cur;
}

/******************************************************************************
 * There are two degenerate cases possible when converting arrangement faces  *
 * to polygons.                                                               *
 * actually, CGAL will generate the polygon fine, but it may not be simple.   *
 *                                                                            *
 * case 1:                                                                    *
 *                                                                            *
 *               ------------                                                 *
 *                \        /                                                  *
 *                 \   A  /                                                   *
 *                  \    /                                                    *
 *                   \  /                                                     *
 *                    \/                                                      *
 *                    /\ v                                                    *
 *                   /  \                                                     *
 *                  /    \                                                    *
 *                 /   B  \                                                   *
 *                /        \                                                  *
 *                ----------                                                  *
 * vertex is degree 4                                                         *
 * result should be two boundaries                                            *
 *                                                                            *
 * case 2:                                                                    *
 *                                                                            *
 *               -----------                                                  *
 *               |         |                                                  *
 *               | A       |                                                  *
 *               |    |\   |                                                  *
 *               |    | \  |                                                  *
 *               |    |  \ |                                                  *
 *               |    | B \|                                                  *
 *               |    |   /| v                                                *
 *               |    |  / |                                                  *
 *               |    | /  |                                                  *
 *               |    |/   |                                                  *
 *               |         |                                                  *
 *               -----------                                                  *
 *                                                                            *
 * vertex is degree 4                                                         *
 * result should be a boundary and a hole                                     *
 *                                                                            *
 * It looks like we should be able to say the hole is 'inside' the boundary   *
 * for the second case, but CGAL doesn't see it that way.                     *
 * The face 'surrounds' the hole, but the hole is not 100% inside it. (at v)  *
 *                                                                            *
 * 1) find nodes with degree > 4                                              *
 * 2) split poly in two from degenerate node                                  *
 * 3) calc point a inside poly A, and b inside poly B                         *
 * 4) case 2 occurs whenever the twin halfedges of B are                      *
 *    incident to the face of A                                               *
 *                                                                            *
 * we need an arrangement of the faces - which we have...                     *
 *                                                                            *
 ******************************************************************************/

void tgPolygonSet::facesFromUntrustedNodes( const std::vector<cgalPoly_Point>& nodes, std::vector<cgalPoly_Polygon>& boundaries, std::vector<cgalPoly_Polygon>& holes )
{
    cgalPoly_Arrangement            arr;
    std::vector<cgalPoly_Segment>   segs;
    
    for (unsigned int i = 0; i < nodes.size(); i++) {
        cgalPoly_Point src = nodes[i];
        cgalPoly_Point trg;
        
        if ( i < nodes.size()-1 ) {
            // target is the next point
            trg = nodes[i+1];
        } else {
            // target is the first point
            trg = nodes[0];
        }
        
        if ( src != trg ) {
            segs.push_back( cgalPoly_Segment( src, trg ) );
        }
    }
    
    insert( arr, segs.begin(), segs.end() );

#if 0

    // TEST - let's dump the holes
    cgalPoly_FaceConstIterator fit;
    char layer[64];
    unsigned int i = 0;
    for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
        if ( fit->is_unbounded() ) {
            SG_LOG(SG_GENERAL, SG_INFO, "Found unbounded face" );
            
            sprintf( layer, "unbounded_%02u", i );
            printFace( layer, fit );
    } else {
        sprintf( layer, "bounded_%02u", i );
        printFace( layer, fit );            
    }
    i++;
    }
#endif

    // the 'main' poly is the hole in the unbounded face.
    // there may be other faces when the hole is not simple.
    
    // first, lets name and save all face handles for debugging - dump them to shapefiles
    std::vector<tgPSFaceID> arrFaces;
    identifyFaces( arr, arrFaces );
    dumpFaces( arrFaces );

    cgalPoly_FaceConstHandle outsideFaceH = arr.unbounded_face();
    
    // get the number of holes in the unbounded face
    cgalPoly_HoleConstIterator holeIt;
    unsigned int numHoles = 0;
    for (holeIt = outsideFaceH->holes_begin(); holeIt != outsideFaceH->holes_end(); holeIt++) {
        numHoles++;
    }
    
    if ( numHoles > 1 ) {
        // pretty sure this is impossible.  even a line segment is part of the hole...
        SG_LOG(SG_GENERAL, SG_ALERT, "tgPolygonSet::facesFromUntrustedNodes - unbounded face has more than one hole" );
        exit(0);
    }
    
    // add polys / holes from the single hole
    holeIt = outsideFaceH->holes_begin();
    
    // value of hole iterator is a ccb
    cgalPoly_CcbHeConstCirculator ccb = *holeIt; 
    cgalPoly_He                   he = *ccb;
    cgalPoly_FaceConstHandle      insideFaceH = (cgalPoly_FaceConstHandle)he.twin()->face();    

    if ( insideFaceH == outsideFaceH ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgPolygonSet::facesFromUntrustedNodes - insideFH == ousideFH" );
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgPolygonSet::facesFromUntrustedNodes - insideFH != ousideFH" );        
    }
    
#if 0    
    // we now have the face handle of the poly we want to add.  - dump it
    char layer[64];
    sprintf( layer, "unboundedhole" );
    printFace( layer, insideFaceH );            
    
    // recursively add polygons.
    // if we find a node with degree > 2, then we need to know what face the new poly is incident to.
    // recursion needs the outside face, inside face, the current halfedge, and a vector of vectors
    cgalPoly_CcbHeConstCirculator insideCcb = insideFaceH->outer_ccb();
    GDALDataset* poDS = openDatasource("Faces");
    addPolys( arr, false, insideFaceH, insideCcb, boundaries, holes, poDS );
    GDALClose( poDS );
#endif

    // create the primary path
    tgPSPath* path  = new tgPSPath( insideFaceH->outer_ccb(), false );
    
    std::list<tgPSPath *> paths;
    paths.push_front( path );
    
    traversePaths( paths, arrFaces );
    
    SG_LOG(SG_GENERAL, SG_ALERT, "tgPolygonSet::facesFromUntrustedNodes - found " << paths.size() << "paths" );
}

void tgPolygonSet::traversePaths( std::list<tgPSPath *>& paths, const std::vector<tgPSFaceID>& faces )
{
    std::vector<cgalPoly_HeConstHandle> visitedHes;
    tgPSPath*                           curPath = NULL;

    do {
        // find an unfinished path, and traverse until completed, or we hit a junction
        std::list<tgPSPath*>::iterator pit = paths.begin();
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
            char datasetname[128];
            
            // open a datasource for the path debug
            sprintf(datasetname, "./Paths/path_%03lu", curPath->id );
            GDALDataset* poDS = openDatasource(datasetname);
            
            // open Point layer
            OGRLayer* poNodeLayer = openLayer(poDS, wkbPoint25D, "nodes");
            
            // save start node
            toShapefile( poNodeLayer, curHe->source()->point(), "start" );
            
            if ( curPath->nodes.size() ) {
                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Continuing path " << curPath->id << " at " << curHe->source()->point() );
            } else {
                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Start path " << curPath->id << " at " << curHe->source()->point() );
            }

            do
            {
                // ignore inner antenna
                if ( curHe->face() != curHe->twin()->face() ) { 
                    curPath->nodes.push_back( curHe->source()->point() );
                    
                    // check for junction
                    // NOTE: if we contiue working on a path, it's always the case that we are at a junction.just pick the next route
 
                    if ( (curHe->source()->degree() > 2) ) {
                        SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Path " << curPath->id << " found a junction at " << curHe->source()->point() );
                        
                        // 1.  find all valid HEs
                        // ( same inner or outer face, and is NOT in visited list )
                        std::vector<tgPSPathJunction> validPaths;
                        
                        // curculate this vertex, and add new paths for all HEs with it as their source ( except our curHE )
                        // the circulator returns hes with vertex as target - look at the twins...
                        cgalPoly_Arrangement::Halfedge_around_vertex_const_circulator vCurCirc = curHe->source()->incident_halfedges();
                        cgalPoly_Arrangement::Halfedge_around_vertex_const_circulator vEndCirc = vCurCirc;
                        
                        do {
                            // get the twin halfedge ( leaving the vertex )
                            cgalPoly_HeConstHandle curChildHe = vCurCirc->twin();
                            if ( !isVisited( curChildHe, visitedHes ) ) {
                                if ( curHe->face() == curChildHe->face() && curHe->twin()->face() == curChildHe->twin()->face() ) {
                                    // same inner and outer face
                                    setVisited( curChildHe, visitedHes );
                                    validPaths.push_back( tgPSPathJunction(curChildHe, tgPSPathJunction::SAME_IN_AND_OUT) );

                                    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has valid path to " << curChildHe->target()->point() << " with same inside and outside faces" );
                                } else if ( curHe->face() == curChildHe->face() ) {
                                    // same inner face
                                    setVisited( curChildHe, visitedHes );
                                    validPaths.push_back( tgPSPathJunction(curChildHe, tgPSPathJunction::SAME_IN) );

                                    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has valid path to " << curChildHe->target()->point() << " with same inside face" );
                                } else if ( curHe->twin()->face() == curChildHe->twin()->face() ) {
                                    // same outer face
                                    setVisited( curChildHe, visitedHes );
                                    validPaths.push_back( tgPSPathJunction(curChildHe, tgPSPathJunction::SAME_OUT) );
                                    
                                    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has valid path to " << curChildHe->target()->point() << " with same outside face" );
                                }
                            } else {
                                SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Path " << curPath->id << " junction has path to already visited halfedge to " << curChildHe->target()->point() );
                            }

                            vCurCirc++;
                        } while ( vCurCirc != vEndCirc );
                        
                        // get the highest priority path
                        // sort?
                        
                        tgPSPathJunction::pathPriority_e highestPri = tgPSPathJunction::UNKNOWN;
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
                                
                                paths.push_back( new tgPSPath( validPaths[i].startCcb, isHole) );
                            }
                        }
                        
                        // now continue past junction with highest priority
                        for ( unsigned int i=0; i<validPaths.size(); i++ ) {
                            if ( validPaths[i].priority == highestPri ) {
                                curCcb = validPaths[i].startCcb;
                                break;
                            }
                        }
                    }
                } else {
                    SG_LOG(SG_GENERAL, SG_INFO, "tgPolygonSet::traversePaths Path " << curPath->id << " - drop antenna node " );
                }

                // add the node
                curPath->nodes.push_back( curHe->source()->point() );
                char nodelabel[32];
                sprintf( nodelabel, "node_%04lu", curPath->nodes.size() );
                toShapefile( poNodeLayer, curHe->source()->point(), nodelabel );
                
                ++curCcb;
                curHe = curCcb;
                if ( curHe->source() == curPath->endVertex ) {
                    curPath->complete = true;
                }
            } while ( !curPath->complete );
            
            GDALClose( poDS );
        }
    } while( curPath != NULL );
}

void tgPolygonSet::setVisited(cgalPoly_HeConstHandle he, std::vector<cgalPoly_HeConstHandle>& visited)
{
    visited.push_back( he );
}

bool tgPolygonSet::isVisited(cgalPoly_HeConstHandle he, std::vector<cgalPoly_HeConstHandle>& visited)
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

void tgPolygonSet::findIntersections( const cgalPoly_PolygonWithHoles& pwh, const cgalPoly_Line& line, std::vector<cgalPoly_Point>& intersections ) const
{
    // find the intersection of all segments and sorth them from bottom to top.
    cgalPoly_Polygon                                  p  = pwh.outer_boundary();
    cgalPoly_PolygonWithHoles::Hole_const_iterator    hit;
    std::vector<cgalPoly_Segment>                     segs;

    polygonToSegmentList( p, segs );
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        polygonToSegmentList( *hit, segs );
    }
    
    for ( unsigned int i=0; i<segs.size(); i++ ) {
        CGAL::Object result = CGAL::intersection(line, segs[i]);
        if (const cgalPoly_Point *ipoint = CGAL::object_cast<cgalPoly_Point>(&result)) {
            intersections.push_back( *ipoint );
        }
    }
    std::sort( intersections.begin(), intersections.end() );    
}

static bool sortDeltaAndPosDescending(boost::tuple<cgalPoly_Kernel::RT, cgalPoly_Kernel::RT> i, boost::tuple<cgalPoly_Kernel::RT, cgalPoly_Kernel::RT> j ) 
{ 
    // sort from largest to smallest
    return (  boost::get<0>(i) > boost::get<0>(j) ); 
}

cgalPoly_Point tgPolygonSet::getInteriorPoint( const cgalPoly_PolygonWithHoles& pwh ) const
{
    std::vector<cgalPoly_Kernel::RT>                                        xcoords;
    std::vector< boost::tuple<cgalPoly_Kernel::RT, cgalPoly_Kernel::RT> >   xbest;
    cgalPoly_Point  max_pos;
    
    // find the largest delta in x
    cgalPoly_Polygon  p  = pwh.outer_boundary();
    CGAL::Bbox_2      bb = p.bbox();

    cgalPoly_PolygonWithHoles::Hole_const_iterator    hit;
    cgalPoly_Polygon::Vertex_const_iterator           vit;
    for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
        xcoords.push_back( vit->x() );
    }
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        for (vit = hit->vertices_begin(); vit != hit->vertices_end(); ++vit) {
            xcoords.push_back( vit->x() );
        }
    }
    std::sort( xcoords.begin(), xcoords.end() );

    for (unsigned int i=0; i<xcoords.size()-1; i++) {
        cgalPoly_Kernel::RT delta = xcoords[i+1]-xcoords[i];
        xbest.push_back( boost::make_tuple( delta, xcoords[i]+delta/2 ) );
    }
    std::sort( xbest.begin(), xbest.end(), sortDeltaAndPosDescending );
    
    // create a vertical line at the midpoint of the largest delta
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::getInteriorPoint: got " << xbest.size() << " x-coords " );
    for ( unsigned int i=0; i<xbest.size(); i++ ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "GetInteriorPoint: i " << i << " width " << xbest[i].get<0>() << " location " << xbest[i].get<1>() );       
    }
    
    for ( unsigned int i=0; i<xbest.size(); i++ ) {
        cgalPoly_Line line( cgalPoly_Point( xbest[i].get<1>(), bb.ymin() ), cgalPoly_Point(xbest[i].get<1>(), bb.ymax()) );
        // get and sort the intersections with all segments of the pwh and this line
        
        std::vector<cgalPoly_Point> intersections;
        findIntersections( pwh, line, intersections );
        // from 0-1 IN face, 1-2 OUT of face, 2-3 IN face, etccc.
        // we want the biggest delta between 0,1 2,3 4,5, etc, and the midpoint of the biggest.

        SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::getInteriorPoint: got " << intersections.size() << " intersections for x-coord " << i );
        cgalPoly_Kernel::RT max_delta = 0.0;
        for ( unsigned int i=0; i<intersections.size(); i+=2 ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::getInteriorPoint: test if (" << intersections[i+1].y() << " - " << intersections[i].y() << ") > " << max_delta ); 
            
            if ( intersections[i+1].y() - intersections[i].y() > max_delta ) {                
                max_delta = intersections[i+1].y()-intersections[i].y();
                max_pos   = cgalPoly_Point( intersections[i].x(), intersections[i].y()+max_delta/2 );
                
                SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::getInteriorPoint: yes - new pos is " << max_pos );
            }
        }
        
        if ( max_delta > 0.000001 ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::getInteriorPoint: " << max_pos << " is good enough - we're done " );
            break;
        }
    }

    return max_pos;
}

const std::vector<cgalPoly_Point>& tgPolygonSet::getInteriorPoints( void ) const
{
    return interiorPoints;
}

void tgPolygonSet::calcInteriorPoints( void )
{
    std::list<cgalPoly_PolygonWithHoles>                 pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator it;

    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::getInteriorPoints: got " << pwh_list.size() << " polys with holes ");
    
    // get an interior point for each poly with holes
    interiorPoints.clear();
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        interiorPoints.push_back( getInteriorPoint( (*it) ) );
    }
}

// intersect and modify current tgPolygonSet
void tgPolygonSet::intersection2( const cgalPoly_Polygon& other )
{    
    ps.intersection( other );    
}

// intersect and return a new tgPolygonSet
tgPolygonSet tgPolygonSet::intersection( const cgalPoly_Polygon& other ) const
{
    // copy the geometry;
    cgalPoly_PolygonSet result = getPs();

    result.intersection( other );
    
    // create a new polygonSet
    return tgPolygonSet( result, getMeta() );
}

void tgPolygonSet::difference( const cgalPoly_Polygon& other )
{    
    ps.intersection( other );    
}

void tgPolygonSet::join( const cgalPoly_Polygon& other )
{    
    ps.join( other );    
}

tgPolygonSet tgPolygonSet::join( const tgPolygonSetList& sets, const tgPolygonSetMeta& m )
{
    tgPolygonSetList::const_iterator it;
    cgalPoly_PolygonSet              result;
    
    for ( it = sets.begin(); it != sets.end(); it++ ) {
        result.join( it->getPs() );
    }
    
    return tgPolygonSet( result, m );
}

tgPolygonSet tgPolygonSet::symmetricDifference( const tgPolygonSet& a, const tgPolygonSet& b, const tgPolygonSetMeta& m )
{
    cgalPoly_PolygonSet result;
    
    result.symmetric_difference( a.getPs(), b.getPs() );
    
    return tgPolygonSet( result, m );
}

CGAL::Bbox_2 tgPolygonSet::getBoundingBox( void ) const
{
    std::list<cgalPoly_PolygonWithHoles> pwh_list;    
    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    
    return CGAL::bbox_2( pwh_list.begin(), pwh_list.end() );
}

// where to put these...
double DirectionToHeading( cgalPoly_Direction dir )
{
    double angle = SGMiscd::rad2deg( atan2( CGAL::to_double(dir.dy()), CGAL::to_double(dir.dx()) ) );

    return SGMiscd::normalizePeriodic( 0, 360, -(angle-90) );   
}

cgalPoly_Transformation CreateGeodesyTranslation( const cgalPoly_Point& src, const CGAL::Vector_2<cgalPoly_Kernel>& dir, double offset )
{
    // get the direction of this vector in degrees
    double h = DirectionToHeading( dir.direction() );

    // use simgear Geodesy to get the second point
    SGGeod gsrc = SGGeod::fromDeg( CGAL::to_double( src.x() ), CGAL::to_double( src.y() ) );
    SGGeod gdst = SGGeodesy::direct( gsrc, h, offset );
    
    cgalPoly_Point dst = cgalPoly_Point( gdst.getLongitudeDeg(), gdst.getLatitudeDeg() );
    CGAL::Vector_2<cgalPoly_Kernel> direct = CGAL::Vector_2<cgalPoly_Kernel>(src, dst);
    
    // create a transformation to translate middle point 
    return cgalPoly_Transformation(CGAL::TRANSLATION, direct);
}

SGGeod OffsetPointMiddle( const cgalPoly_Point& pPrev, const cgalPoly_Point& pCur, const cgalPoly_Point& pNext, double offset_by )
{
    // Generate two unit vectors from middle to prev, and middle to next
    CGAL::Vector_2<cgalPoly_Kernel> vecPrev;
    vecPrev = CGAL::Vector_2<cgalPoly_Kernel>( pCur, pPrev );
    vecPrev = vecPrev / sqrt( CGAL::to_double( vecPrev.squared_length() ) );
    
    CGAL::Vector_2<cgalPoly_Kernel> vecNext;
    vecNext = CGAL::Vector_2<cgalPoly_Kernel>( pCur, pNext );
    vecNext = vecNext / sqrt( CGAL::to_double( vecNext.squared_length() ) );
    
    CGAL::Vector_2<cgalPoly_Kernel> vecAvg;
    if ( CGAL::right_turn( pPrev, pCur, pNext ) ) {
        vecAvg = vecPrev + vecNext;
    } else {
        vecAvg = -(vecPrev + vecNext);
    }
    
    // create a translation along vecAvg for offset_by meters
    cgalPoly_Transformation translate = CreateGeodesyTranslation( pCur, vecAvg, offset_by );
    cgalPoly_Point p = translate( pCur );
    
    return SGGeod::fromDeg( CGAL::to_double( p.x() ), CGAL::to_double( p.y() ) );
}

SGGeod OffsetPointFirst( const cgalPoly_Point& pCur, const cgalPoly_Point& pNext, double offset_by )
{
    // Generate vector from cur to next
    CGAL::Vector_2<cgalPoly_Kernel> vecNext;
    vecNext = CGAL::Vector_2<cgalPoly_Kernel>( pCur, pNext );
    
    // create perp to the right
    CGAL::Vector_2<cgalPoly_Kernel> vecRight = vecNext.perpendicular( CGAL::CLOCKWISE );
    
    // create a translation along vecRight for offset_by meters
    cgalPoly_Transformation translate = CreateGeodesyTranslation( pCur, vecRight, offset_by );
    cgalPoly_Point p = translate( pCur );
    
    return SGGeod::fromDeg( CGAL::to_double( p.x() ), CGAL::to_double( p.y() ) );    
}

SGGeod OffsetPointLast( const cgalPoly_Point& pPrev, const cgalPoly_Point& pCur, double offset_by )
{
    // Generate vector from prev to cur
    CGAL::Vector_2<cgalPoly_Kernel> vecCur;
    vecCur = CGAL::Vector_2<cgalPoly_Kernel>( pPrev, pCur );
    
    // create perp to the right
    CGAL::Vector_2<cgalPoly_Kernel> vecRight = vecCur.perpendicular( CGAL::CLOCKWISE );
    
    // create a translation along vecRight for offset_by meters
    cgalPoly_Transformation translate = CreateGeodesyTranslation( pCur, vecRight, offset_by );
    cgalPoly_Point p = translate( pCur );
    
    return SGGeod::fromDeg( CGAL::to_double( p.x() ), CGAL::to_double( p.y() ) );        
}

unsigned long tgPSPath::cur_id = 1;

tgPSPath::tgPSPath( cgalPoly_CcbHeConstCirculator sccb, bool isHole ) : id(tgPSPath::cur_id++)
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

void tgPSPath::toShapefile( const char* ds )
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

void tgPolygonSet::identifyFaces( const cgalPoly_Arrangement& arr, std::vector<tgPSFaceID>& faces )
{
    
}

void tgPolygonSet::dumpFaces( std::vector<tgPSFaceID>& faces )
{
    
}
