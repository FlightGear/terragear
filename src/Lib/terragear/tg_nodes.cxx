#include <simgear/debug/logstream.hxx>

#include "tg_nodes.hxx"
#include "tg_shapefile.hxx"

const double fgPoint3_Epsilon = 0.000001;

// The spacial search utilizes the boost tuple construct.
// The k-d tree is generated in two dimensions. and the first element of the tuple is this 2d point
// the second element of the tuple is the elevation of this point
// Three dimensional queries is a bit overkill, but the code, although faster, is slightly more cumbersome

// Build the k-d tree
void TGNodes::init_spacial_query( void )
{
#if 0    
    tg_kd_tree.clear();

    for(unsigned int i = 0; i < tg_node_list.size(); i++) {
        // generate the tuple
        tgn_Point pt( tg_node_list[i].GetPosition().getLongitudeDeg(), tg_node_list[i].GetPosition().getLatitudeDeg() );
        double    e( tg_node_list[i].GetPosition().getElevationM() );
        Point_and_Elevation pande( pt, e, &tg_node_list[i] );

        // and insert into tree
        tg_kd_tree.insert( pande );
    }

    kd_tree_valid = true;
#endif    
}

// Spacial Queries using CGAL and boost tuple

// This query finds all nodes within the bounding box
bool TGNodes::get_geod_inside( const SGGeod& min, const SGGeod& max, std::vector<SGGeod>& points ) const {
    points.clear();

#if 0
    // Have we generated the k-d tree?
    if ( !kd_tree_valid ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "get_geod_inside called with invalid kdtree" );
        exit(0);
        return false;
    }
#endif

    // define an exact rectangulat range query  (fuzziness=0)
    TGNodePoint     ll( min.getLongitudeDeg() - fgPoint3_Epsilon, min.getLatitudeDeg() - fgPoint3_Epsilon );
    TGNodePoint     ur( max.getLongitudeDeg() + fgPoint3_Epsilon, max.getLatitudeDeg() + fgPoint3_Epsilon );
    TGNodeFuzzyBox  exact_bb(ll, ur);

    // list of tuples as a result
    std::list<TGNodeData> result;
    std::list<TGNodeData>::iterator it;

    // perform the query
    tg_kd_tree.search(std::back_inserter( result ), exact_bb);

    // and convert the tuples back into SGGeod
    for ( it = result.begin(); it != result.end(); it++ ) {
        points.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    return true;
}

bool TGNodes::get_nodes_inside( const SGGeod& min, const SGGeod& max, std::vector<TGNode*>& points ) const {
    points.clear();
    
#if 0    
    // Have we generated the k-d tree?
    if ( !kd_tree_valid ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "get_nodes_inside called with invalid kdtree" );
        exit(0);
        return false;
    }
#endif

    // define an exact rectangulat range query  (fuzziness=0)
    TGNodePoint     ll( min.getLongitudeDeg() - fgPoint3_Epsilon, min.getLatitudeDeg() - fgPoint3_Epsilon );
    TGNodePoint     ur( max.getLongitudeDeg() + fgPoint3_Epsilon, max.getLatitudeDeg() + fgPoint3_Epsilon );
    TGNodeFuzzyBox  exact_bb(ll, ur);
    
    // list of tuples as a result
    std::list<TGNodeData> result;
    std::list<TGNodeData>::iterator it;
    
    // perform the query
    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    
    // and convert the tuples back into SGGeod
    for ( it = result.begin(); it != result.end(); it++ ) {
        points.push_back( boost::get<3>(*it) );
    }
    
    return true;
}

// This query finds all nodes along the tile borders (north, south, east and west)
bool TGNodes::get_geod_edge( const SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west ) const {
    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();

    TGNodePoint     ll;
    TGNodePoint     ur;
    TGNodeFuzzyBox  exact_bb;

    std::list<TGNodeData> result;
    std::list<TGNodeData>::iterator it;

    north.clear();
    south.clear();
    east.clear();
    west.clear();

#if 0    
    // Have we generated the k-d tree?
    if ( !kd_tree_valid ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "get_geod_edge called with invalid kdtree" );
        exit(0);
        return false;
    }
#endif

    // find northern points
    ll = TGNodePoint( west_compare - fgPoint3_Epsilon, north_compare - fgPoint3_Epsilon );
    ur = TGNodePoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = TGNodeFuzzyBox(ll, ur);
    
    result.clear();
    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        north.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    // find southern points
    ll = TGNodePoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = TGNodePoint( east_compare + fgPoint3_Epsilon, south_compare + fgPoint3_Epsilon );
    exact_bb = TGNodeFuzzyBox(ll, ur);
    result.clear();

    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        south.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    // find eastern points
    ll = TGNodePoint( east_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = TGNodePoint( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = TGNodeFuzzyBox(ll, ur);
    result.clear();

    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        east.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    // find western points
    ll = TGNodePoint( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = TGNodePoint( west_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = TGNodeFuzzyBox(ll, ur);
    result.clear();

    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        west.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    return true;
}

void TGNodes::get_geod_nodes( std::vector<SGGeod>& points  ) const {
    points.clear();
    for ( unsigned int i = 0; i < tg_node_list.size(); i++ ) {
        points.push_back( tg_node_list[i].GetPosition() );
    }
}

void TGNodes::get_wgs84_nodes( std::vector<SGVec3d>& points ) const {
    points.clear();
    for ( unsigned int i = 0; i < tg_node_list.size(); i++ ) {
        points.push_back( tg_node_list[i].GetWgs84() );
    }
}

void TGNodes::DeleteUnused( void ) {
    // copy the old list to a new list
    std::vector<TGNode> used_nodes;
    unsigned int total_nodes = tg_node_list.size();
    
    for(unsigned int i = 0; i < tg_node_list.size(); i++) {
        if ( tg_node_list[i].IsUsed() ) {
            used_nodes.push_back( tg_node_list[i] );
        }
    }
    
    // rebuild and reindex the tree
    tg_node_list.clear();
    tg_kd_tree.clear();
    
    for(unsigned int i = 0; i < used_nodes.size(); i++) {
        unique_add(used_nodes[i].GetPosition(), used_nodes[i].GetType() );
    }
    
    unsigned int kept_nodes = tg_node_list.size();

    SG_LOG(SG_GENERAL, SG_ALERT, "TGNodes::DeleteUnused total_nodes " << total_nodes << " kept nodes " << kept_nodes );
}

void TGNodes::CalcElevations( tgNodeType type ) {
    for(unsigned int i = 0; i < tg_node_list.size(); i++) {
        if ( tg_node_list[i].GetType() == type ) {
            SGGeod pos = tg_node_list[i].GetPosition();

            switch (type)
            {
                case TG_NODE_FIXED_ELEVATION:
                    // invalid - just ignore
                    break;

                case TG_NODE_INTERPOLATED:
                    // get elevation from array
                    SetElevation( i, array->altitude_from_grid(pos.getLongitudeDeg() * 3600.0, pos.getLatitudeDeg() * 3600.0) );
                    break;

                case TG_NODE_SMOOTHED:
                    // get elevation from smoothing function
                    break;

                case TG_NODE_DRAPED:
                    // get elevation from triangle list
                    break;
            }
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "CalcElevations (interpolated) Ignore pos " << tg_node_list[i].GetPosition() << " with type " << tg_node_list[i].GetType() );
        }
    }
}
    
void TGNodes::CalcElevations( tgNodeType type, const tgSurface& surf ) {
    for(unsigned int i = 0; i < tg_node_list.size(); i++) {
        if ( tg_node_list[i].GetType() == type ) {
            SGGeod pos = tg_node_list[i].GetPosition();

            switch (type)
            {
                case TG_NODE_FIXED_ELEVATION:
                case TG_NODE_INTERPOLATED:
                case TG_NODE_DRAPED:
                    break;

                case TG_NODE_SMOOTHED:
                    SetElevation( i, surf.query( pos ) );
                    break;
            }
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "CalcElevations smoothed Ignore pos " << tg_node_list[i].GetPosition() << " with type " << tg_node_list[i].GetType() );
        }        
    }
}

void TGNodes::CalcElevations( tgNodeType type, const tgtriangle_list& mesh ) {
    for(unsigned int i = 0; i < tg_node_list.size(); i++) {
        if ( tg_node_list[i].GetType() == type ) {
            SGGeod pos = tg_node_list[i].GetPosition();
            bool foundElev = false;
            
            switch (type)
            {
                case TG_NODE_FIXED_ELEVATION:
                case TG_NODE_INTERPOLATED:
                case TG_NODE_SMOOTHED:
                    break;
                    
                case TG_NODE_DRAPED:
                    // we need to find the triangle this node is within
                    for ( unsigned int tri=0; tri<mesh.size() && !foundElev; tri++ ) {
                        foundElev = mesh[tri].InterpolateHeight( pos );
                        if ( foundElev )
                        {
                            tg_node_list[i].SetElevation( pos.getElevationM() + 0.01f );
                        }
                    }
                    
                    if (!foundElev) {
                        SG_LOG(SG_GENERAL, SG_ALERT, "CalcElevations Could not drape point " << pos );
                    }
                    break;
            }
        }  else {
            SG_LOG(SG_GENERAL, SG_ALERT, "CalcElevations draped Ignore pos " << tg_node_list[i].GetPosition() << " with type " << tg_node_list[i].GetType() );
        }
    }
}

void TGNodes::get_normals( std::vector<SGVec3f>& normals ) const {
    normals.clear();
    for ( unsigned int i = 0; i < tg_node_list.size(); i++ ) {
        normals.push_back( tg_node_list[i].GetNormal() );
    }
}

void TGNodes::Dump( void ) {
    for (unsigned int i=0; i<tg_node_list.size(); i++) {
        TGNode const& node = tg_node_list[ i ];
        std::string fixed;

        if ( node.IsFixedElevation() ) {
            fixed = " z is fixed elevation ";
        } else {
            fixed = " z is interpolated elevation ";
        }

        SG_LOG(SG_GENERAL, SG_ALERT, "Point[" << i << "] is " << node.GetPosition() << fixed );
    }
}

void TGNodes::ToShapefile( const std::string& datasource )
{
    std::vector<SGGeod> fixed_nodes;
    std::vector<SGGeod> interpolated_nodes;
    std::vector<SGGeod> draped_nodes;
    std::vector<SGGeod> smoothed_nodes;
    
    for (unsigned int i=0; i<tg_node_list.size(); i++) {
        switch( tg_node_list[ i ].GetType() ) {
            case TG_NODE_FIXED_ELEVATION:
                fixed_nodes.push_back( tg_node_list[ i ].GetPosition() );
                break;

            case TG_NODE_INTERPOLATED:
                interpolated_nodes.push_back( tg_node_list[ i ].GetPosition() );
                if ( interpolated_nodes.size() == 1633 ) {
                    tgShapefile::FromGeod( tg_node_list[ i ].GetPosition(), "./", "node1633", "interpolated" );
                    SG_LOG(SG_GENERAL, SG_ALERT, "interpolated node 1633 has " << tg_node_list[i].GetFaces().size() );
                }
                break;

            case TG_NODE_DRAPED:
                draped_nodes.push_back( tg_node_list[ i ].GetPosition() );
                break;

            case TG_NODE_SMOOTHED:
                smoothed_nodes.push_back( tg_node_list[ i ].GetPosition() );
                break;                
        }
    }
    
    tgShapefile::FromGeodList( fixed_nodes, false, datasource, "fixed nodes", "fixed" );
    tgShapefile::FromGeodList( interpolated_nodes, false, datasource, "interpolated nodes", "interpolated" );
    tgShapefile::FromGeodList( draped_nodes, false, datasource, "draped nodes", "draped" );
    tgShapefile::FromGeodList( smoothed_nodes, false, datasource, "smoothed nodes", "smoothed" );
}

void TGNodes::SaveToGzFile( gzFile& fp )
{
    // Just save the node_list - rebuild the kd_tree on load
    sgWriteUInt( fp, tg_node_list.size() );
    for (unsigned int i=0; i<tg_node_list.size(); i++) {
        tg_node_list[i].SaveToGzFile( fp );
    }    
}

void TGNodes::LoadFromGzFile( gzFile& fp )
{
    unsigned int count;
    sgReadUInt( fp, &count );
    for (unsigned int i=0; i<count; i++) {
        TGNode node;
        node.LoadFromGzFile( fp );
        unique_add(node.GetPosition(), node.GetType() );
    }
}