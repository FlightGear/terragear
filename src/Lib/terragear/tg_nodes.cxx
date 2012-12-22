#include <simgear/debug/logstream.hxx>
#include "tg_nodes.hxx"

const double fgPoint3_Epsilon = 0.000001;

#define USE_SPACIAL_QUERY

#ifndef USE_SPACIAL_QUERY

void TGNodes::init_spacial_query( void )
{
    kd_tree_valid = true;
}

static bool IsWithin( const SGGeod pt, double xmin, double xmax, double ymin, double ymax )
{
    return ( (xmin <= pt.getLongitudeDeg()) && (ymin <= pt.getLatitudeDeg()) &&
             (xmax >= pt.getLongitudeDeg()) && (ymax >= pt.getLatitudeDeg()) );
}

static bool IsAlmostWithin( const SGGeod pt, const SGGeod& min, const SGGeod& max )
{
    // make sure we take epsilon into account
    return ( IsWithin(pt,
                      min.getLongitudeDeg() - fgPoint3_Epsilon,
                      max.getLongitudeDeg() + fgPoint3_Epsilon,
                      min.getLatitudeDeg()  - fgPoint3_Epsilon,
                      max.getLatitudeDeg()  + fgPoint3_Epsilon ) );
}

bool TGNodes::get_geod_inside( const SGGeod& min, const SGGeod& max, std::vector<SGGeod>& points ) const {
    points.clear();
    for ( unsigned int i = 0; i < tg_node_list.size(); i++ ) {
        SGGeod const& pt = tg_node_list[i].GetPosition();

        if ( IsAlmostWithin( pt, min, max ) ) {
            points.push_back( pt );
        }
    }

    return true;
}

bool TGNodes::get_geod_edge( const SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west ) const {
    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();

    north.clear();
    south.clear();
    east.clear();
    west.clear();

    for ( unsigned int i = 0; i < tg_node_list.size(); i++ ) {
        SGGeod const& pt = tg_node_list[i].GetPosition();

        // may save the same point twice - so we get all the corners
        if ( fabs(pt.getLatitudeDeg() - north_compare) < SG_EPSILON) {
            north.push_back( pt );
        }
        if ( fabs(pt.getLatitudeDeg() - south_compare) < SG_EPSILON) {
            south.push_back( pt );
        }
        if ( fabs(pt.getLongitudeDeg() - east_compare) < SG_EPSILON) {
            east.push_back( pt );
        }
        if ( fabs(pt.getLongitudeDeg() - west_compare) < SG_EPSILON) {
            west.push_back( pt );
        }
    }

    return true;
}

#else

// The spacial search utilizes the boost tuple construct.
// The k-d tree is generated in two dimensions. and the first element of the tuple is this 2d point
// the second element of the tuple is the elevation of this point
// Three dimensional queries is a bit overkill, but the code, although faster, is slightly more cumbersome

// get function for the property map - needed for cgal trait extension
My_point_property_map::reference get(My_point_property_map, My_point_property_map::key_type p) {
    return boost::get<0>(p);
}

// Build the k-d tree
void TGNodes::init_spacial_query( void )
{
    tg_kd_tree.clear();

    for(unsigned int i = 0; i < tg_node_list.size(); i++) {
        // generate the tuple
        Point   pt( tg_node_list[i].GetPosition().getLongitudeDeg(), tg_node_list[i].GetPosition().getLatitudeDeg() );
        double  e( tg_node_list[i].GetPosition().getElevationM() );
        Point_and_Elevation pande(pt, e);

        // and insert into tree
        tg_kd_tree.insert( pande );
    }

    kd_tree_valid = true;
}

// Spacial Queries using CGAL and boost tuple

// This query finds all nodes within the bounding box
bool TGNodes::get_geod_inside( const SGGeod& min, const SGGeod& max, std::vector<SGGeod>& points ) const {
    points.clear();

    // Have we generated the k-d tree?
    if ( !kd_tree_valid ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "get_geod_inside called with invalid kdtree" );
        exit(0);
        return false;
    }

    // define an exact rectangulat range query  (fuzziness=0)
    Point ll( min.getLongitudeDeg() - fgPoint3_Epsilon, min.getLatitudeDeg() - fgPoint3_Epsilon );
    Point ur( max.getLongitudeDeg() + fgPoint3_Epsilon, max.getLatitudeDeg() + fgPoint3_Epsilon );
    Fuzzy_bb exact_bb(ll, ur);

    // list of tuples as a result
    std::list<Point_and_Elevation> result;
    std::list<Point_and_Elevation>::iterator it;

    // perform the query
    tg_kd_tree.search(std::back_inserter( result ), exact_bb);

    // and convert the tuples back into SGGeod
    for ( it = result.begin(); it != result.end(); it++ ) {
        points.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    return true;
}

// This query finds all nodes along the tile borders (north, south, east and west)
bool TGNodes::get_geod_edge( const SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west ) const {
    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();

    Point ll;
    Point ur;
    Fuzzy_bb exact_bb;

    std::list<Point_and_Elevation> result;
    std::list<Point_and_Elevation>::iterator it;

    north.clear();
    south.clear();
    east.clear();
    west.clear();

    // Have we generated the k-d tree?
    if ( !kd_tree_valid ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "get_geod_edge called with invalid kdtree" );
        exit(0);
        return false;
    }

    // find northern points
    ll = Point( west_compare - fgPoint3_Epsilon, north_compare - fgPoint3_Epsilon );
    ur = Point( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = Fuzzy_bb(ll, ur);
    result.clear();
    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        north.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    // find southern points
    ll = Point( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = Point( east_compare + fgPoint3_Epsilon, south_compare + fgPoint3_Epsilon );
    exact_bb = Fuzzy_bb(ll, ur);
    result.clear();

    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        south.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    // find eastern points
    ll = Point( east_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = Point( east_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = Fuzzy_bb(ll, ur);
    result.clear();

    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        east.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    // find western points
    ll = Point( west_compare - fgPoint3_Epsilon, south_compare - fgPoint3_Epsilon );
    ur = Point( west_compare + fgPoint3_Epsilon, north_compare + fgPoint3_Epsilon );
    exact_bb = Fuzzy_bb(ll, ur);
    result.clear();

    tg_kd_tree.search(std::back_inserter( result ), exact_bb);
    for ( it = result.begin(); it != result.end(); it++ ) {
        west.push_back( SGGeod::fromDegM( boost::get<0>(*it).x(), boost::get<0>(*it).y(), boost::get<1>(*it) ) );
    }

    return true;
}

#endif

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

        if ( node.GetFixedPosition() ) {
            fixed = " z is fixed elevation ";
        } else {
            fixed = " z is interpolated elevation ";
        }

        SG_LOG(SG_GENERAL, SG_ALERT, "Point[" << i << "] is " << node.GetPosition() << fixed );
    }
}

void TGNodes::SaveToGzFile( gzFile& fp )
{
    tg_node_list.SaveToGzFile( fp );
}

void TGNodes::LoadFromGzFile( gzFile& fp )
{
    tg_node_list.LoadFromGzFile( fp );
}