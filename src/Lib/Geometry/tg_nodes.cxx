#include <simgear/debug/logstream.hxx>

#include "tg_nodes.hxx"

// Find the index of the specified point (compair to the same
// tolerance as unique_add().  Returns -1 if not found.
int TGNodes::find( const Point3D& p ) const {
    const_node_list_iterator current, last;
    Point3D pos;
    int counter = 0;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();
    for ( ; current != last; ++current ) {
        pos = current->GetPosition();
        if ( close_enough_2d(p, pos ) ) {
            return counter;
        }

        ++counter;
    }

    return -1;
}

int TGNodes::unique_add( const Point3D& p ) {
    node_list_iterator current, last;
    int     counter = 0;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        if ( close_enough_3d(p, (*current).GetPosition() ) ) {
            // cout << "found an existing match!" << endl;
            return counter;
        }

        ++counter;
    }

    TGNode node( p );
    node.SetFixedPosition( false );

    // add to list
    tg_node_list.push_back( node );

    return counter;
}

int TGNodes::unique_add_fixed_elevation( const Point3D& p ) {
    node_list_iterator current, last;
    int     counter = 0;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        if ( close_enough_3d(p, (*current).GetPosition() ) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Adding fixed elev node : node already exists at " << counter << " old position is " << (*current).GetPosition() << " new position is " << p );

            // Force the match to our position, and mark as fixed
            (*current).SetPosition( p );
            (*current).SetFixedPosition( true );

            return counter;
        }

        ++counter;
    }

    TGNode node( p );
    node.SetFixedPosition( true );

    // add to list
    tg_node_list.push_back( node );

    return counter;
}

point_list TGNodes::get_geod_nodes( void ) const {
    point_list points;
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        points.push_back( (*current).GetPosition() );
    }
        
    return points;
}

std::vector< SGVec3d > TGNodes::get_wgs84_nodes_as_SGVec3d( void ) const {
    std::vector< SGVec3d > points;
    Point3D    pos;
    const_node_list_iterator current, last;

    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        pos = (*current).GetPosition();
    
        SGGeod  geod = SGGeod::fromDegM( pos.x(), pos.y(), pos.z() );
        SGVec3d cart = SGVec3d::fromGeod(geod);

        points.push_back( cart );
    }
        
    return points;
}

point_list TGNodes::get_wgs84_nodes_as_Point3d( void ) const {
    point_list points;
    Point3D    pos;
    const_node_list_iterator current, last;

    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        pos = (*current).GetPosition();
    
        SGGeod  geod = SGGeod::fromDegM( pos.x(), pos.y(), pos.z() );
        SGVec3d cart = SGVec3d::fromGeod(geod);

        points.push_back( Point3D::fromSGVec3( cart ) );
    }
        
    return points;
}

node_list TGNodes::get_fixed_elevation_nodes( void ) const {
    node_list fixed_elev;
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        if ( (*current).GetFixedPosition() ) {
            fixed_elev.push_back( (*current) );
        }
    }
        
    return fixed_elev;        
}

point_list TGNodes::get_normals( void ) const {
    point_list points;
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        points.push_back( (*current).GetNormal() );
    }
        
    return points;
}

bool TGNodes::LookupFixedElevation( Point3D p, double* z )
{
    int  index = find( p );
    bool found = false;

    if (index >= 0) {
        TGNode  node = tg_node_list[index];
        if ( node.GetFixedPosition() ) {
            *z = tg_node_list[index].GetPosition().z();
            found = true;
        }
    }

    return found;
}
