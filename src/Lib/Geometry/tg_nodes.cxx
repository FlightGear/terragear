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
        if ( close_enough_2d(p, (*current).GetPosition() ) ) {
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
        if ( close_enough_2d(p, (*current).GetPosition() ) ) {
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

point_list TGNodes::get_geod_inside( Point3D min, Point3D max ) const {
    point_list points;
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        Point3D pt = (*current).GetPosition();

        if ( pt.IsWithin( min, max ) ) {
            points.push_back( pt );
        } else {
            if ( (pt < max) && (pt > min) ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "pt " << pt << " failes IsWithin, but sholdn't have: min " << min << " max " << max );
            }
        }
    }

    return points;
}

std::vector< SGVec3d > TGNodes::get_wgs84_nodes_as_SGVec3d( void ) const {
    const_node_list_iterator current, last;
    std::vector< SGVec3d > points;

    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        points.push_back( (*current).GetWgs84AsSGVec3d() );
    }
        
    return points;
}


point_list TGNodes::get_wgs84_nodes_as_Point3d( void ) const {
    const_node_list_iterator current, last;
    point_list points;

    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        points.push_back( (*current).GetWgs84AsPoint3D() );
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

void TGNodes::Dump( void ) {
    for (unsigned int i=0; i<tg_node_list.size(); i++) {
        TGNode node = tg_node_list[ i ];
        std::string fixed;
        
        if ( node.GetFixedPosition() ) {
            fixed = " z is fixed elevation ";
        } else {
            fixed = " z is interpolated elevation ";
        }

        SG_LOG(SG_GENERAL, SG_ALERT, "Point[" << i << "] is " << node.GetPosition() << fixed );
        if ( node.GetFaces().size() ) {
            TGFaceList faces = node.GetFaces();
            for (unsigned int j=0; j<faces.size(); j++) {
                SG_LOG(SG_GENERAL, SG_ALERT, "\tface " << faces[j].area << "," << faces[j].shape << "," << faces[j].seg << "," << faces[j].tri );
            }
        }
    }
}

#if 0
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
#endif
