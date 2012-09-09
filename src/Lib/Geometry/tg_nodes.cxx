#include <simgear/debug/logstream.hxx>
#include <CGAL/Plane_3.h>

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

void TGNodes::get_geod_edge( SGBucket b, point_list& north, point_list& south, point_list& east, point_list& west ) const {
    const_node_list_iterator current, last;
    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();

    // find all points on the edges
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        Point3D pt = (*current).GetPosition();

        // may save the same point twice - so we get all the corners
        if ( fabs(pt.y() - north_compare) < SG_EPSILON) {
            north.push_back( pt );
        }
        if ( fabs(pt.y() - south_compare) < SG_EPSILON) {
            south.push_back( pt );
        }
        if ( fabs(pt.x() - east_compare) < SG_EPSILON) {
            east.push_back( pt );
        }
        if ( fabs(pt.x() - west_compare) < SG_EPSILON) {
            west.push_back( pt );
        }
    }
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

// input from stream
std::istream& operator >> ( std::istream& in, TGNode& n )
{
    int i, nCount;

    // Load a tgnode
    in >> n.position;
    n.CalcWgs84();

    in >> n.normal;
    in >> n.fixed_position;
    in >> n.fixed_normal;

    in >> nCount;
    for (i=0; i<nCount; i++) {
        TGFaceLookup face;

        in >> face.area;
        in >> face.shape;
        in >> face.seg;
        in >> face.tri;

        n.faces.push_back( face );
    }

    return in;
}

std::ostream& operator<< ( std::ostream& out, const TGNode& n )
{
    int i, nCount;

    // Save a tgnode
    out << n.position;
    out << n.normal;
    out << n.fixed_position << " ";
    out << n.fixed_normal << "\n";

    nCount = n.faces.size();
    out << nCount << "\n";
    for (i=0; i<nCount; i++) {
        out << n.faces[i].area << " ";
        out << n.faces[i].shape << " ";
        out << n.faces[i].seg << " ";
        out << n.faces[i].tri << "\n";
    }

    return out;
}

// input from stream
std::istream& operator >> ( std::istream& in, TGNodes& ns )
{
    int i, nCount;

    // Load all tgnodes
    in >> nCount;
    
    for (i=0; i<nCount; i++) {
        TGNode node;
        in >> node;

        ns.tg_node_list.push_back( node );
    }

    return in;
}

std::ostream& operator<< ( std::ostream& out, const TGNodes& ns )
{
    int i, nCount;

    // Save all tgnodes
    nCount = ns.tg_node_list.size();
    out << nCount << "\n";

    for (i=0; i<nCount; i++) {
        out << ns.tg_node_list[i];
    }

    return out;
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
