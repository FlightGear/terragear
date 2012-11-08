#include <simgear/debug/logstream.hxx>
#include <CGAL/Plane_3.h>

#include "tg_nodes.hxx"

// compare node's positions (x, then y)
int compare_position(const TGNode& n1, const TGNode& n2)
{
    Point3D pos1 = n1.GetPosition();
    Point3D pos2 = n2.GetPosition();

    if ( pos1.x() == pos2.x() ) {
        return ( pos1.y() < pos2.y() );
    } else {
        return ( pos1.x() < pos2.x() );
    }
}

int fuzzy_compare_xposition(const TGNode& n1, const TGNode& n2)
{
    Point3D pos1 = n1.GetPosition();
    Point3D pos2 = n2.GetPosition();

    if ( fabs(pos1.x() - pos2.x()) < FG_PROXIMITY_EPSILON ) {
        /* if x coords are within vacinity, then pos1 < pos2 */
        return 1; 
    } else {
        return compare_position( n1, n2 );
    }
}

void TGNodes::SortNodes( void )
{
    std::sort(tg_node_list.begin(), tg_node_list.end(), compare_position);
    sorted = true;
}
// Find the index of the specified point (compair to the same
// tolerance as unique_add().  Returns -1 if not found.
int TGNodes::sorted_find( const Point3D& p ) const {
    TGNode node( p );
    const_node_list_iterator lb, ub;

    // first, find the range to search
    ub=lower_bound( tg_node_list.begin(), tg_node_list.end(), node, fuzzy_compare_xposition );
    lb=upper_bound( tg_node_list.begin(), tg_node_list.end(), node, fuzzy_compare_xposition );

    // then do a normal linear search in the range
    if ( lb != tg_node_list.end() ) {
        for ( ; lb != ub; ++lb ) {
            if ( close_enough_2d(p, lb->GetPosition()) ) {
                return std::distance( tg_node_list.begin(), lb );
            }
        }
    }

    return -1;
}

int TGNodes::linear_find( const Point3D& p ) const {
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

#if 0
void TGNodes::sorted_unique_add( const Point3D& p ) {
# if 0
    TGNode node( p );
    node.SetFixedPosition( false );

    node_list_iterator idx = std::lower_bound( tg_node_list.begin(), tg_node_list.end(), node, compare_position );
    tg_node_list.insert( idx, node );
#else
    TGNode node( p );
    node.SetFixedPosition( false );

    node_list_iterator lb, ub, idx;

    // first, find the range to search
    ub=lower_bound( tg_node_list.begin(), tg_node_list.end(), node, fuzzy_compare_xposition );
    lb=upper_bound( tg_node_list.begin(), tg_node_list.end(), node, fuzzy_compare_xposition );

    // then do a normal linear search in the range
    if ( lb != tg_node_list.end() ) {
        for ( ; lb != ub; ++lb ) {
            if ( close_enough_2d(p, lb->GetPosition()) ) {
//              return std::distance( tg_node_list.begin(), lb );
                return;
            }
        }
    }

    // we didn't find an existing node - insert new one in correct order
    idx = std::lower_bound( lb, ub, node, compare_position );
    tg_node_list.insert( idx, node );

//  return std::distance( tg_node_list.begin(), idx );
#endif
}
#endif

void TGNodes::linear_unique_add( const Point3D& p ) {
    node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        if ( close_enough_2d(p, (*current).GetPosition() ) ) {
            return;
        }

    }

    TGNode node( p );
    node.SetFixedPosition( false );

    // add to list
    tg_node_list.push_back( node );
}

#if 0
void TGNodes::sorted_unique_add_fixed_elevation( const Point3D& p ) {
# if 0
    TGNode node( p );
    node.SetFixedPosition(true);

    node_list_iterator idx = std::lower_bound( tg_node_list.begin(), tg_node_list.end(), node, compare_position );

    if ( idx != tg_node_list.end() ) {
        if ( close_enough_2d( p, idx->GetPosition() ) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "AddFixedLocation: node " << p << " exists at " << std::distance(tg_node_list.begin(), idx) );
            idx->SetPosition( p );
            idx->SetFixedPosition( true );
        } else {
            tg_node_list.insert( idx, node );
}
    }
#else
    TGNode node( p );
    node.SetFixedPosition(true);

    node_list_iterator lb, ub, idx;

    // first, find the range to search
    ub=lower_bound( tg_node_list.begin(), tg_node_list.end(), node, fuzzy_compare_xposition );
    lb=upper_bound( tg_node_list.begin(), tg_node_list.end(), node, fuzzy_compare_xposition );

    // then do a normal linear search in the range
    if ( lb != tg_node_list.end() ) {
        for ( ; lb != ub; ++lb ) {
            if ( close_enough_2d(p, lb->GetPosition()) ) {
                lb->SetPosition( p );
                lb->SetFixedPosition( true );

//              return std::distance( tg_node_list.begin(), lb );
                return;
            }
        }
    }

    // we didn't find an existing node - insert new one in correct order
    idx = std::lower_bound( lb, ub, node, compare_position );
    tg_node_list.insert( idx, node );

//  return std::distance( tg_node_list.begin(), idx );
#endif
}
#endif

void TGNodes::linear_unique_add_fixed_elevation( const Point3D& p ) {
    node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        if ( close_enough_2d(p, (*current).GetPosition() ) ) {

            // Force the match to our position, and mark as fixed
            (*current).SetPosition( p );
            (*current).SetFixedPosition( true );

            return;
        }

    }

    TGNode node( p );
    node.SetFixedPosition( true );

    // add to list
    tg_node_list.push_back( node );

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

// TODO: if the list is sorted, we should be able to get the range from x=min to x=max,
// then within that range, add each point where y is within ymin, max
// still linear search, but should be less points
point_list TGNodes::get_geod_inside( Point3D min, Point3D max ) const {
    point_list points;
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    for ( ; current != last; ++current ) {
        Point3D pt = (*current).GetPosition();

        if ( pt.IsAlmostWithin( min, max ) ) {
            points.push_back( pt );
        } else {
            if ( (pt < max) && (pt > min) ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "pt " << pt << " fails IsAlmostWithin, but sholdn't have: min " << min << " max " << max );
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

void TGNode::LoadFromGzFile(gzFile& fp)
{
    int i, nCount;

    // Load a tgnode
    sgReadPoint3D( fp, position);
    CalcWgs84();

    sgReadPoint3D( fp, normal );
    sgReadInt( fp, (int*)&fixed_position );
    sgReadInt( fp, (int*)&fixed_normal );

    sgReadInt( fp, &nCount );
    for (i=0; i<nCount; i++) {
        TGFaceLookup face;

        sgReadUInt( fp, &face.area );
        sgReadUInt( fp, &face.shape );
        sgReadUInt( fp, &face.seg );
        sgReadUInt( fp, &face.tri );

        faces.push_back( face );
    }
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

void TGNode::SaveToGzFile(gzFile& fp)
{
    int  i, nCount;

    // Save a tgnode
    sgWritePoint3D( fp, position );
    sgWritePoint3D( fp, normal );
    sgWriteInt( fp, (int)fixed_position );
    sgWriteInt( fp, (int)fixed_normal );

    nCount = faces.size();
    sgWriteInt( fp, nCount );
    for (i=0; i<nCount; i++) {
        sgWriteUInt( fp, faces[i].area );
        sgWriteUInt( fp, faces[i].shape );
        sgWriteUInt( fp, faces[i].seg );
        sgWriteUInt( fp, faces[i].tri );
    }
}

// input from stream
std::istream& operator >> ( std::istream& in, TGNodes& ns )
{
    int i, nCount;

    // Load sorted flag
    in >> ns.sorted;
    // Load all tgnodes
    in >> nCount;
    
    for (i=0; i<nCount; i++) {
        TGNode node;
        in >> node;

        ns.tg_node_list.push_back( node );
    }

    return in;
}

void TGNodes::LoadFromGzFile(gzFile& fp)
{
    int i, nCount;

    // Load sorted flag
    sgReadInt( fp, (int*)&sorted );

    // Load all tgnodes
    sgReadInt( fp, &nCount );

    for (i=0; i<nCount; i++) {
        TGNode node;
        node.LoadFromGzFile( fp );

        tg_node_list.push_back( node );
    }
}

std::ostream& operator<< ( std::ostream& out, const TGNodes& ns )
{
    int i, nCount;

    // Save sorted flag
    out << ns.sorted << "\n";
    // Save all tgnodes
    nCount = ns.tg_node_list.size();
    out << nCount << "\n";

    for (i=0; i<nCount; i++) {
        out << ns.tg_node_list[i];
    }

    return out;
}

void TGNodes::SaveToGzFile(gzFile& fp)
{
    int i, nCount;

    // Save sorted flag
    sgWriteInt( fp, (int)sorted );

    // Save all tgnodes
    nCount = tg_node_list.size();
    sgWriteInt( fp, nCount );

    for (i=0; i<nCount; i++) {
        tg_node_list[i].SaveToGzFile( fp );
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
