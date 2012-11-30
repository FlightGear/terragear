#include <simgear/debug/logstream.hxx>
#include <CGAL/Plane_3.h>

#include "tg_nodes.hxx"

// compare node's positions (x, then y)
int compare_position(const TGNode& n1, const TGNode& n2)
{
    SGGeod pos1 = n1.GetPosition();
    SGGeod pos2 = n2.GetPosition();

    if ( pos1.getLongitudeDeg() == pos2.getLongitudeDeg() ) {
        return ( pos1.getLatitudeDeg() < pos2.getLatitudeDeg() );
    } else {
        return ( pos1.getLongitudeDeg() < pos2.getLongitudeDeg() );
    }
}

int fuzzy_compare_xposition(const TGNode& n1, const TGNode& n2)
{
    SGGeod pos1 = n1.GetPosition();
    SGGeod pos2 = n2.GetPosition();

    if ( fabs(pos1.getLongitudeDeg() - pos2.getLongitudeDeg()) < FG_PROXIMITY_EPSILON ) {
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
int TGNodes::sorted_find( const SGGeod& p ) const {
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

int TGNodes::linear_find( const SGGeod& p ) const {
    const_node_list_iterator current, last;
    SGGeod pos;
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

void TGNodes::linear_unique_add( const SGGeod& p ) {
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

void TGNodes::linear_unique_add_fixed_elevation( const SGGeod& p ) {
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

void TGNodes::get_geod_nodes( std::vector<SGGeod>& points  ) const {
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    points.clear();
    for ( ; current != last; ++current ) {
        points.push_back( (*current).GetPosition() );
    }
}

// TODO: if the list is sorted, we should be able to get the range from x=min to x=max,
// then within that range, add each point where y is within ymin, max
// still linear search, but should be less points
const double fgPoint3_Epsilon = 0.000001;

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

void TGNodes::get_geod_inside( const SGGeod& min, const SGGeod& max, std::vector<SGGeod>& points ) const {
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    points.clear();
    for ( ; current != last; ++current ) {
        SGGeod pt = (*current).GetPosition();

        if ( IsAlmostWithin( pt, min, max ) ) {
            points.push_back( pt );
        }
    }
}

void TGNodes::get_geod_edge( const SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west ) const {
    const_node_list_iterator current, last;
    double north_compare = b.get_center_lat() + 0.5 * b.get_height();
    double south_compare = b.get_center_lat() - 0.5 * b.get_height();
    double east_compare  = b.get_center_lon() + 0.5 * b.get_width();
    double west_compare  = b.get_center_lon() - 0.5 * b.get_width();

    // find all points on the edges
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    north.clear();
    south.clear();
    east.clear();
    west.clear();

    for ( ; current != last; ++current ) {
        SGGeod pt = (*current).GetPosition();

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
}

void TGNodes::get_wgs84_nodes( std::vector<SGVec3d>& points ) const {
    const_node_list_iterator current, last;

    current = tg_node_list.begin();
    last    = tg_node_list.end();

    points.clear();
    for ( ; current != last; ++current ) {
        points.push_back( (*current).GetWgs84() );
    }
}

void TGNodes::get_normals( std::vector<SGVec3f>& normals ) const {
    const_node_list_iterator current, last;

    // see if point already exists
    current = tg_node_list.begin();
    last    = tg_node_list.end();

    normals.clear();
    for ( ; current != last; ++current ) {
        normals.push_back( (*current).GetNormal() );
    }
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

void TGNode::LoadFromGzFile(gzFile& fp)
{
    int i, nCount;

    // Load a tgnode
    sgReadGeod( fp, position);
    CalcWgs84();

    sgReadVec3( fp, normal );
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
    sgWriteGeod( fp, position );
    sgWriteVec3( fp, normal );
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