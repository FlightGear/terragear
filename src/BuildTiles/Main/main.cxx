// main.cxx -- top level construction routines
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: main.cxx,v 1.58 2005-09-28 16:40:32 curt Exp $


// TODO TODO TODO : Get rid of construct - data hiding is moretrouble than it's worth right now.
// constantly needing to call set after every operation....


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef HAVE_SYS_TIME_H
#  include <sys/time.h>		// set mem allocation limit
#endif
#ifndef _MSC_VER
#  include <sys/resource.h>	// set mem allocation limit
#  include <unistd.h>		// set mem allocation limit
#endif

#include <simgear/compiler.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_dir.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/math/sg_types.hxx>

#include <simgear/math/SGMath.hxx>
#include <simgear/misc/sgstream.hxx>


#include <boost/foreach.hpp>

#include <Geometry/poly_support.hxx>
#include <landcover/landcover.hxx>

#include "construct.hxx"
#include "usgs.hxx"

using std::string;
using std::vector;

using namespace std;

vector<string> load_dirs;
double nudge=0.0;

#if 0
// For each triangle assigned to the "default" area type, see if we
// can lookup a better land cover type from the 1km data structure.
static void fix_land_cover_assignments( TGConstruct& c ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Fixing up default land cover types");
    // the list of node locations
    TGTriNodes trinodes = c.get_tri_nodes();
    point_list geod_nodes = trinodes.get_node_list();

    // the list of triangles (with area type attribute)
    triele_list tri_elements = c.get_tri_elements();

    // traverse the triangle element groups
    SG_LOG(SG_GENERAL, SG_ALERT, "  Total Nodes = " << geod_nodes.size());
    SG_LOG(SG_GENERAL, SG_ALERT, "  Total triangles = " << tri_elements.size());
    for ( unsigned int i = 0; i < tri_elements.size(); ++i ) {
        TGTriEle t = tri_elements[i];
        if ( t.get_attribute() == get_default_area_type() ) {
            Point3D p1 = geod_nodes[t.get_n1()];
            Point3D p2 = geod_nodes[t.get_n2()];
            Point3D p3 = geod_nodes[t.get_n3()];

            // offset by -quarter_cover_size because that is what we
            // do for the coverage squares
            AreaType a1 = get_area_type( c.get_cover(),
                                         p1.x() - quarter_cover_size,
                                         p1.y() - quarter_cover_size,
                                         cover_size, cover_size );
            AreaType a2 = get_area_type( c.get_cover(),
                                         p2.x() - quarter_cover_size,
                                         p2.y() - quarter_cover_size,
                                         cover_size, cover_size );
            AreaType a3 = get_area_type( c.get_cover(),
                                         p3.x() - quarter_cover_size,
                                         p3.y() - quarter_cover_size,
                                         cover_size, cover_size );

            // update the original triangle element attribute
            AreaType new_area;

            // majority rules
            if ( a1 == a2 ) {
                new_area = a1;
            } else if ( a1 == a3 ) {
                new_area = a1;
            } else if ( a2 == a3 ) {
                new_area = a2;
            } else {
                // a different coverage for each vertex, just pick
                // from the middle/average
                Point3D average = ( p1 + p2 + p3 ) / 3.0;
                //cout << "    average triangle center = " << average;
                new_area = get_area_type( c.get_cover(), 
                                          average.x() - quarter_cover_size,
                                          average.y() - quarter_cover_size,
                                          cover_size, cover_size );
            }
              
            //cout << "  new attrib = " << get_area_name( new_area ) << endl;
            c.set_tri_attribute( i, new_area );
        }
    }
}
#endif

#if 0
// build the node -> element (triangle) reverse lookup table.  there
// is an entry for each point containing a list of all the triangles
// that share that point.
static belongs_to_list gen_node_ele_lookup_table( TGConstruct& c ) {
    belongs_to_list reverse_ele_lookup;
    reverse_ele_lookup.clear();

    int_list ele_list;
    ele_list.clear();

    // initialize reverse_ele_lookup structure by creating an empty
    // list for each point
    point_list wgs84_nodes = c.get_wgs84_nodes();

    SG_LOG(SG_GENERAL, SG_ALERT, "there are " << wgs84_nodes.size() << " wgs84 nodes" );

    const_point_list_iterator w_current = wgs84_nodes.begin();
    const_point_list_iterator w_last = wgs84_nodes.end();
    for ( ; w_current != w_last; ++w_current ) {
        reverse_ele_lookup.push_back( ele_list );
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "1 " );

    // traverse triangle structure building reverse lookup table
    triele_list tri_elements = c.get_tri_elements();
    const_triele_list_iterator current = tri_elements.begin();
    const_triele_list_iterator last = tri_elements.end();
    int counter = 0;

    SG_LOG(SG_GENERAL, SG_ALERT, "2 " );

    for ( ; current != last; ++current ) {

        //SG_LOG(SG_GENERAL, SG_ALERT, "CURRENT " << current );
//        SG_LOG(SG_GENERAL, SG_ALERT, "N1: " << current->get_n1() << " N2: " << current->get_n2() << " N3: " << current->get_n3() );
        
        reverse_ele_lookup[ current->get_n1() ].push_back( counter );
        reverse_ele_lookup[ current->get_n2() ].push_back( counter );
        reverse_ele_lookup[ current->get_n3() ].push_back( counter );
        ++counter;
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "3 " );

    return reverse_ele_lookup;
}
#endif

// caclulate the area for the specified triangle face
static double tri_ele_area( const TGConstruct& c, const TGTriEle tri ) {
    point_list nodes = c.get_geod_nodes();

    Point3D p1 = nodes[ tri.get_n1() ];
    Point3D p2 = nodes[ tri.get_n2() ];
    Point3D p3 = nodes[ tri.get_n3() ];

    return triangle_area( p1, p2, p3 );
}

#if 0
// caclulate the normal for the specified triangle face
static Point3D calc_normal( TGConstruct& c, int i ) {
    SGVec3d v1, v2, normal;

    point_list wgs84_nodes = c.get_wgs84_nodes();
    triele_list tri_elements = c.get_tri_elements();

    Point3D p1 = wgs84_nodes[ tri_elements[i].get_n1() ];
    Point3D p2 = wgs84_nodes[ tri_elements[i].get_n2() ];
    Point3D p3 = wgs84_nodes[ tri_elements[i].get_n3() ];

    // do some sanity checking.  With the introduction of landuse
    // areas, we can get some long skinny triangles that blow up our
    // "normal" calculations here.  Let's check for really small
    // triangle areas and check if one dimension of the triangle
    // coordinates is nearly coincident.  If so, assign the "default"
    // normal of straight up.

    bool degenerate = false;
    const double area_eps = 1.0e-12;
    double area = tri_ele_area( c, tri_elements[i] );
    // cout << "   area = " << area << endl;
    if ( area < area_eps ) {
        degenerate = true;
    }

    // cout << "  " << p1 << endl;
    // cout << "  " << p2 << endl;
    // cout << "  " << p3 << endl;
    if ( fabs(p1.x() - p2.x()) < SG_EPSILON && fabs(p1.x() - p3.x()) < SG_EPSILON ) {
        degenerate = true;
    }
    if ( fabs(p1.y() - p2.y()) < SG_EPSILON && fabs(p1.y() - p3.y()) < SG_EPSILON ) {
        degenerate = true;
    }
    if ( fabs(p1.z() - p2.z()) < SG_EPSILON && fabs(p1.z() - p3.z()) < SG_EPSILON ) {
        degenerate = true;
    }

    if ( degenerate ) {
        normal = normalize(SGVec3d(p1.x(), p1.y(), p1.z()));
	    SG_LOG(SG_GENERAL, SG_ALERT, "Degenerate tri!");
    } else {
    	v1[0] = p2.x() - p1.x();
    	v1[1] = p2.y() - p1.y();
    	v1[2] = p2.z() - p1.z();
    	v2[0] = p3.x() - p1.x();
    	v2[1] = p3.y() - p1.y();
    	v2[2] = p3.z() - p1.z();
    	normal = normalize(cross(v1, v2));
    }

    return Point3D( normal[0], normal[1], normal[2] );
}
#endif

#if 0
// build the face normal list
static point_list gen_face_normals( TGConstruct& c ) {
    point_list face_normals;

    // traverse triangle structure building the face normal table
    SG_LOG(SG_GENERAL, SG_ALERT, "calculating face normals");

    triele_list tri_elements = c.get_tri_elements();
    for ( int i = 0; i < (int)tri_elements.size(); i++ ) {
        Point3D p = calc_normal(c,  i );
        //cout << p << endl;
        face_normals.push_back( p );
    }

    return face_normals;
}
#endif

#if 0
// calculate the normals for each point in wgs84_nodes
static point_list gen_point_normals( TGConstruct& c ) {
    point_list point_normals;

    Point3D normal;
    SG_LOG(SG_GENERAL, SG_ALERT, "calculating node normals");

    point_list wgs84_nodes = c.get_wgs84_nodes();
    belongs_to_list reverse_ele_lookup = c.get_reverse_ele_lookup();
    point_list face_normals = c.get_face_normals();
    triele_list tri_elements = c.get_tri_elements();

    // for each node
    for ( int i = 0; i < (int)wgs84_nodes.size(); ++i ) {
        int_list tri_list = reverse_ele_lookup[i];
        double total_area = 0.0;

        Point3D average( 0.0 );

        // for each triangle that shares this node
        for ( int j = 0; j < (int)tri_list.size(); ++j ) {
            normal = face_normals[ tri_list[j] ];
            double area = tri_ele_area( c, tri_elements[ tri_list[j] ] );
            normal *= area;	// scale normal weight relative to area
            total_area += area;
            average += normal;
            // cout << normal << endl;
        }

        average /= total_area;
        //cout << "average = " << average << endl;

        point_normals.push_back( average );
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "1st");
    SG_LOG(SG_GENERAL, SG_ALERT, "wgs84 node list size = " << wgs84_nodes.size());
    SG_LOG(SG_GENERAL, SG_ALERT, "normal list size = " << point_normals.size());

    return point_normals;
}
#endif


# if 0
// generate the flight gear scenery file
static void do_output( TGConstruct& c, TGGenOutput& output ) {
    output.build_tris( c );
    output.write_tris( c );
}
#endif

bool TGNodesSortByLon( const TGNode& n1, const TGNode& n2 )
{
    return ( n1.GetPosition().x() < n2.GetPosition().x() );
}

static void dump_nodes( TGConstruct& c ) {
    for (unsigned int i=0; i<c.get_nodes()->size(); i++) {
        TGNode node = c.get_nodes()->get_node( i );
        string fixed;
        
        if ( node.GetFixedPosition() ) {
            fixed = " z is fixed elevation ";
        } else {
            fixed = " z is interpolated elevation ";
        }

        SG_LOG(SG_GENERAL, SG_ALERT, "Point[" << i << "] is " << node.GetPosition() << fixed );
    }
}

static void dump_lat_nodes( TGConstruct& c, double lat ) {
    node_list all_nodes = c.get_nodes()->get_node_list();
    node_list sorted_nodes;
    for (unsigned int i=0; i<all_nodes.size(); i++) {
        if ( fabs( all_nodes[i].GetPosition().y() - lat ) < 0.0000001 ) {
            sorted_nodes.push_back( all_nodes[i] );
        }
    }

    sort( sorted_nodes.begin(), sorted_nodes.end(), TGNodesSortByLon );

    for (unsigned int i=0; i<sorted_nodes.size(); i++) {
        string fixed;
        
        if ( sorted_nodes[i].GetFixedPosition() ) {
            fixed = " z is fixed elevation ";
        } else {
            fixed = " z is interpolated elevation ";
        }

        SG_LOG(SG_GENERAL, SG_ALERT, "Point[" << i << "] is " << sorted_nodes[i].GetPosition() << fixed );
    }
}

// display usage and exit
static void usage( const string name ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Usage: " << name);
    SG_LOG(SG_GENERAL, SG_ALERT, "[ --output-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --work-dir=<directory>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --cover=<path to land-cover raster>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --tile-id=<id>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --lon=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --lat=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --xdist=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --ydist=<degrees>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --nudge=<float>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --priorities=<filename>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --usgs-map=<filename>");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --useUKgrid");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --no-write-shared-edges");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --use-own-shared-edges");
    SG_LOG(SG_GENERAL, SG_ALERT, "  --ignore-landmass");
    SG_LOG(SG_GENERAL, SG_ALERT, " ] <load directory...>");
    exit(-1);
}

int main(int argc, char **argv) {
    string output_dir = ".";
    string work_dir = ".";
    string cover = "";
    string priorities_file = DEFAULT_PRIORITIES_FILE;
    string usgs_map_file = DEFAULT_USGS_MAPFILE;
    double lon = -110.664244;	// P13
    double lat = 33.352890;
    double xdist = -1;		// 1/2 degree in each direction
    double ydist = -1;
    long tile_id = -1;

    // flag indicating whether UK grid should be used for in-UK
    // texture coordinate generation
    bool useUKgrid = false;
    
    // flag indicating whether this is a rebuild and Shared edge
    // data should only be used for fitting, but not rewritten
    bool writeSharedEdges = true;
    
    // flag indicating whether the shared edge data of the
    // tile to be built should be used in addition to neighbour data
    bool useOwnSharedEdges = false;
    
    bool ignoreLandmass = false;
    
    sglog().setLogLevels( SG_ALL, SG_INFO );

    //
    // Parse the command-line arguments.
    //
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        string arg = argv[arg_pos];

        if (arg.find("--output-dir=") == 0) {
            output_dir = arg.substr(13);
        } else if (arg.find("--work-dir=") == 0) {
            work_dir = arg.substr(11);
        } else if (arg.find("--tile-id=") == 0) {
            tile_id = atol(arg.substr(10).c_str());
        } else if (arg.find("--lon=") == 0) {
            lon = atof(arg.substr(6).c_str());
        } else if (arg.find("--lat=") == 0) {
            lat = atof(arg.substr(6).c_str());
        } else if (arg.find("--xdist=") == 0) {
            xdist = atof(arg.substr(8).c_str());
        } else if (arg.find("--ydist=") == 0) {
            ydist = atof(arg.substr(8).c_str());
        } else if (arg.find("--nudge=") == 0) {
            nudge = atof(arg.substr(8).c_str())*SG_EPSILON;
        } else if (arg.find("--cover=") == 0) {
            cover = arg.substr(8);
        } else if (arg.find("--priorities=") == 0) {
            priorities_file = arg.substr(13);
        } else if (arg.find("--usgs-map=") == 0) {
            usgs_map_file = arg.substr(11);
        } else if (arg.find("--useUKgrid") == 0) {
            useUKgrid = true;
        } else if (arg.find("--no-write-shared-edges") == 0) {
            writeSharedEdges = false;
        } else if (arg.find("--use-own-shared-edges") == 0) {
            useOwnSharedEdges = true;
        } else if (arg.find("--ignore-landmass") == 0) {
            ignoreLandmass = true;
        } else if (arg.find("--") == 0) {
            usage(argv[0]);
        } else {
            break;
        }
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "Output directory is " << output_dir);
    SG_LOG(SG_GENERAL, SG_ALERT, "Working directory is " << work_dir);
    if ( tile_id > 0 ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Tile id is " << tile_id);
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "Center longitude is " << lon);
        SG_LOG(SG_GENERAL, SG_ALERT, "Center latitude is " << lat);
        SG_LOG(SG_GENERAL, SG_ALERT, "X distance is " << xdist);
        SG_LOG(SG_GENERAL, SG_ALERT, "Y distance is " << ydist);
    }
    SG_LOG(SG_GENERAL, SG_ALERT, "Nudge is " << nudge);
    for (int i = arg_pos; i < argc; i++) {
        load_dirs.push_back(argv[i]);
        SG_LOG(SG_GENERAL, SG_ALERT, "Load directory: " << argv[i]);
    }
    SG_LOG(SG_GENERAL, SG_ALERT, "Priorities file is " << priorities_file);
    if ( ! load_area_types( priorities_file ) ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to load priorities file " << priorities_file);
        exit(-1);
    }
    SG_LOG(SG_GENERAL, SG_ALERT, "USGS Map file is " << usgs_map_file);
    if ( ! load_usgs_map( usgs_map_file ) ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to load USGS map file " << usgs_map_file);
	    exit(-1);
    }


#if defined( __CYGWIN__ ) || defined( __CYGWIN32__ ) || defined( _MSC_VER )
    // the next bit crashes Cygwin for me - DCL
    // MSVC does not have the function or variable type defined - BRF
#else
    // set mem allocation limit.  Reason: occasionally the triangle()
    // routine can blow up and allocate memory forever.  We'd like
    // this process to die before things get out of hand so we can try
    // again with a smaller interior angle limit.
    struct rlimit limit;
    limit.rlim_cur = 40000000;
    limit.rlim_max = 40000000;

#if 0
    result = setrlimit( RLIMIT_DATA, &limit );
    cout << "result of setting mem limit = " << result << endl;
    result = setrlimit( RLIMIT_STACK, &limit );
    cout << "result of setting mem limit = " << result << endl;
    result = setrlimit( RLIMIT_CORE, &limit );
    cout << "result of setting mem limit = " << result << endl;
    result = setrlimit( RLIMIT_RSS, &limit );
    cout << "result of setting mem limit = " << result << endl;
#endif

    // cpu time limit since occassionally the triangulator can go into
    // an infinite loop.
    limit.rlim_cur = 43200;	// seconds
    limit.rlim_max = 43200;	// seconds
    if (setrlimit( RLIMIT_CPU, &limit )) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Error setting RLIMIT_CPU, aborting");
        exit(-1);
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "Setting RLIMIT_CPU to " << limit.rlim_cur << " seconds");
    };
    
#endif  // end of stuff that crashes Cygwin

    // main construction data management class
    TGConstruct c;

    c.set_cover( cover );
    c.set_work_base( work_dir );
    c.set_output_base( output_dir );
    c.set_load_dirs( load_dirs );
    c.set_useUKGrid( useUKgrid );
    c.set_write_shared_edges( writeSharedEdges );
    c.set_use_own_shared_edges( useOwnSharedEdges );
    c.set_ignore_landmass( ignoreLandmass );
    c.set_nudge( nudge );

    if (tile_id == -1) {
        if (xdist == -1 || ydist == -1) {
            // construct the tile around the specified location
            SG_LOG(SG_GENERAL, SG_ALERT, "Building single tile at " << lat << ',' << lon);
            SGBucket b( lon, lat );
            c.construct_bucket( b );
        } else {
            // build all the tiles in an area
            SG_LOG(SG_GENERAL, SG_ALERT, "Building tile(s) at " << lat << ',' << lon << " with x distance " << xdist << " and y distance " << ydist);
            double min_x = lon - xdist;
            double min_y = lat - ydist;
            SGBucket b_min( min_x, min_y );
            SGBucket b_max( lon + xdist, lat + ydist );

            SGBucket b_start(550401L);
            bool do_tile = true;

            if ( b_min == b_max ) {
            	c.construct_bucket( b_min );
            } else {
                SGBucket b_cur;
                int dx, dy, i, j;

                sgBucketDiff(b_min, b_max, &dx, &dy);
                SG_LOG(SG_GENERAL, SG_ALERT, "  construction area spans tile boundaries");
                SG_LOG(SG_GENERAL, SG_ALERT, "  dx = " << dx << "  dy = " << dy);

                for ( j = 0; j <= dy; j++ ) {
                    for ( i = 0; i <= dx; i++ ) {
                        b_cur = sgBucketOffset(min_x, min_y, i, j);

                        if ( b_cur == b_start ) {
                            do_tile = true;
                        }

                        if ( do_tile ) {
                            c.construct_bucket( b_cur );
                        } else {
                            SG_LOG(SG_GENERAL, SG_ALERT, "skipping " << b_cur);
                        }
                    }
                }
            }
        }
    } else {
        // construct the specified tile
        SG_LOG(SG_GENERAL, SG_ALERT, "Building tile " << tile_id);
        SGBucket b( tile_id );
        c.construct_bucket( b );
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "[Finished successfully]");
    return 0;
}
