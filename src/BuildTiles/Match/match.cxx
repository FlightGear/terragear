// match.cxx -- Handle details of matching up tile edges
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998 - 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: match.cxx,v 1.21 2004-11-19 22:25:49 curt Exp $


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>
#include <Geometry/point3d.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sgstream.hxx>
#include <simgear/misc/sg_path.hxx>

#include "match.hxx"

#include <stdlib.h>

using std::cout;
using std::endl;
using std::string;

TGMatch::TGMatch( void ) {
}


TGMatch::~TGMatch( void ) {
}


// scan the specified share file for the specified information
void TGMatch::scan_share_file( const string& dir, const SGBucket& b, 
			       neighbor_type search, neighbor_type dest )
{
    string file = dir + "/"  + b.gen_base_path() + "/" + b.gen_index_str();

    cout << "reading shared data from " << file << endl;

    sg_gzifstream in( file );
    if ( !in.is_open() ) {
        cout << "Cannot open file: " << file << endl;
	return;
    }

    cout << "open successful." << endl;

    string target;
    if ( search == SW_Corner ) {
	target = "sw_node";
    } else if ( search == SE_Corner ) {
	target = "se_node";
    } else if ( search == NE_Corner ) {
	target = "ne_node";
    } else if ( search == NW_Corner ) {
	target = "nw_node";
    } else if ( search == NORTH ) {
	target = "n_node";
    } else if ( search == SOUTH ) {
	target = "s_node";
    } else if ( search == EAST ) {
	target = "e_node";
    } else if ( search == WEST ) {
	target = "w_node";
    }

    string key;
    Point3D node, normal;
    while ( in ) {
	in >> key;
	in >> node;
	if ( key == target ) {
	    // cout << key << " " << node << endl;
	    in >> key;
	    in >> normal;

	    if ( dest == SW_Corner ) {
		sw_node = node;
		sw_normal = normal;
		sw_flag = true;
	    } else if ( dest == SE_Corner ) {
		se_node = node;
		se_normal = normal;
		se_flag = true;
	    } else if ( dest == NE_Corner ) {
		ne_node = node;
		ne_normal = normal;
		ne_flag = true;
	    } else if ( dest == NW_Corner ) {
		nw_node = node;
		nw_normal = normal;
		nw_flag = true;
	    } else if ( dest == NORTH ) {
		north_nodes.push_back(node);
		north_normals.push_back(normal);
		north_flag = true;
	    } else if ( dest == SOUTH ) {
		south_nodes.push_back(node);
		south_normals.push_back(normal);
		south_flag = true;
	    } else if ( dest == EAST ) {
		east_nodes.push_back(node);
		east_normals.push_back(normal);
		east_flag = true;
	    } else if ( dest == WEST ) {
		west_nodes.push_back(node);
		west_normals.push_back(normal);
		west_flag = true;
	    }
	} else if ( (target == "n_node") && (key == "n_null") ) {
	    south_flag = true;
	} else if ( (target == "s_node") && (key == "s_null") ) {
	    north_flag = true;
	} else if ( (target == "e_node") && (key == "e_null") ) {
	    west_flag = true;
	} else if ( (target == "w_node") && (key == "w_null") ) {
	    east_flag = true;
	}
    }
}


// try to find info for the specified shared component
void TGMatch::load_shared( const TGConstruct& c, neighbor_type n ) {
    SGBucket b = c.get_bucket();

    double clon = b.get_center_lon();
    double clat = b.get_center_lat();

    string base = c.get_work_base() + "/Shared/";

    SGBucket cb;

    if ( n == SW_Corner ) {
	// cout << "searching for SW corner data" << endl;
	cb = sgBucketOffset(clon, clat, -1, 0);
	scan_share_file( base, cb, SE_Corner, n );
	cb = sgBucketOffset(clon, clat, -1, -1);
	scan_share_file( base, cb, NE_Corner, n );
	cb = sgBucketOffset(clon, clat, 0, -1);
	scan_share_file( base, cb, NW_Corner, n );
    } else if ( n == SE_Corner ) {
	// cout << "searching for SE corner data" << endl;
	cb = sgBucketOffset(clon, clat, 0, -1);
	scan_share_file( base, cb, NE_Corner, n );
	cb = sgBucketOffset(clon, clat, 1, -1);
	scan_share_file( base, cb, NW_Corner, n );
	cb = sgBucketOffset(clon, clat, 1, 0);
	scan_share_file( base, cb, SW_Corner, n );
    } else if ( n == NE_Corner ) {
	// cout << "searching for NE corner data" << endl;
	cb = sgBucketOffset(clon, clat, 1, 0);
	scan_share_file( base, cb, NW_Corner, n );
	cb = sgBucketOffset(clon, clat, 1, 1);
	scan_share_file( base, cb, SW_Corner, n );
	cb = sgBucketOffset(clon, clat, 0, 1);
	scan_share_file( base, cb, SE_Corner, n );
    } else if ( n == NW_Corner ) {
	// cout << "searching for NW corner data" << endl;
	cb = sgBucketOffset(clon, clat, 0, 1);
	scan_share_file( base, cb, SW_Corner, n );
	cb = sgBucketOffset(clon, clat, -1, 1);
	scan_share_file( base, cb, SE_Corner, n );
	cb = sgBucketOffset(clon, clat, -1, 0);
	scan_share_file( base, cb, NE_Corner, n );
    } else if ( n == NORTH ) {
	// cout << "searching for NORTH edge data" << endl;
 	cb = sgBucketOffset(clon, clat, 0, 1);
	scan_share_file( base, cb, SOUTH, n );
    } else if ( n == SOUTH ) {
	// cout << "searching for SOUTH edge data" << endl;
 	cb = sgBucketOffset(clon, clat, 0, -1);
	scan_share_file( base, cb, NORTH, n );
    } else if ( n == EAST ) {
	// cout << "searching for EAST edge data" << endl;
 	cb = sgBucketOffset(clon, clat, 1, 0);
	scan_share_file( base, cb, WEST, n );
    } else if ( n == WEST ) {
	// cout << "searching for WEST edge data" << endl;
 	cb = sgBucketOffset(clon, clat, -1, 0);
	scan_share_file( base, cb, EAST, n );
    }
}


// load any previously existing shared data from all neighbors (if
// shared data for a component exists set that components flag to true
void TGMatch::load_neighbor_shared( TGConstruct& c ) {
    cout << "Loading existing shared data from neighbor tiles" << endl;

    // start with all flags false
    sw_flag = se_flag = ne_flag = nw_flag = false;
    north_flag = south_flag = east_flag = west_flag = false;

    load_shared( c, SW_Corner );
    load_shared( c, SE_Corner );
    load_shared( c, NE_Corner );
    load_shared( c, NW_Corner );

    north_nodes.clear();
    south_nodes.clear();
    east_nodes.clear();
    west_nodes.clear();

    load_shared( c, NORTH );
    load_shared( c, SOUTH );
    load_shared( c, EAST );
    load_shared( c, WEST );

    cout << "Shared data read in:" << endl;
    if ( sw_flag ) {
	cout << "  sw corner = " << sw_node << endl;
	cout << "     normal = " << sw_normal << endl;
    }
    if ( se_flag ) {
	cout << "  se corner = " << se_node << endl;
	cout << "     normal = " << se_normal << endl;
    }
    if ( ne_flag ) {
	cout << "  ne corner = " << ne_node << endl;
	cout << "     normal = " << ne_normal << endl;
    }
    if ( nw_flag ) {
	cout << "  nw corner = " << nw_node << endl;
	cout << "     normal = " << nw_normal << endl;
    }
    if ( north_flag ) {
	cout << "  north nodes = " << north_nodes.size() << endl;
	for ( int i = 0; i < (int)north_nodes.size(); ++i ) {
	    cout << "    " << north_nodes[i] << endl;
	}
    }
    if ( south_flag ) { 
	cout << "  south nodes = " << south_nodes.size() << endl;
	for ( int i = 0; i < (int)south_nodes.size(); ++i ) {
	    cout << "    " << south_nodes[i] << endl;
	}
    }
    if ( east_flag ) { 
	cout << "  east nodes = " << east_nodes.size() << endl;
	for ( int i = 0; i < (int)east_nodes.size(); ++i ) {
	    cout << "    " << east_nodes[i] << endl;
	}
    }
    if ( west_flag ) { 
	cout << "  west nodes = " << west_nodes.size() << endl;
	for ( int i = 0; i < (int)west_nodes.size(); ++i ) {
	    cout << "    " << west_nodes[i] << endl;
	}
    }
}

// try to load any missing shared data from our own shared data file
void TGMatch::load_missing_shared( TGConstruct& c ) {
    SGBucket b = c.get_bucket();

    double clon = b.get_center_lon();
    double clat = b.get_center_lat();

    string base = c.get_work_base() + "/Shared/";
    
    if ( !nw_flag ) {
	scan_share_file( base, b, NW_Corner, NW_Corner );
    }
    
    if ( !ne_flag ) {
	scan_share_file( base, b, NE_Corner, NE_Corner );
    }
    
    if ( !se_flag ) {
	scan_share_file( base, b, SE_Corner, SE_Corner );
    }
    
    if ( !sw_flag ) {
	scan_share_file( base, b, SW_Corner, SW_Corner );
    }
    
    if ( !north_flag ) {
	scan_share_file( base, b, NORTH, NORTH );
    }
    
    if ( !east_flag ) {
	scan_share_file( base, b, EAST, EAST );
    }
    
    if ( !south_flag ) {
	scan_share_file( base, b, SOUTH, SOUTH );
    }
    
    if ( !west_flag ) {
	scan_share_file( base, b, WEST, WEST );
    }
}


// fake a normal for a point which is basically straight up
Point3D tgFakeNormal( const Point3D& p ) {
    Point3D radians = Point3D( p.x() * SGD_DEGREES_TO_RADIANS,
			       p.y() * SGD_DEGREES_TO_RADIANS,
			       p.z() );
    Point3D cart = sgGeodToCart(radians);
    double len = Point3D(0.0).distance3D(cart);
    // cout << "len = " << len << endl;
    cart /= len;
    cout << "new fake normal = " << cart << endl;

    return cart;
}


// split up the tile between the shared edge points, normals, and
// segments and the body.  This must be done after calling
// load_neighbor_data() and will ignore any shared data from the
// current tile that already exists from a neighbor.
void TGMatch::split_tile( TGConstruct& c ) {
    int i;

    cout << "Spliting tile" << endl;
    cout << "  extracting (shared) edge nodes and normals" << endl;

    // calculate tile boundaries
    point2d min, max;
    SGBucket b = c.get_bucket();
    min.x = b.get_center_lon() - 0.5 * b.get_width();
    min.y = b.get_center_lat() - 0.5 * b.get_height();
    max.x = b.get_center_lon() + 0.5 * b.get_width();
    max.y = b.get_center_lat() + 0.5 * b.get_height();

    // defaults "just in case"
    if ( ! sw_flag ) {
	sw_node = Point3D( min.x, min.y, 0.0 );
	sw_normal = tgFakeNormal( sw_node );
    }
    if ( ! se_flag ) {
	se_node = Point3D( max.x, min.y, 0.0 );
	se_normal = tgFakeNormal( se_node );
    }
    if ( ! nw_flag ) {
	nw_node = Point3D( min.x, max.y, 0.0 );
 	nw_normal = tgFakeNormal( nw_node );
    }
    if ( ! ne_flag ) {
	ne_node = Point3D( max.x, max.y, 0.0 );
 	ne_normal = tgFakeNormal( ne_node );
   }

    // separate nodes and normals into components

    body_nodes.clear();

    point_list nodes = c.get_geod_nodes();
    point_list point_normals = c.get_point_normals();

    for ( i = 0; i < (int)nodes.size(); ++i ) {
	Point3D node = nodes[i];
	Point3D normal = point_normals[i];

	if ( (fabs(node.y() - min.y) < SG_EPSILON) && 
	     (fabs(node.x() - min.x) < SG_EPSILON) ) {
	    if ( ! sw_flag ) {
		sw_node = node;
		sw_normal = normal;
	    }
	} else if ( (fabs(node.y() - min.y) < SG_EPSILON) &&
		    (fabs(node.x() - max.x) < SG_EPSILON) ) {
	    if ( ! se_flag ) {
		se_node = node;
		se_normal = normal;
	    }
	} else if ( (fabs(node.y() - max.y) < SG_EPSILON) &&
		    (fabs(node.x() - max.x) < SG_EPSILON)) {
	    if ( ! ne_flag ) {
		ne_node = node;
		ne_normal = normal;
	    }
	} else if ( (fabs(node.y() - max.y) < SG_EPSILON) &&
		    (fabs(node.x() - min.x) < SG_EPSILON) ) {
	    if ( ! nw_flag ) {
		nw_node = node;
		nw_normal = normal;
	    }
	} else if ( fabs(node.x() - min.x) < SG_EPSILON ) {
	    if ( ! west_flag ) {
		west_nodes.push_back( node );
		west_normals.push_back( normal );
	    }
	} else if ( fabs(node.x() - max.x) < SG_EPSILON ) {
	    if ( ! east_flag ) {
		east_nodes.push_back( node );
		east_normals.push_back( normal );
	    }
	} else if ( fabs(node.y() - min.y) < SG_EPSILON ) {
	    if ( ! south_flag ) {
		south_nodes.push_back( node );
		south_normals.push_back( normal );
	    }
	} else if ( fabs(node.y() - max.y) < SG_EPSILON ) {
	    if ( ! north_flag ) {
		north_nodes.push_back( node );
		north_normals.push_back( normal );
	    }
	} else {
	    body_nodes.push_back( node );
	    body_normals.push_back( normal );
	}
    }

    // separate area edge segment into components
    cout << "  extracting (shared) area edge segments" << endl;

    TGTriSeg seg;
    Point3D p1, p2;
    triseg_list seg_list = c.get_tri_segs().get_seg_list();
    triseg_list_iterator current = seg_list.begin();
    triseg_list_iterator last = seg_list.end();

    for ( ; current != last; ++current ) {
	seg = *current;
	p1 = nodes[ seg.get_n1() ];
	p2 = nodes[ seg.get_n2() ];

	if ( fabs(p1.y() - p2.y()) < SG_EPSILON ) {
	    // check if horizontal
	    if ( fabs(p1.y() - max.y) < SG_EPSILON ) {
		north_segs.push_back( seg );
	    } else if ( fabs(p1.y() - min.y) < SG_EPSILON ) {
		south_segs.push_back( seg );
	    } else {
		body_segs.push_back( seg );
	    }
	} else if ( fabs(p1.x() - p2.x()) < SG_EPSILON ) {
	    // check if vertical
	    if ( fabs(p1.x() - max.x) < SG_EPSILON ) {
		east_segs.push_back( seg );
	    } else if ( fabs(p1.x() - min.x) < SG_EPSILON ) {
		west_segs.push_back( seg );
	    } else {
		body_segs.push_back( seg );
	    }
	} else {
	    body_segs.push_back( seg );
	}
    }

    if ( !sw_flag ) { cout << "  sw corner = " << sw_node << endl; }
    if ( !se_flag ) { cout << "  se corner = " << se_node << endl; }
    if ( !ne_flag ) { cout << "  ne corner = " << ne_node << endl; }
    if ( !nw_flag ) { cout << "  nw corner = " << nw_node << endl; }
    /*
    if ( !north_flag ) { 
	cout << "  north nodes = " << north_nodes.size() << endl;
	for ( i = 0; i < (int)north_nodes.size(); ++i ) {
	    cout << "    " << north_nodes[i] << endl;
	}
    }
    if ( !south_flag ) { 
	cout << "  south nodes = " << south_nodes.size() << endl;
	for ( i = 0; i < (int)south_nodes.size(); ++i ) {
	    cout << "    " << south_nodes[i] << endl;
	}
    }
    if ( !east_flag ) { 
	cout << "  east nodes = " << east_nodes.size() << endl;
	for ( i = 0; i < (int)east_nodes.size(); ++i ) {
	    cout << "    " << east_nodes[i] << endl;
	}
    }
    if ( !west_flag ) { 
	cout << "  west nodes = " << west_nodes.size() << endl;
	for ( i = 0; i < (int)west_nodes.size(); ++i ) {
	    cout << "    " << west_nodes[i] << endl;
	}
    }
    cout << "  body nodes = " << body_nodes.size() << endl;
    for ( i = 0; i < (int)body_nodes.size(); ++i ) {
	cout << "    " << body_nodes[i] << endl;
    }
    */
}


// write the new shared edge points, normals, and segments for this
// tile
void TGMatch::write_shared( TGConstruct& c ) {
    string base = c.get_work_base();
    SGBucket b = c.get_bucket();

    string dir = base + "/Shared/" + b.gen_base_path();
    string file = dir + "/" + b.gen_index_str();

    SGPath sgp( dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    cout << "shared data will be written to " << file << endl;


    cout << "FLAGS" << endl;
    cout << "=====" << endl;
    cout << "sw_flag = " << sw_flag << endl;
    cout << "se_flag = " << se_flag << endl;
    cout << "ne_flag = " << ne_flag << endl;
    cout << "nw_flag = " << nw_flag << endl;
    cout << "north_flag = " << north_flag << endl;
    cout << "south_flag = " << south_flag << endl;
    cout << "east_flag = " << east_flag << endl;
    cout << "west_flag = " << west_flag << endl;

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "w" )) == NULL ) {
	cout << "ERROR: opening " << file << " for writing!" << endl;
	exit(-1);
    }

    /*
     * We only write data out for those sides for which the adjacent
     * tiles still have to be built.
     *
     * If we have already read data for a given corner or side, this
     * means that the adjacent tile already has been built.
     */
    if ( ! sw_flag ) {
	fprintf( fp, "sw_node %.6f %.6f %.6f\n", 
		 sw_node.x(), sw_node.y(), sw_node.z() );
	fprintf( fp, "sw_normal %.6f %.6f %.6f\n", 
		 sw_normal.x(), sw_normal.y(), sw_normal.z() );
    }

    if ( ! se_flag ) {
	fprintf( fp, "se_node %.6f %.6f %.6f\n", 
		 se_node.x(), se_node.y(), se_node.z() );
	fprintf( fp, "se_normal %.6f %.6f %.6f\n", 
		 se_normal.x(), se_normal.y(), se_normal.z() );
    }

    if ( ! nw_flag ) {
	fprintf( fp, "nw_node %.6f %.6f %.6f\n", 
		 nw_node.x(), nw_node.y(), nw_node.z() );
	fprintf( fp, "nw_normal %.6f %.6f %.6f\n", 
		 nw_normal.x(), nw_normal.y(), nw_normal.z() );
    }

    if ( ! ne_flag ) {
	fprintf( fp, "ne_node %.6f %.6f %.6f\n", 
		 ne_node.x(), ne_node.y(), ne_node.z() );
	fprintf( fp, "ne_normal %.6f %.6f %.6f\n", 
		 ne_normal.x(), ne_normal.y(), ne_normal.z() );
    }

    if ( ! north_flag ) {
	if ( (int)north_nodes.size() == 0 ) {
	    fprintf( fp, "n_null -999.0 -999.0 -999.0\n" );
	} else {
	    for ( int i = 0; i < (int)north_nodes.size(); ++i ) {
		fprintf( fp, "n_node %.6f %.6f %.6f\n", 
			 north_nodes[i].x(), north_nodes[i].y(),
			 north_nodes[i].z() );
		fprintf( fp, "n_normal %.6f %.6f %.6f\n", 
			 north_normals[i].x(), north_normals[i].y(),
			 north_normals[i].z() );
	    }
	}
    }

    if ( ! south_flag ) {
	if ( (int)south_nodes.size() == 0 ) {
	    fprintf( fp, "s_null -999.0 -999.0 -999.0\n" );
	} else {
	    for ( int i = 0; i < (int)south_nodes.size(); ++i ) {
		fprintf( fp, "s_node %.6f %.6f %.6f\n", 
			 south_nodes[i].x(), south_nodes[i].y(),
			 south_nodes[i].z() );
		fprintf( fp, "s_normal %.6f %.6f %.6f\n", 
			 south_normals[i].x(), south_normals[i].y(),
			 south_normals[i].z() );
	    }
	} 
    }

    if ( ! east_flag ) {
	if ( (int)east_nodes.size() == 0 ) {
	    fprintf( fp, "e_null -999.0 -999.0 -999.0\n" );
	} else {
	    for ( int i = 0; i < (int)east_nodes.size(); ++i ) {
		fprintf( fp, "e_node %.6f %.6f %.6f\n", 
			 east_nodes[i].x(), east_nodes[i].y(), 
			 east_nodes[i].z() );
		fprintf( fp, "e_normal %.6f %.6f %.6f\n", 
			 east_normals[i].x(), east_normals[i].y(),
			 east_normals[i].z() );
	    }
	}
    }

    if ( ! west_flag ) {
	if ( (int)west_nodes.size() == 0 ) {
	    fprintf( fp, "w_null -999.0 -999.0 -999.0\n" );
	} else {
	    for ( int i = 0; i < (int)west_nodes.size(); ++i ) {
		fprintf( fp, "w_node %.6f %.6f %.6f\n", 
			 west_nodes[i].x(), west_nodes[i].y(),
			 west_nodes[i].z() );
		fprintf( fp, "w_normal %.6f %.6f %.6f\n", 
			 west_normals[i].x(), west_normals[i].y(),
			 west_normals[i].z() );
	    }
	}
    }

#if 0 // not needed
    point_list nodes = c.get_geod_nodes();
    Point3D p1, p2;

    for ( int i = 0; i < (int)north_segs.size(); ++i ) {
	p1 = nodes[ north_segs[i].get_n1() ];
	p2 = nodes[ north_segs[i].get_n2() ];
	fprintf( fp, "n_seg %.6f %.6f %.6f %.6f\n", 
		 p1.x(), p1.y(), p2.x(), p2.y() );
    }

    for ( int i = 0; i < (int)south_segs.size(); ++i ) {
	p1 = nodes[ south_segs[i].get_n1() ];
	p2 = nodes[ south_segs[i].get_n2() ];
	fprintf( fp, "s_seg %.6f %.6f %.6f %.6f\n", 
		 p1.x(), p1.y(), p2.x(), p2.y() );
    }

    for ( int i = 0; i < (int)east_segs.size(); ++i ) {
	p1 = nodes[ east_segs[i].get_n1() ];
	p2 = nodes[ east_segs[i].get_n2() ];
	fprintf( fp, "e_seg %.6f %.6f %.6f %.6f\n", 
		 p1.x(), p1.y(), p2.x(), p2.y() );
    }

    for ( int i = 0; i < (int)west_segs.size(); ++i ) {
	p1 = nodes[ west_segs[i].get_n1() ];
	p2 = nodes[ west_segs[i].get_n2() ];
	fprintf( fp, "w_seg %.6f %.6f %.6f %.6f\n", 
		 p1.x(), p1.y(), p2.x(), p2.y() );
    }
#endif

    fclose( fp );

    string command = "gzip --force --best " + file;
    system(command.c_str());
}


// insert normal into vector, extending it first if needed
void insert_normal( point_list& normals, Point3D n, int i ) {
    Point3D empty( 0.0 );

    // extend vector if needed
    while ( i >= (int)normals.size() ) {
	normals.push_back( empty );
    }

    normals[i] = n;
}


// reassemble the tile pieces (combining the shared data and our own
// data)
void TGMatch::assemble_tile( TGConstruct& c ) {
    int i;
    TGTriNodes new_nodes;
    new_nodes.clear();

    point_list new_normals;
    new_normals.clear();

    TGTriSegments new_segs;
    new_segs.clear();

    // add the corner points
    int sw_index = new_nodes.unique_add( sw_node );
    insert_normal( new_normals, sw_normal, sw_index );

    int se_index = new_nodes.unique_add( se_node );
    insert_normal( new_normals, se_normal, se_index );

    int ne_index = new_nodes.unique_add( ne_node );
    insert_normal( new_normals, ne_normal, ne_index );

    int nw_index = new_nodes.unique_add( nw_node );
    insert_normal( new_normals, nw_normal, nw_index );

    cout << "after adding corners:" << endl;
    cout << "  new_nodes = " << new_nodes.size() << endl;
    cout << "  new normals = " << new_normals.size() << endl;

    // add the edge points

    int index;

    // cout << "Total north nodes = " << north_nodes.size() << endl;
    for ( i = 0; i < (int)north_nodes.size(); ++i ) {
	// cout << "adding north node " << north_nodes[i] << endl;
	index = new_nodes.unique_add( north_nodes[i] );
	insert_normal( new_normals, north_normals[i], index );
    }

    for ( i = 0; i < (int)south_nodes.size(); ++i ) {
	index = new_nodes.unique_add( south_nodes[i] );
	insert_normal( new_normals, south_normals[i], index );
    }

    for ( i = 0; i < (int)east_nodes.size(); ++i ) {
	index = new_nodes.unique_add( east_nodes[i] );
	insert_normal( new_normals, east_normals[i], index );
    }

    // cout << "Total west nodes = " << west_nodes.size() << endl;
    for ( i = 0; i < (int)west_nodes.size(); ++i ) {
	// cout << "adding west node " << west_nodes[i] << endl;
	index = new_nodes.unique_add( west_nodes[i] );
	insert_normal( new_normals, west_normals[i], index );
    }

    cout << "after adding edges:" << endl;
    cout << "  new_nodes = " << new_nodes.size() << endl;
    cout << "  new normals = " << new_normals.size() << endl;

    // add the body points
    for ( i = 0; i < (int)body_nodes.size(); ++i ) {
	index = new_nodes.unique_add( body_nodes[i] );
	insert_normal( new_normals, body_normals[i], index );
    }

    cout << "after adding body points:" << endl;
    cout << "  new_nodes = " << new_nodes.size() << endl;
    cout << "  new normals = " << new_normals.size() << endl;

    // add the edge segments
    new_segs.unique_divide_and_add( new_nodes.get_node_list(),
				    TGTriSeg(sw_index, se_index, 1) );
    new_segs.unique_divide_and_add( new_nodes.get_node_list(),
				    TGTriSeg(se_index, ne_index, 1) );
    new_segs.unique_divide_and_add( new_nodes.get_node_list(),
				    TGTriSeg(ne_index, nw_index, 1) );
    new_segs.unique_divide_and_add( new_nodes.get_node_list(),
				    TGTriSeg(nw_index, sw_index, 1) );
    cout << "after adding edge segments:" << endl;
    cout << "  new_nodes = " << new_nodes.size() << endl;
    cout << "  new normals = " << new_normals.size() << endl;

    // add the body segments

    point_list geod_nodes = c.get_geod_nodes();

    TGTriSeg seg;
    Point3D p1, p2;
    int n1, n2, marker;

    triseg_list_iterator current = body_segs.begin();
    triseg_list_iterator last = body_segs.end();

    for ( ; current != last; ++current ) {
	seg = *current;

	// get the original points (x,y,z)
	p1 = geod_nodes[ seg.get_n1() ];
	p2 = geod_nodes[ seg.get_n2() ];
	marker = seg.get_boundary_marker();

	// make sure these points are in the new node list (and get
	// their new index)
	n1 = new_nodes.unique_add( p1 );
	if ( n1 >= (int)new_normals.size() ) {
	    cout << "Adding a segment resulted in a new node, faking a normal" 
		 << endl;
	    Point3D fake = tgFakeNormal( p1 );
	    insert_normal( new_normals, fake, n1 );
	}

	n2 = new_nodes.unique_add( p2 );
	if ( n2 >= (int)new_normals.size() ) {
	    cout << "Adding a segment resulted in a new node, faking a normal" 
		 << endl;
	    Point3D fake = tgFakeNormal( p2 );
	    insert_normal( new_normals, fake, n2 );
	}

	// add the segment using the new indices
	new_segs.unique_divide_and_add( new_nodes.get_node_list(),
					TGTriSeg(n1, n2, marker) );
    }

    c.set_tri_nodes( new_nodes );
    c.set_point_normals( new_normals );
    c.set_tri_segs( new_segs );

    cout << "after adding all segments (should be the same):" << endl;
    cout << "  new_nodes = " << new_nodes.size() << endl;
    cout << "  new normals = " << new_normals.size() << endl;

}
