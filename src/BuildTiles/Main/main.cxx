// main.cxx -- top level construction routines
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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
// $Id$


#ifdef _MSC_VER
#  include <io.h>
#else
#  include <sys/types.h>	// for directory reading
#  include <dirent.h>		// for directory reading
#endif

#ifdef HAVE_SYS_TIME_H
#  include <sys/time.h>		// set mem allocation limit
#endif
#ifndef _MSC_VER
#  include <sys/resource.h>	// set mem allocation limit
#  include <unistd.h>		// set mem allocation limit
#endif

#include <iostream>
#include <string>
#include <vector>

#include <plib/sg.h>

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>

#include <Geometry/poly_support.hxx>
#include <Array/array.hxx>
#include <Clipper/clipper.hxx>
#include <GenOutput/genobj.hxx>
#include <Match/match.hxx>
#include <Triangulate/triangle.hxx>
#include <landcover/landcover.hxx>

#include "construct.hxx"

SG_USING_STD(cout);
SG_USING_STD(endl);
SG_USING_STD(string);
SG_USING_STD(vector);

vector<string> load_dirs;


// Translate USGS land cover values into TerraGear area types.
static AreaType translateUSGSCover (int usgs_value)
{
  switch (usgs_value) {

  case 1:			// Urban and Built-Up Land
    return BuiltUpCover;
  case 2:			// Dryland Cropland and Pasture
    return DryCropPastureCover;
  case 3:			// Irrigated Cropland and Pasture
    return IrrCropPastureCover;
  case 4:			// Mixed Dryland/Irrigated Cropland and Pasture
    return MixedCropPastureCover;
  case 5:			// Cropland/Grassland Mosaic
    return CropGrassCover;
  case 6:			// Cropland/Woodland Mosaic
    return CropWoodCover;
  case 7:			// Grassland
    return GrassCover;
  case 8:			// Shrubland
    return ShrubCover;
  case 9:			// Mixed Shrubland/Grassland
    return ShrubGrassCover;
  case 10:			// Savanna
    return SavannaCover;
  case 11:			// Deciduous Broadleaf Forest
    return DeciduousBroadCover;
  case 12:			// Deciduous Needleleaf Forest
    return DeciduousNeedleCover;
  case 13:			// Evergreen Broadleaf Forest
    return EvergreenBroadCover;
  case 14:			// Evergreen Needleleaf Forest
    return EvergreenNeedleCover;
  case 15:			// Mixed Forest
    return MixedForestCover;
  case 16:			// Water Bodies
    // FIXME: use the type of an adjoining area if possible
    // return WaterBodyCover;
    return DefaultArea;
  case 17:			// Herbaceous Wetland
    return HerbWetlandCover;
  case 18:			// Wooded Wetland
    return WoodedWetlandCover;
  case 19:			// Barren or Sparsely Vegetated
    return BarrenCover;
  case 20:			// Herbaceous Tundra
    return HerbTundraCover;
  case 21:			// Wooded Tundra
    return WoodedTundraCover;
  case 22:			// Mixed Tundra
    return MixedTundraCover;
  case 23:			// Bare Ground Tundra
    return BareTundraCover;
  case 24:			// Snow or Ice
    return SnowCover;
  default:			// Unknown
    return DefaultArea;
  }
}


// Scan a directory and load polygon files.
static int actual_load_polys( const string& dir,
			      FGConstruct& c,
			      FGClipper& clipper ) {
    int counter = 0;
    string base = c.get_bucket().gen_base_path();
    string tile_str = c.get_bucket().gen_index_str();
    string ext;
    string file, f_index, full_path;
    int pos;

#ifdef _MSC_VER
    long hfile;
    struct _finddata_t de;
    string path;

    path = dir + "/*.*";

    if ( ( hfile = _findfirst( path.c_str(), &de ) ) == -1 ) {
	cout << "cannot open directory " << dir << "\n";
	return 0;
    }

    // load all matching polygon files
    do {
        file = de.name;
        pos = file.find(".");
        f_index = file.substr(0, pos);

        if ( tile_str == f_index ) {
            ext = file.substr(pos + 1);
            cout << file << "  " << f_index << "  '" << ext << "'" << endl;
            full_path = dir + "/" + file;
            if ( (ext == "dem") || (ext == "dem.gz") ) {
                // skip
            } else if (ext == "osgb36") {
                cout << "Loading osgb36 poly definition file\n";
                clipper.load_osgb36_polys( full_path );
                ++counter;
            } else {
                cout << "ext = '" << ext << "'" << endl;
                clipper.load_polys( full_path );
                ++counter;
            }
        }
    } while ( _findnext( hfile, &de ) == 0 );

#else

    DIR *d;
    struct dirent *de;

    if ( (d = opendir( dir.c_str() )) == NULL ) {
        cout << "cannot open directory " << dir << "\n";
	return 0;
    }

    // load all matching polygon files
    while ( (de = readdir(d)) != NULL ) {
        file = de->d_name;
        pos = file.find(".");
        f_index = file.substr(0, pos);

        if ( tile_str == f_index ) {
            ext = file.substr(pos + 1);
            cout << file << "  " << f_index << "  '" << ext << "'" << endl;
            full_path = dir + "/" + file;
            if ( (ext == "dem") || (ext == "dem.gz") || (ext == "ind") ) {
                // skip
            } else if (ext == "osgb36") {
                cout << "Loading osgb36 poly definition file\n";
                clipper.load_osgb36_polys( full_path );
                ++counter;
            } else {
                cout << "ext = '" << ext << "'" << endl;
                clipper.load_polys( full_path );
                ++counter;
            }
        }
    }

    closedir(d);
#endif
	
    return counter;
}


// Add a polygon to a list, merging if possible.
//
// Merge a polygon with an existing one if possible, append a new one
// otherwise; this function is used by actual_load_landcover, below,
// to reduce the number of separate polygons.
static void inline add_to_polys ( FGPolygon &accum, const FGPolygon &poly) {
    if ( accum.contours() > 0 ) {
	accum = polygon_union( accum, poly );
    } else {
	accum = poly;
    }
}


// make the area specified area, look up the land cover type, and add
// it to polys
static void make_area( const LandCover &cover, FGPolygon *polys,
		       double x1, double y1, double x2, double y2,
		       double half_dx, double half_dy )
{
    // Look up the land cover for the square
    int cover_value = cover.getValue( x1 + half_dx, y1 + half_dy );
    cout << " position: " << x1 << ',' << y1 << ','
	 << cover.getDescUSGS(cover_value) << endl;
    AreaType area = translateUSGSCover(cover_value);
    if (area != DefaultArea) {
	// Create a square polygon and merge it into the list.
	FGPolygon poly;
	poly.erase();
	poly.add_node(0, Point3D(x1, y1, 0.0));
	poly.add_node(0, Point3D(x1, y2, 0.0));
	poly.add_node(0, Point3D(x2, y2, 0.0));
	poly.add_node(0, Point3D(x2, y1, 0.0));
	add_to_polys(polys[area], poly);
    }
}


// Generate polygons from la and-cover raster.  Horizontally- or
// vertically-adjacent polygons will be merged automatically.
static int actual_load_landcover ( FGConstruct & c,
				   FGClipper &clipper ) {

    int count = 0;

    try {

        LandCover cover(c.get_cover());
        FGPolygon polys[FG_MAX_AREA_TYPES];
        FGPolygon poly;		// working polygon

        double dx = 1.0 / 120.0;
        double dy = dx;

        double half_dx = dx * 0.5;
        double half_dy = half_dx;

        double quarter_dx = dx * 0.25;
        double quarter_dy = quarter_dx;

        // Get the top corner of the tile
        double base_lon = c.get_bucket().get_center_lon()
            - 0.5 * c.get_bucket().get_width()
            - quarter_dx;
        double base_lat = c.get_bucket().get_center_lat()
            - 0.5 * c.get_bucket().get_height()
            - quarter_dy;

        cout << "DPM: tile at " << base_lon << ',' << base_lat << endl;
    
        double max_lon = c.get_bucket().get_center_lon() +
            (0.5 * c.get_bucket().get_width());
        double max_lat = c.get_bucket().get_center_lat() +
            (0.5 * c.get_bucket().get_height());

        // Figure out how many units wide and high this tile is; each unit
        // is 30 arc seconds.
        // int x_span = int(120 * bucket_span(base_lat)); // arcsecs of longitude
        // int y_span = int(120 * FG_BUCKET_SPAN); // arcsecs of latitude
        
        double x1 = base_lon;
        double y1 = base_lat;
        double x2 = x1 + dx;
        double y2 = y1 + dy;

        while ( x1 < max_lon ) {
            while ( y1 < max_lat ) {
                make_area( cover, polys, x1, y1, x2, y2, half_dx, half_dy );
                
                y1 = y2;
                y2 += dy;
            }

            x1 = x2;
            x2 += dx;
            y1 = base_lat;
            y2 = y1 + dy;
        }

        // Now that we're finished looking up land cover, we have a list
        // of lists of polygons, one (possibly-empty) list for each area
        // type.  Add the remaining polygons to the clipper.
        for ( int i = 0; i < FG_MAX_AREA_TYPES; i++ ) {
            if ( polys[i].contours() ) {
                clipper.add_poly( i, polys[i] );
                count++;
            }
        }
    } catch ( string e ) {
        cerr << "Died with exception: " << e << endl;
        exit(-1);
    }

    // Return the number of polygons actually read.
    return count;
}


// load all 2d polygons from the specified load disk directories and
// clip against each other to resolve any overlaps
static int load_polys( FGConstruct& c ) {
    FGClipper clipper;
    int i;

    string base = c.get_bucket().gen_base_path();
    string poly_path;
    int count = 0;

    // initialize clipper
    clipper.init();

    // load 2D polygons from all directories provided
    for ( i = 0; i < (int)load_dirs.size(); ++i ) {
	poly_path = load_dirs[i] + '/' + base;
	cout << "poly_path = " << poly_path << endl;
	count += actual_load_polys( poly_path, c, clipper );
	cout << "  loaded " << count << " total polys" << endl;
    }

    // Load the land use polygons if the --cover option was specified
    if ( c.get_cover().size() > 0 ) {
	count += actual_load_landcover (c, clipper);
    }

    point2d min, max;
    min.x = c.get_bucket().get_center_lon() - 0.5 * c.get_bucket().get_width();
    min.y = c.get_bucket().get_center_lat() - 0.5 * c.get_bucket().get_height();
    max.x = c.get_bucket().get_center_lon() + 0.5 * c.get_bucket().get_width();
    max.y = c.get_bucket().get_center_lat() + 0.5 * c.get_bucket().get_height();

    // do clipping
    cout << "clipping polygons" << endl;
    clipper.clip_all(min, max);

    // update main data repository
    c.set_clipped_polys( clipper.get_polys_clipped() );

    return count;
}


// Load elevation data from a DEM file, a regular grid of elevation
// data--dem based) and return list of fitted nodes.
static int load_dem( FGConstruct& c, FGArray& array) {
    point_list result;
    string base = c.get_bucket().gen_base_path();
    int i;

    for ( i = 0; i < (int)load_dirs.size(); ++i ) {
	string dem_path = load_dirs[i] + "/" + base
	    + "/" + c.get_bucket().gen_index_str() + ".dem";
	cout << "dem_path = " << dem_path << endl;

	if ( array.open(dem_path) ) {
	    cout << "Found DEM file " << dem_path << endl;
	    break;
	} else {
	    cout << "Failed to open DEM file " << dem_path << endl;
	}
    }

    SGBucket b = c.get_bucket();
    array.parse( b );

    return 1;
}


// fit dem nodes, return number of fitted nodes
static int fit_dem(FGArray& array, int error) {
    return array.fit( error );
}


// triangulate the data for each polygon ( first time before splitting )
static void first_triangulate( FGConstruct& c, const FGArray& array,
			       FGTriangle& t ) {
    // first we need to consolidate the points of the DEM fit list and
    // all the polygons into a more "Triangle" friendly format

    point_list corner_list = array.get_corner_node_list();
    point_list fit_list = array.get_fit_node_list();
    FGPolyList gpc_polys = c.get_clipped_polys();

    cout << "ready to build node list and polygons" << endl;
    t.build( corner_list, fit_list, gpc_polys );
    cout << "done building node list and polygons" << endl;

    cout << "ready to do triangulation" << endl;
    t.run_triangulate( c.get_angle(), 1 );
    cout << "finished triangulation" << endl;
}


// triangulate the data for each polygon ( second time after splitting
// and reassembling )
static void second_triangulate( FGConstruct& c, FGTriangle& t ) {
    t.rebuild( c );
    cout << "done re building node list and polygons" << endl;

    cout << "ready to do second triangulation" << endl;

    cout << "  (pre) nodes = " << c.get_tri_nodes().size() << endl;
    cout << "  (pre) normals = " << c.get_point_normals().size() << endl;

    t.run_triangulate( c.get_angle(), 2 );

    cout << "  (post) nodes = " << t.get_out_nodes().size() << endl;

    cout << "finished second triangulation" << endl;
}


// calculate distance based on x,y only
static double distance2D( const Point3D p1, const Point3D p2 ) {
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return sqrt( dx*dx + dy*dy );
}


// fix the elevations of the geodetic nodes
static void fix_point_heights( FGConstruct& c, const FGArray& array ) {
    int i;
    double z;

    cout << "fixing node heights" << endl;

    point_list raw_nodes = c.get_tri_nodes().get_node_list();

    for ( i = 0; i < (int)raw_nodes.size(); ++i ) {
	z = array.interpolate_altitude( raw_nodes[i].x() * 3600.0, 
					raw_nodes[i].y() * 3600.0 );
	// cout << "  old z = " << raw_nodes[i].z() << "  new z = " << z
	//      << endl;
	if ( raw_nodes[i].z() != z ) {
	    cout << "    DIFFERENT" << endl;
	}
	raw_nodes[i].setz( z );
    }

    cout << "flattening ocean connected nodes" << endl;

    triele_list tris = c.get_tri_elements();
    FGTriEle t;
    Point3D p;
    AreaType a;
    int n1, n2, n3;

    for ( int count = 0; count < 3; ++count ) {
	for ( i = 0; i < (int)tris.size(); ++i ) {
	    double e1, e2, e3, ave, min;

	    t = tris[i];
	    n1 = t.get_n1();
	    n2 = t.get_n2();
	    n3 = t.get_n3();
	    a = (AreaType)((int)(t.get_attribute()));

	    // scale elevation of all water nodes based on the average
	    // of the elevations of the nodes of the triangle of which
	    // they are a member.  This could really suck for certain
	    // cases, but it is my first stab at something reasonable.
	    // It might be better to eventually iterate, and allow
	    // some flexibility in elevations to handle rivers and
	    // things like that.
	    if ( (a == LakeArea) || (a == ReservoirArea) ) {
		e1 = raw_nodes[n1].z();
		e2 = raw_nodes[n2].z();
		e3 = raw_nodes[n3].z();

		min = e1; p = raw_nodes[n1];
		if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }
		if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }
		ave = (e1 + e2 + e3) / 3.0;
	    
		raw_nodes[n1].setz( min );
		raw_nodes[n2].setz( min );
		raw_nodes[n3].setz( min );
	    } else if ( (a == StreamArea) || (a == CanalArea) ) {
		e1 = raw_nodes[n1].z();
		e2 = raw_nodes[n2].z();
		e3 = raw_nodes[n3].z();

		min = e1; p = raw_nodes[n1];
		if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }
		if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }
	    
		double d1 = distance2D( p, raw_nodes[n1] ); 
		double d2 = distance2D( p, raw_nodes[n2] ); 
		double d3 = distance2D( p, raw_nodes[n3] ); 

		double max1 = 1000.0 * d1 + min;
		double max2 = 1000.0 * d2 + min;
		double max3 = 1000.0 * d3 + min;

		if ( max1 < e1 ) { raw_nodes[n1].setz( max1 ); }
		if ( max2 < e2 ) { raw_nodes[n2].setz( max2 ); }
		if ( max3 < e3 ) { raw_nodes[n3].setz( max3 ); }
	    }
	}
    }
 
    for ( i = 0; i < (int)tris.size(); ++i ) {
	// set all ocean nodes to 0.0
	t = tris[i];
	n1 = t.get_n1();
	n2 = t.get_n2();
	n3 = t.get_n3();
	a = (AreaType)((int)(t.get_attribute()));

	if ( a == OceanArea ) {
	    raw_nodes[n1].setz( 0.0 );
	    raw_nodes[n2].setz( 0.0 );
	    raw_nodes[n3].setz( 0.0 );
	}

    }

    FGTriNodes tmp;
    tmp.set_node_list( raw_nodes );
    c.set_tri_nodes( tmp );
}


// build the wgs-84 point list
static void build_wgs_84_point_list( FGConstruct& c, const FGArray& array ) {
    point_list geod_nodes;
    point_list wgs84_nodes;
    int i;

    cout << "generating wgs84 list" << endl;
    Point3D geod, radians, cart;

    point_list raw_nodes = c.get_tri_nodes().get_node_list();

    for ( i = 0; i < (int)raw_nodes.size(); ++i ) {
	geod = raw_nodes[i];

	// convert to radians
	radians = Point3D( geod.x() * SGD_DEGREES_TO_RADIANS,
			   geod.y() * SGD_DEGREES_TO_RADIANS,
			   geod.z() );

        cart = sgGeodToCart(radians);
	// cout << cart << endl;

	geod_nodes.push_back(geod);
        wgs84_nodes.push_back(cart);
    }

    c.set_geod_nodes( geod_nodes );
    c.set_wgs84_nodes( wgs84_nodes );
}


// build the node -> element (triangle) reverse lookup table.  there
// is an entry for each point containing a list of all the triangles
// that share that point.
static belongs_to_list gen_node_ele_lookup_table( FGConstruct& c ) {
    belongs_to_list reverse_ele_lookup;
    reverse_ele_lookup.clear();

    int_list ele_list;
    ele_list.clear();

    // initialize reverse_ele_lookup structure by creating an empty
    // list for each point
    point_list wgs84_nodes = c.get_wgs84_nodes();
    const_point_list_iterator w_current = wgs84_nodes.begin();
    const_point_list_iterator w_last = wgs84_nodes.end();
    for ( ; w_current != w_last; ++w_current ) {
	reverse_ele_lookup.push_back( ele_list );
    }

    // traverse triangle structure building reverse lookup table
    triele_list tri_elements = c.get_tri_elements();
    const_triele_list_iterator current = tri_elements.begin();
    const_triele_list_iterator last = tri_elements.end();
    int counter = 0;
    for ( ; current != last; ++current ) {
	reverse_ele_lookup[ current->get_n1() ].push_back( counter );
	reverse_ele_lookup[ current->get_n2() ].push_back( counter );
	reverse_ele_lookup[ current->get_n3() ].push_back( counter );
	++counter;
    }

    return reverse_ele_lookup;
}


// caclulate the area for the specified triangle face
static double tri_ele_area( const FGConstruct& c, const FGTriEle tri ) {
    point_list nodes = c.get_geod_nodes();

    Point3D p1 = nodes[ tri.get_n1() ];
    Point3D p2 = nodes[ tri.get_n2() ];
    Point3D p3 = nodes[ tri.get_n3() ];

    return triangle_area( p1, p2, p3 );
}


// caclulate the normal for the specified triangle face
static Point3D calc_normal( FGConstruct& c, int i ) {
    sgVec3 v1, v2, normal;

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
    if ( fabs(p1.x() - p2.x()) < SG_EPSILON &&
	 fabs(p1.x() - p3.x()) < SG_EPSILON ) {
	degenerate = true;
    }
    if ( fabs(p1.y() - p2.y()) < SG_EPSILON &&
	 fabs(p1.y() - p3.y()) < SG_EPSILON ) {
	degenerate = true;
    }
    if ( fabs(p1.z() - p2.z()) < SG_EPSILON &&
	 fabs(p1.z() - p3.z()) < SG_EPSILON ) {
	degenerate = true;
    }

    if ( degenerate ) {
	sgSetVec3( normal, p1.x(), p1.y(), p1.z() );
	sgNormalizeVec3( normal );
	cout << "Degenerate tri!" << endl;
    } else {
	v1[0] = p2.x() - p1.x();
	v1[1] = p2.y() - p1.y();
	v1[2] = p2.z() - p1.z();
	v2[0] = p3.x() - p1.x();
	v2[1] = p3.y() - p1.y();
	v2[2] = p3.z() - p1.z();

	sgVectorProductVec3( normal, v1, v2 );
	sgNormalizeVec3( normal );
    }

    return Point3D( normal[0], normal[1], normal[2] );
}


// build the face normal list
static point_list gen_face_normals( FGConstruct& c ) {
    point_list face_normals;

    // traverse triangle structure building the face normal table

    cout << "calculating face normals" << endl;

    triele_list tri_elements = c.get_tri_elements();
    for ( int i = 0; i < (int)tri_elements.size(); i++ ) {
	Point3D p = calc_normal(c,  i );
	cout << p << endl;
	face_normals.push_back( p );
    }

    return face_normals;
}


// calculate the normals for each point in wgs84_nodes
static point_list gen_point_normals( FGConstruct& c ) {
    point_list point_normals;

    Point3D normal;
    cout << "calculating node normals" << endl;

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
	cout << "average = " << average << endl;

	point_normals.push_back( average );
    }

    cout << "1st" << endl;
    cout << "wgs84 node list size = " << wgs84_nodes.size() << endl;
    cout << "normal list size = " << point_normals.size() << endl;

    return point_normals;
}


// generate the flight gear scenery file
static void do_output( FGConstruct& c, FGGenOutput& output ) {
    output.build( c );
    output.write( c );
}


// collect custom objects and move to scenery area
static void do_custom_objects( const FGConstruct& c ) {
    SGBucket b = c.get_bucket();

    // Create/open the output .stg file for writing
    string output_base = c.get_output_base();
    string dest_dir = output_base + "/Scenery/" + b.gen_base_path();
    string dest_ind = dest_dir + "/" + b.gen_index_str() + ".stg";

    FILE *fp;
    if ( (fp = fopen( dest_ind.c_str(), "w" )) == NULL ) {
        cout << "ERROR: opening " << dest_ind << " for writing!" << endl;
        exit(-1);
    }

    // Start with the default custom object which is the base terrain
    fprintf(fp, "OBJECT_BASE %s.btg\n", b.gen_index_str().c_str());

    for ( int i = 0; i < (int)load_dirs.size(); ++i ) {
	string base_dir = load_dirs[i] + "/" + b.gen_base_path();
	string index_file = base_dir + "/" + b.gen_index_str() + ".ind";
	cout << "collecting custom objects from " << index_file << endl;

	sg_gzifstream in( index_file );

	if ( ! in.is_open() ) {
	    cout << "No custom objects" << endl;
	} else {
	    string token, name;

	    while ( ! in.eof() ) {
		in >> token;
		in >> name;
		in >> skipws;

		cout << "token = " << token << " name = " << name << endl;
#ifdef _MSC_VER
		string command = "copy " + base_dir + "/" + name + ".gz "
		    + dest_dir;
#else
		string command = "cp " + base_dir + "/" + name + ".gz "
		    + dest_dir;
#endif
		cout << "running " << command << endl;
		system( command.c_str() );

		fprintf(fp, "OBJECT %s\n", name.c_str());
	    }
	}
    }

    fclose(fp);
}

// master construction routine
static void construct_tile( FGConstruct& c ) {
    cout << "Construct tile, bucket = " << c.get_bucket() << endl;

    // fit with ever increasing error tolerance until we produce <=
    // 80% of max nodes.  We should really have the sim end handle
    // arbitrarily complex tiles.

    bool acceptable = false;
    bool growing = false;
    bool shrinking = false;

    double error = 200.0;
    int count = 0;

    // load and clip 2d polygon data
    if ( load_polys( c ) == 0 ) {
	// don't build the tile if there is no 2d data ... it *must*
	// be ocean and the sim can build the tile on the fly.
	return;
    }

    // load grid of elevation data (dem)
    FGArray array;
    load_dem( c, array );

    FGTriangle t;

    while ( ! acceptable ) {
	// do a least squares fit of the (dem) data with the given
	// error tolerance
	array.fit( error );

	// triangulate the data for each polygon
	first_triangulate( c, array, t );

	acceptable = true;

	count = t.get_out_nodes_size();

	if ( (count < c.get_min_nodes()) && (error >= 25.0) ) {
	    // reduce error tolerance until number of points exceeds the
	    // minimum threshold
	    cout << "produced too few nodes ..." << endl;

	    acceptable = false;
	    growing = true;

	    if ( shrinking ) {
		error /= 1.25;
		shrinking = false;
	    } else {
		error /= 1.5;
	    }
	    cout << "Setting error to " << error << " and retrying fit." 
		 << endl;
	}

	if ( count > c.get_max_nodes() ) {
	    if ( error <= 1000.0 ) {
		// increase error tolerance until number of points drops below
		// the maximum threshold
		cout << "produced too many nodes ..." << endl;
	    
		acceptable = false;
		shrinking = true;

		if ( growing ) {
		    error *= 1.25;
		    growing = false;
		} else {
		    error *= 1.5;
		}

		cout << "Setting error to " << error << " and retrying fit." 
		     << endl;
	    } else {
		// we tried, but can't seem to get down to a
		// reasonable number of points even with a huge error
		// tolerance.  This could be related to the triangle()
		// call which might be having trouble with our input
		// set.  Let's just die hope that our parent can try
		// again with a smaller interior triangle angle.
		cout << "Error:  Too many nodes." << endl;
		exit(-1);
	    }
	}
    }

    cout << "finished fit with error = " << error << " node count = " 
	 << count << endl;

    // save the results of the triangulation
    c.set_tri_nodes( t.get_out_nodes() );
    c.set_tri_elements( t.get_elelist() );
    c.set_tri_segs( t.get_out_segs() );

    // calculate wgs84 (cartesian) form of node list
    fix_point_heights( c, array );
    build_wgs_84_point_list( c, array );

    // build the node -> element (triangle) reverse lookup table
    c.set_reverse_ele_lookup( gen_node_ele_lookup_table( c ) );

    // build the face normal list
    c.set_face_normals( gen_face_normals( c ) );

    // calculate the normals for each point in wgs84_nodes
    c.set_point_normals( gen_point_normals( c ) );

    // match tile edges with any neighbor tiles that have already been
    // generated
    FGMatch m;
    m.load_neighbor_shared( c );
    m.split_tile( c );
    m.write_shared( c );
    m.assemble_tile( c );

    // now we must retriangulate the pasted together tile points
    second_triangulate( c, t );

    // save the results of the triangulation
    c.set_tri_nodes( t.get_out_nodes() );
    c.set_tri_elements( t.get_elelist() );
    c.set_tri_segs( t.get_out_segs() );

    // double check on the off chance that the triangulator was forced
    // to introduce additional points
    if ( c.get_tri_nodes().size() > c.get_point_normals().size() ) {
	cout << "oops, need to add normals :-(" << endl;
	point_list normals = c.get_point_normals();
	int start = normals.size();
	int end = c.get_tri_nodes().size();
	for ( int i = start; i < end; ++i ) {
	    cout << "adding a normal for " << c.get_tri_nodes().get_node(i)
		 << endl;
	    Point3D p = tgFakeNormal( c.get_tri_nodes().get_node(i) );
	    normals.push_back( p );
	}
	c.set_point_normals( normals );
    }

    // calculate wgs84 (cartesian) form of node list
    build_wgs_84_point_list( c, array );

    // generate the output
    FGGenOutput output;
    do_output( c, output );

    array.close();

    // collect custom objects and move to scenery area
    do_custom_objects( c );
}


// display usage and exit
static void usage( const string name ) {
    cout << "Usage: " << name << endl;
    cout << "[ --output-dir=<directory>" << endl;
    cout << "  --work-dir=<directory>" << endl;
    cout << "  --cover=<path to land-cover raster>" << endl;
    cout << "  --min-angle=<angle>" << endl;
    cout << "  --tile-id=<id>" << endl;
    cout << "  --lon=<degrees>" << endl;
    cout << "  --lat=<degrees>" << endl;
    cout << "  --xdist=<degrees>" << endl;
    cout << "  --ydist=<degrees>" << endl;
    cout << "  --useUKgrid" << endl;
    cout << " ] <load directory...>" << endl;
    exit(-1);
}


int main(int argc, char **argv) {
    string output_dir = ".";
    string work_dir = ".";
    string min_angle = "10";
    string cover = "";
    double lon = -110.664244;	// P13
    double lat = 33.352890;
    double xdist = -1;		// 1/2 degree in each direction
    double ydist = -1;
    long tile_id = -1;

    // flag indicating whether UK grid should be used for in-UK
    // texture coordinate generation
    bool useUKgrid = false;
    
    sglog().setLogLevels( SG_ALL, SG_DEBUG );

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
	} else if (arg.find("--min-angle=") == 0) {
	    min_angle = arg.substr(12);
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
	} else if (arg.find("--cover=") == 0) {
	    cover = arg.substr(8);
	} else if (arg.find("--useUKgrid") == 0) {
		useUKgrid = true;
	} else if (arg.find("--") == 0) {
	    usage(argv[0]);
	} else {
	    break;
	}
    }

    cout << "Output directory is " << output_dir << endl;
    cout << "Working directory is " << work_dir << endl;
    cout << "Minimum angle is " << min_angle << endl;
    cout << "Tile id is " << tile_id << endl;
    cout << "Center longitude is " << lon << endl;
    cout << "Center latitude is " << lat << endl;
    cout << "X distance is " << xdist << endl;
    cout << "Y distance is " << ydist << endl;
    for (int i = arg_pos; i < argc; i++) {
	load_dirs.push_back(argv[i]);
	cout << "Load directory: " << argv[i] << endl;
    }

#if defined( __CYGWIN__ ) || defined( __CYGWIN32__ ) || defined( _MSC_VER )
    // the next bit crashes Cygwin for me - DCL
    // MSVC does not have the function or variable type defined - BRF
#else
    // set mem allocation limit.  Reason: occasionally the triangle()
    // routine can blow up and allocate memory forever.  We'd like
    // this process to die before things get out of hand so we can try
    // again with a smaller interior angle limit.
    int result;
    struct rlimit limit;
    limit.rlim_cur = 20000000;
    limit.rlim_max = 20000000;

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
    limit.rlim_cur = 120;
    limit.rlim_max = 120;
    result = setrlimit( RLIMIT_CPU, &limit );
    cout << "result of setting mem limit = " << result << endl;
#endif  // end of stuff that crashes Cygwin

    // main construction data management class
    FGConstruct c;

    c.set_angle( min_angle );
    c.set_cover( cover );
    c.set_work_base( work_dir );
    c.set_output_base( output_dir );
    c.set_useUKGrid( useUKgrid );

    c.set_min_nodes( 50 );
    c.set_max_nodes( (int)(FG_MAX_NODES * 0.8) );

    if (tile_id == -1) {
	if (xdist == -1 || ydist == -1) {
	    // construct the tile around the specified location
	    cout << "Building single tile at " << lat << ',' << lon << endl;
	    SGBucket b( lon, lat );
	    c.set_bucket( b );
	    construct_tile( c );
	} else {
	    // build all the tiles in an area

	    cout << "Building tile(s) at " << lat << ',' << lon
		 << " with x distance " << xdist
		 << " and y distance " << ydist << endl;
	    double min_x = lon - xdist;
	    double min_y = lat - ydist;
	    SGBucket b_min( min_x, min_y );
	    SGBucket b_max( lon + xdist, lat + ydist );

	    SGBucket b_start(550401L);
	    bool do_tile = true;

	    if ( b_min == b_max ) {
		c.set_bucket( b_min );
		construct_tile( c );
	    } else {
		SGBucket b_cur;
		int dx, dy, i, j;
	    
		sgBucketDiff(b_min, b_max, &dx, &dy);
		cout << "  construction area spans tile boundaries" << endl;
		cout << "  dx = " << dx << "  dy = " << dy << endl;

		for ( j = 0; j <= dy; j++ ) {
		    for ( i = 0; i <= dx; i++ ) {
			b_cur = sgBucketOffset(min_x, min_y, i, j);

			if ( b_cur == b_start ) {
			    do_tile = true;
			}

			if ( do_tile ) {
			    c.set_bucket( b_cur );
			    construct_tile( c );
			} else {
			    cout << "skipping " << b_cur << endl;
			}
		    }
		}
		// string answer; cin >> answer;
	    }
	}
    } else {
	// construct the specified tile
        cout << "Building tile " << tile_id << endl;
	SGBucket b( tile_id );
	c.set_bucket( b );
	construct_tile( c );
    }

    cout << "[Finished successfully]" << endl;
    return 0;
}
