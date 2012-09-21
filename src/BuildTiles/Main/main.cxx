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

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_dir.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/math/SGMath.hxx>

#include <boost/foreach.hpp>

#include <Geometry/poly_support.hxx>
#include <Array/array.hxx>
#include <Clipper/clipper.hxx>
#include <GenOutput/genobj.hxx>
#include <Match/match.hxx>
#include <Triangulate/triangle.hxx>
#include <landcover/landcover.hxx>

#include "construct.hxx"
#include "usgs.hxx"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;

vector<string> load_dirs;


static const double cover_size = 1.0 / 120.0;
static const double half_cover_size = cover_size * 0.5;

// If we don't offset land use squares by some amount, then we can get
// land use square boundaries coinciding with tile boundaries.
//
// This can foul up our edge matching scheme because a subsequently
// generated adjacent tile might be forced to have edge nodes not
// found in the first tile and not on the shared edge.  This can lead
// to gaps.  If we put skirts around everything that might hide the
// problem.
static const double quarter_cover_size = cover_size * 0.25;

double nudge=0.0;


// Scan a directory and load polygon files.
static int actual_load_polys( const SGPath& dir,
			      TGConstruct& c,
			      TGClipper& clipper ) {
    int counter = 0;
    string tile_str = c.get_bucket().gen_index_str();

    simgear::Dir d(dir);
    if (!d.exists()) {
        cout << "directory not found:" << dir.str() << "\n";
	    return 0;
    }
    
    BOOST_FOREACH(const SGPath& p, d.children(simgear::Dir::TYPE_FILE)) {
        if (p.file_base() != tile_str) {
            continue;
        }
        
        string lext = p.complete_lower_extension();
        if ((lext == "arr") || (lext == "arr.gz") || (lext == "btg.gz") ||
            (lext == "fit") || (lext == "fit.gz") || (lext == "ind")) 
        {
            // skipped!
        } else if (lext == "osgb36") {
            cout << "Loading osgb36 poly definition file\n";
            clipper.load_osgb36_polys( p.str() );
            ++counter;
        } else {
            cout << "ext = '" << lext << "'" << endl;
            clipper.load_polys( p.str() );
            ++counter;
        }
    } // of directory file children
	
    return counter;
}


// Add a polygon to a list, merging if possible.
//
// Merge a polygon with an existing one if possible, append a new one
// otherwise; this function is used by actual_load_landcover, below,
// to reduce the number of separate polygons.
static void inline add_to_polys ( TGPolygon &accum, const TGPolygon &poly) {
    if ( accum.contours() > 0 ) {
	accum = tgPolygonUnion( accum, poly );
    } else {
	accum = poly;
    }
}


static AreaType get_area_type (const LandCover &cover,
			       double xpos, double ypos,
			       double dx, double dy)
{
    // Look up the land cover for the square
    int cover_value = cover.getValue(xpos, ypos);
    AreaType area = translateUSGSCover(cover_value);

    if ( area != get_default_area_type() ) {
        // Non-default area is fine.
        return area;
    } else {
        // If we're stuck with the default area, try to borrow from a
        // neighbour.
        for (double x = xpos - dx; x <= xpos + dx; x += dx) {
            for (double y = ypos - dy; y < ypos + dx; y += dy) {
                if (x != xpos || y != ypos) {
                    cover_value = cover.getValue(x, y);
                    area = translateUSGSCover(cover_value);
                    if (area != get_default_area_type() ) {
                        return area;
                    }
                }
            }
        }
    }

    // OK, give up and return default
    return get_default_area_type();
}


// Come up with a "rough" metric for the roughness of the terrain
// coverted by a polygon
static double measure_roughness( const TGArray &array, TGPolygon &poly ) {
    int i;
    unsigned int j;

    // find the elevation range
    double max_z = -9999.0;
    double min_z = 9999.0;

    for ( i = 0; i < poly.contours(); ++i ) {
        point_list points = poly.get_contour( i );
        for ( j = 0; j < points.size(); ++j ) {
            double z;
            z = array.altitude_from_grid( points[j].x() * 3600.0, 
                                          points[j].y() * 3600.0 );
            if ( z < -9000 ) {
                z = array.closest_nonvoid_elev( points[j].x() * 3600.0, 
                                                points[j].y() * 3600.0 );
            }
            // cout << "elevation = " << z << endl;
            if ( z < min_z ) {
                min_z = z;
            }
            if ( z > max_z ) {
                max_z = z;
            }
        }
    }

    double diff = max_z - min_z;

    // 50m difference in polygon elevation range yields a roughness
    // metric of 1.0.  Less than 1.0 is relatively flat.  More than
    // 1.0 is relatively rough.

    cout << "roughness = " << diff / 50.0 << endl;

    return diff / 50.0;
}


// make the area specified area, look up the land cover type, and add
// it to polys
static void make_area( const LandCover &cover, const TGArray &array,
                       TGPolygon *polys,
		       double x1, double y1, double x2, double y2,
		       double half_dx, double half_dy )
{
    const double fudge = 0.0001;  // (0.0001 degrees =~ 10 meters)

    AreaType area = get_area_type( cover,
                                   x1 + half_dx, y1 + half_dy,
                                   x2 - x1, y2 - y1 );
    if ( area != get_default_area_type() ) {
	// Create a square polygon and merge it into the list.
	TGPolygon poly;
	poly.erase();
	poly.add_node(0, Point3D(x1 - fudge, y1 - fudge, 0.0));
	poly.add_node(0, Point3D(x1 - fudge, y2 + fudge, 0.0));
	poly.add_node(0, Point3D(x2 + fudge, y2 + fudge, 0.0));
	poly.add_node(0, Point3D(x2 + fudge, y1 - fudge, 0.0));

        if ( measure_roughness( array, poly ) < 1.0 ) {
            add_to_polys(polys[area], poly);
        }
    }
}


// Generate polygons from land-cover raster.  Horizontally- or
// vertically-adjacent polygons will be merged automatically.
static int actual_load_landcover ( TGConstruct &c, const TGArray &array,
				   TGClipper &clipper )
{
    int count = 0;

    try {

        LandCover cover(c.get_cover());
        TGPolygon polys[TG_MAX_AREA_TYPES];
        TGPolygon poly;		// working polygon

        // Get the lower left (SW) corner of the tile
        double base_lon = c.get_bucket().get_center_lon()
            - 0.5 * c.get_bucket().get_width()
            - quarter_cover_size;
        double base_lat = c.get_bucket().get_center_lat()
            - 0.5 * c.get_bucket().get_height()
            - quarter_cover_size;

        cout << "raster land cover: tile at "
             << base_lon << ',' << base_lat << endl;
    
        double max_lon = c.get_bucket().get_center_lon()
            + 0.5 * c.get_bucket().get_width();
        double max_lat = c.get_bucket().get_center_lat()
            + 0.5 * c.get_bucket().get_height();

        cout << "raster land cover: extends to "
             << max_lon << ',' << max_lat << endl;

        // cout << "raster land cover: width = " << c.get_bucket().get_width()
        //      << " height = " << c.get_bucket().get_height() << endl;

        // cout << "cover_size = " << cover_size << endl;

        double x1 = base_lon;
        double y1 = base_lat;
        double x2 = x1 + cover_size;
        double y2 = y1 + cover_size;

        while ( x1 < max_lon ) {
            while ( y1 < max_lat ) {
                make_area( cover, array, polys,
                           x1, y1, x2, y2, half_cover_size, half_cover_size );
                
                y1 = y2;
                y2 += cover_size;
            }

            x1 = x2;
            x2 += cover_size;
            y1 = base_lat;
            y2 = y1 + cover_size;
        }

        // Now that we're finished looking up land cover, we have a list
        // of lists of polygons, one (possibly-empty) list for each area
        // type.  Add the remaining polygons to the clipper.
        for ( int i = 0; i < TG_MAX_AREA_TYPES; i++ ) {
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
static int load_polys( TGConstruct& c, const TGArray &array ) {
    TGClipper clipper;
    int i;

    string base = c.get_bucket().gen_base_path();
    string poly_path;
    int count = 0;
    
    clipper.nudge = nudge;
    clipper.ignore_landmass( c.get_ignore_landmass() );

    // initialize clipper
    clipper.init();

    // load 2D polygons from all directories provided
    for ( i = 0; i < (int)load_dirs.size(); ++i ) {
	poly_path = c.get_work_base() + "/" + load_dirs[i] + '/' + base;
	cout << "poly_path = " << poly_path << endl;
	count += actual_load_polys( poly_path, c, clipper );
	cout << "  loaded " << count << " total polys" << endl;
    }

    // Load the land use polygons if the --cover option was specified
    if ( c.get_cover().size() > 0 ) {
	count += actual_load_landcover (c, array, clipper);
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
    c.set_fixed_elevations( clipper.get_fixed_elevations() );

    return count;
}


// Load elevation data from an Array file, a regular grid of elevation
// data) and return list of fitted nodes.
static bool load_array( TGConstruct& c, TGArray& array) {
    string base = c.get_bucket().gen_base_path();
    int i;

    for ( i = 0; i < (int)load_dirs.size(); ++i ) {
	string array_path = c.get_work_base() + "/" + load_dirs[i] + "/" + base
	    + "/" + c.get_bucket().gen_index_str();
	cout << "array_path = " << array_path << endl;

	if ( array.open(array_path) ) {
	    cout << "Found Array file " << array_path << endl;
	    break;
	} else {
	    cout << "Failed to open Array file " << array_path << endl;
	}
    }

    SGBucket b = c.get_bucket();
    array.parse( b );
    array.remove_voids();

    return true;
}


// triangulate the data for each polygon ( first time before splitting )
static void first_triangulate( TGConstruct& c, const TGArray& array,
			       TGTriangle& t ) {
    // first we need to consolidate the points of the Array fit list and
    // all the polygons into a more "Triangle" friendly format

    point_list corner_list = array.get_corner_list();
    point_list fit_list = array.get_fitted_list();
    TGPolyList gpc_polys = c.get_clipped_polys();

    cout << "ready to build node list and polygons" << endl;
    t.build( corner_list, fit_list, gpc_polys );
    cout << "done building node list and polygons" << endl;

    cout << "ready to do triangulation" << endl;
    t.run_triangulate( 0.0, 1 );
    cout << "finished triangulation" << endl;
}


// triangulate the data for each polygon ( second time after splitting
// and reassembling )
static void second_triangulate( TGConstruct& c, TGTriangle& t ) {
    t.rebuild( c );
    cout << "done re building node list and polygons" << endl;

    cout << "ready to do second triangulation" << endl;

    cout << "  (pre) nodes = " << c.get_tri_nodes().size() << endl;
    cout << "  (pre) normals = " << c.get_point_normals().size() << endl;

    t.run_triangulate( 0.0, 2 );

    cout << "  (post) nodes = " << t.get_out_nodes().size() << endl;

    cout << "finished second triangulation" << endl;
}


inline void calc_gc_course_dist( const Point3D& start, const Point3D& dest, 
                                 double *course, double *dist )
{
  SGGeoc gs = start.toSGGeoc();
  SGGeoc gd = dest.toSGGeoc();
  *course = SGGeoc::courseRad(gs, gd);
  *dist = SGGeoc::distanceM(gs, gd);
}

// calculate spherical distance between two points (lon, lat specified
// in degrees, result returned in meters)
static double distanceSphere( const Point3D p1, const Point3D p2 ) {
    Point3D r1( p1.x() * SG_DEGREES_TO_RADIANS,
                p1.y() * SG_DEGREES_TO_RADIANS,
                p1.z() );
    Point3D r2( p2.x() * SG_DEGREES_TO_RADIANS,
                p2.y() * SG_DEGREES_TO_RADIANS,
                p2.z() );

    double course, dist_m;
    calc_gc_course_dist( r1, r2, &course, &dist_m );

    return dist_m;
}


// fix the elevations of the geodetic nodes
static void fix_point_heights( TGConstruct& c, const TGArray& array )
{
    int i;
    double z;

    cout << "fixing node heights" << endl;

    point_list raw_nodes = c.get_tri_nodes().get_node_list();

    for ( i = 0; i < (int)raw_nodes.size(); ++i ) {
        //cout << "  fixing = " << raw_nodes[i] << " ";
        int index = c.get_fixed_elevations().find( raw_nodes[i] );
        if ( index >= 0 ) {
            // found an elevation we want to preserve
            z = c.get_fixed_elevations().get_node(index).z();
            //cout << " forced = " << z << endl;
        } else {
            // interpolate point from DEM data.
            z = array.altitude_from_grid( raw_nodes[i].x() * 3600.0, 
                                          raw_nodes[i].y() * 3600.0 );
            //cout << " interpolated = " << z << endl;
        }
        raw_nodes[i].setz( z );
    }

    triele_list tris = c.get_tri_elements();
    TGTriEle t;
    Point3D p;
    AreaType a;
    int n1, n2, n3;

    cout << "flattening lake connected nodes (and smoothing streams)" << endl;

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
	    if ( is_lake_area( a ) ) {
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
	    } else if ( is_stream_area( a ) ) {
		e1 = raw_nodes[n1].z();
		e2 = raw_nodes[n2].z();
		e3 = raw_nodes[n3].z();

		min = e1; p = raw_nodes[n1];
		if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }
		if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }
	    
		double d1 = distanceSphere( p, raw_nodes[n1] ); 
		double d2 = distanceSphere( p, raw_nodes[n2] ); 
		double d3 = distanceSphere( p, raw_nodes[n3] ); 

		double max1 = d1 * 0.20 + min;
		double max2 = d2 * 0.20 + min;
		double max3 = d3 * 0.20 + min;

		if ( max1 < e1 ) { raw_nodes[n1].setz( max1 ); }
		if ( max2 < e2 ) { raw_nodes[n2].setz( max2 ); }
		if ( max3 < e3 ) { raw_nodes[n3].setz( max3 ); }
	    } else if ( is_road_area( a ) ) {
		e1 = raw_nodes[n1].z();
		e2 = raw_nodes[n2].z();
		e3 = raw_nodes[n3].z();

		min = e1; p = raw_nodes[n1];
		if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }
		if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }
	    
		double d1 = distanceSphere( p, raw_nodes[n1] ); 
		double d2 = distanceSphere( p, raw_nodes[n2] ); 
		double d3 = distanceSphere( p, raw_nodes[n3] ); 

		double max1 = d1 * 0.30 + min;
		double max2 = d2 * 0.30 + min;
		double max3 = d3 * 0.30 + min;

		if ( max1 < e1 ) { raw_nodes[n1].setz( max1 ); }
		if ( max2 < e2 ) { raw_nodes[n2].setz( max2 ); }
		if ( max3 < e3 ) { raw_nodes[n3].setz( max3 ); }
	    }
	}
    }
 
    cout << "flattening ocean connected nodes" << endl;

    for ( i = 0; i < (int)tris.size(); ++i ) {
	// set all ocean nodes to 0.0
	t = tris[i];
	n1 = t.get_n1();
	n2 = t.get_n2();
	n3 = t.get_n3();
	a = (AreaType)((int)(t.get_attribute()));

	if ( is_ocean_area( a ) ) {
	    raw_nodes[n1].setz( 0.0 );
	    raw_nodes[n2].setz( 0.0 );
	    raw_nodes[n3].setz( 0.0 );
	}

    }

    TGTriNodes tmp;
    tmp.set_node_list( raw_nodes );
    c.set_tri_nodes( tmp );
}


// For each triangle assigned to the "default" area type, see if we
// can lookup a better land cover type from the 1km data structure.
static void fix_land_cover_assignments( TGConstruct& c ) {
    cout << "Fixing up default land cover types" << endl;
    // the list of node locations
    TGTriNodes trinodes = c.get_tri_nodes();
    point_list geod_nodes = trinodes.get_node_list();

    // the list of triangles (with area type attribute)
    triele_list tri_elements = c.get_tri_elements();

    // traverse the triangle element groups
    cout << "  Total Nodes = " << geod_nodes.size() << endl;
    cout << "  Total triangles = " << tri_elements.size() << endl;
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


// build the wgs-84 point list
static void build_wgs_84_point_list( TGConstruct& c, const TGArray& array ) {
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
static belongs_to_list gen_node_ele_lookup_table( TGConstruct& c ) {
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
static double tri_ele_area( const TGConstruct& c, const TGTriEle tri ) {
    point_list nodes = c.get_geod_nodes();

    Point3D p1 = nodes[ tri.get_n1() ];
    Point3D p2 = nodes[ tri.get_n2() ];
    Point3D p3 = nodes[ tri.get_n3() ];

    return triangle_area( p1, p2, p3 );
}


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
        normal = normalize(SGVec3d(p1.x(), p1.y(), p1.z()));
	    cout << "Degenerate tri!" << endl;
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


// build the face normal list
static point_list gen_face_normals( TGConstruct& c ) {
    point_list face_normals;

    // traverse triangle structure building the face normal table

    cout << "calculating face normals" << endl;

    triele_list tri_elements = c.get_tri_elements();
    for ( int i = 0; i < (int)tri_elements.size(); i++ ) {
	Point3D p = calc_normal(c,  i );
	//cout << p << endl;
	face_normals.push_back( p );
    }

    return face_normals;
}


// calculate the normals for each point in wgs84_nodes
static point_list gen_point_normals( TGConstruct& c ) {
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
	//cout << "average = " << average << endl;

	point_normals.push_back( average );
    }

    cout << "1st" << endl;
    cout << "wgs84 node list size = " << wgs84_nodes.size() << endl;
    cout << "normal list size = " << point_normals.size() << endl;

    return point_normals;
}


// generate the flight gear scenery file
static void do_output( TGConstruct& c, TGGenOutput& output ) {
    output.build( c );
    output.write( c );
}


// collect custom objects and move to scenery area
static void do_custom_objects( const TGConstruct& c ) {
    SGBucket b = c.get_bucket();

    // Create/open the output .stg file for writing
    SGPath dest_d(c.get_output_base().c_str());
    dest_d.append(b.gen_base_path().c_str());
    string dest_dir = dest_d.str_native();
    SGPath dest_i(dest_d);
    dest_i.append(b.gen_index_str());
    dest_i.concat(".stg");
    string dest_ind = dest_i.str_native();

    FILE *fp;
    if ( (fp = fopen( dest_ind.c_str(), "w" )) == NULL ) {
        cout << "ERROR: opening " << dest_ind << " for writing!" << endl;
        exit(-1);
    }

    // Start with the default custom object which is the base terrain
    fprintf(fp, "OBJECT_BASE %s.btg\n", b.gen_index_str().c_str());

    char line[2048];             // big enough?
    char token[256];
    char name[256];

    for ( int i = 0; i < (int)load_dirs.size(); ++i ) {
        SGPath base(c.get_work_base().c_str());
        base.append(load_dirs[i]);
        base.append( b.gen_base_path() );
        SGPath index(base);
        index.append( b.gen_index_str() );
        index.concat(".ind");
        string index_file = index.str_native();
        //cout << "collecting custom objects from " << index_file << endl;

        sg_gzifstream in( index_file );

        if ( ! in.is_open() ) {
            //cout << "No custom objects" << endl;
        } else {
            while ( ! in.eof() ) {
                cout << "Collecting custom objects from " << index_file << endl;
                in.getline(line, 2048);
                cout << "line = " << line << endl;

                int result = sscanf( line, "%s %s", token, name );
                cout << "scanf scanned " << result << " tokens" << endl;

                if ( result > 0 ) {
                    cout << "token = " << token << " name = " << name << endl;

                    if ( strcmp( token, "OBJECT" ) == 0 ) {
                        SGPath srcbase(base);
                        srcbase.append(name);
                        srcbase.concat(".gz");
                        string basecom = srcbase.str_native();
#ifdef _MSC_VER
                        string command = "copy " + basecom + " " + dest_dir;
#else
                        string command = "cp " + basecom + " " + dest_dir;
#endif
                        SG_LOG( SG_GENERAL, SG_INFO, "running " << command );
                        system( command.c_str() );

                        fprintf(fp, "OBJECT %s\n", name);
                    } else {
                        fprintf(fp, "%s\n", line);
                    }
                }
            }
        }
    }

    fclose(fp);
}

// master construction routine
static void construct_tile( TGConstruct& c ) {
    cout << "Construct tile, bucket = " << c.get_bucket() << endl;

    // load grid of elevation data (Array)
    TGArray array;
    load_array( c, array );

    // load and clip 2d polygon data
    if ( load_polys( c, array ) == 0 ) {
	// don't build the tile if there is no 2d data ... it *must*
	// be ocean and the sim can build the tile on the fly.
	return;
    }

    TGTriangle t;

    // triangulate the data for each polygon
    first_triangulate( c, array, t );

    cout << "number of fitted nodes = " << t.get_out_nodes_size() << endl;

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
    TGMatch m;
    m.load_neighbor_shared( c );
    if ( c.get_use_own_shared_edges() ) {
            m.load_missing_shared( c );
    }
    m.split_tile( c );
    if ( c.get_write_shared_edges() ) {
            m.write_shared( c );
    }
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

    if ( c.get_cover().size() > 0 ) {
        // Now for all the remaining "default" land cover polygons, assign
        // each one it's proper type from the land use/land cover
        // database.
        fix_land_cover_assignments( c );
    }

    // calculate wgs84 (cartesian) form of node list
    build_wgs_84_point_list( c, array );

    // generate the output
    TGGenOutput output;
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
    cout << "  --tile-id=<id>" << endl;
    cout << "  --lon=<degrees>" << endl;
    cout << "  --lat=<degrees>" << endl;
    cout << "  --xdist=<degrees>" << endl;
    cout << "  --ydist=<degrees>" << endl;
    cout << "  --nudge=<float>" << endl;
    cout << "  --priorities=<filename>" << endl;
    cout << "  --usgs-map=<filename>" << endl;
    cout << "  --useUKgrid" << endl;
    cout << "  --no-write-shared-edges" << endl;
    cout << "  --use-own-shared-edges" << endl;
    cout << "  --ignore-landmass" << endl;
    cout << " ] <load directory...>" << endl;
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

    cout << "Output directory is " << output_dir << endl;
    cout << "Working directory is " << work_dir << endl;
    if ( tile_id > 0 ) {
        cout << "Tile id is " << tile_id << endl;
    } else {
        cout << "Center longitude is " << lon << endl;
        cout << "Center latitude is " << lat << endl;
        cout << "X distance is " << xdist << endl;
        cout << "Y distance is " << ydist << endl;
    }
    cout << "Nudge is " << nudge << endl;
    for (int i = arg_pos; i < argc; i++) {
	load_dirs.push_back(argv[i]);
	cout << "Load directory: " << argv[i] << endl;
    }
    cout << "Priorities file is " << priorities_file << endl;
    if ( ! load_area_types( priorities_file ) ) {
    	    SG_LOG(SG_GENERAL, SG_ALERT, "Failed to load priorities file " << priorities_file);
    	    exit(-1);
    }
    cout << "USGS Map file is " << usgs_map_file << endl;
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
        cout << "Setting RLIMIT_CPU to " << limit.rlim_cur << " seconds" << endl;
    };
    
#endif  // end of stuff that crashes Cygwin

    // main construction data management class
    TGConstruct c;

    c.set_cover( cover );
    c.set_work_base( work_dir );
    c.set_output_base( output_dir );
    c.set_useUKGrid( useUKgrid );
    c.set_write_shared_edges( writeSharedEdges );
    c.set_use_own_shared_edges( useOwnSharedEdges );
    c.set_ignore_landmass( ignoreLandmass );

    c.set_min_nodes( 50 );
    c.set_max_nodes( (int)(TG_MAX_NODES * 0.8) );

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
