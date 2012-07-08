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

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_dir.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/sg_types.hxx>

#include <simgear/math/SGMath.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/misc/sgstream.hxx>
#include <simgear/misc/texcoord.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/structure/exception.hxx>

#include <boost/foreach.hpp>

#include <Geometry/poly_support.hxx>
#include <Geometry/poly_extra.hxx>
#include <Array/array.hxx>
#include <Clipper/clipper.hxx>
#include <GenOutput/genobj.hxx>
#include <Match/match.hxx>
#include <Triangulate/triangle.hxx>
#include <landcover/landcover.hxx>

#include "construct.hxx"
#include "usgs.hxx"

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
static int actual_load_polys( const SGPath& dir, TGConstruct& c, TGClipper& clipper ) {
    int counter = 0;
    string tile_str = c.get_bucket().gen_index_str();

    simgear::Dir d(dir);
    if (!d.exists()) {
        SG_LOG(SG_GENERAL, SG_ALERT, "directory not found:" << dir.str());
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
            SG_LOG(SG_GENERAL, SG_ALERT, "Loading osgb36 poly definition file");
            clipper.load_osgb36_polys( p.str() );
            ++counter;
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "ext = '" << lext << "'");
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

static AreaType get_area_type (const LandCover &cover, double xpos, double ypos, double dx, double dy)
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

    SG_LOG(SG_GENERAL, SG_ALERT, "roughness = " << diff / 50.0 );

    return diff / 50.0;
}


// make the area specified area, look up the land cover type, and add
// it to polys
static void make_area( const LandCover &cover, const TGArray &array, TGPolygon *polys,
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
static int actual_load_landcover ( TGConstruct &c, const TGArray &array, TGClipper &clipper )
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

        SG_LOG(SG_GENERAL, SG_ALERT, "raster land cover: tile at " << base_lon << ',' << base_lat);
    
        double max_lon = c.get_bucket().get_center_lon()
            + 0.5 * c.get_bucket().get_width();
        double max_lat = c.get_bucket().get_center_lat()
            + 0.5 * c.get_bucket().get_height();

        SG_LOG(SG_GENERAL, SG_ALERT, "raster land cover: extends to " << max_lon << ',' << max_lat);

        double x1 = base_lon;
        double y1 = base_lat;
        double x2 = x1 + cover_size;
        double y2 = y1 + cover_size;

        while ( x1 < max_lon ) {
            while ( y1 < max_lat ) {
                make_area( cover, array, polys, x1, y1, x2, y2, half_cover_size, half_cover_size );
                
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
                clipper.add_poly( i, polys[i], get_area_name((AreaType)i ));
                count++;
            }
        }
    } catch ( string e ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Died with exception: " << e);
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
        SG_LOG(SG_GENERAL, SG_ALERT, "poly_path = " << poly_path);
        count += actual_load_polys( poly_path, c, clipper );
        SG_LOG(SG_GENERAL, SG_ALERT, "  loaded " << count << " total polys");
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
    SG_LOG(SG_GENERAL, SG_ALERT, "clipping polygons");
    clipper.clip_all(min, max);

    // update main data repository
    c.set_clipped_polys( clipper.get_polys_clipped() );
    // c.set_fixed_elevations( clipper.get_fixed_elevations_nodes() );

    // really set nodes
    c.set_nodes( clipper.get_nodes() );

    return count;
}


// Load elevation data from an Array file, a regular grid of elevation
// data) and return list of fitted nodes.
static bool load_array( TGConstruct& c, TGArray& array) {
    string base = c.get_bucket().gen_base_path();
    int i;

    for ( i = 0; i < (int)load_dirs.size(); ++i ) {
        string array_path = c.get_work_base() + "/" + load_dirs[i] + "/" + base + "/" + c.get_bucket().gen_index_str();
        SG_LOG(SG_GENERAL, SG_ALERT, "array_path = " << array_path);

        if ( array.open(array_path) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Found Array file " << array_path);
            break;
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "Failed to open Array file " << array_path);
        }
    }

    SGBucket b = c.get_bucket();
    array.parse( b );
    array.remove_voids();

    return true;
}

#if 0
// triangulate the data for each polygon ( first time before splitting )
static void first_triangulate( TGConstruct& c, const TGArray& array, TGTriangle& t ) {
    // first we need to consolidate the points of the Array fit list and
    // all the polygons into a more "Triangle" friendly format

    point_list corner_list = array.get_corner_list();
    point_list fit_list = array.get_fitted_list();
    TGPolyList gpc_polys = c.get_clipped_polys();

    SG_LOG(SG_GENERAL, SG_ALERT, "ready to build node list and polygons");
    t.build( corner_list, fit_list, gpc_polys );
    SG_LOG(SG_GENERAL, SG_ALERT, "done building node list and polygons");

    SG_LOG(SG_GENERAL, SG_ALERT, "ready to do triangulation");
    t.run_triangulate( 0.0, 1 );
    SG_LOG(SG_GENERAL, SG_ALERT, "finished triangulation");
}
#endif

#if 0   // UNUSED
// triangulate the data for each polygon ( second time after splitting
// and reassembling )
static void second_triangulate( TGConstruct& c, TGTriangle& t ) { 
    t.rebuild( c );

    SG_LOG(SG_GENERAL, SG_ALERT, "done re building node list and polygons");
    SG_LOG(SG_GENERAL, SG_ALERT, "ready to do second triangulation");
    SG_LOG(SG_GENERAL, SG_ALERT, "  (pre) nodes = " << c.get_tri_nodes().size());
    SG_LOG(SG_GENERAL, SG_ALERT, "  (pre) normals = " << c.get_point_normals().size());

    t.run_triangulate( 0.0, 2 );

    SG_LOG(SG_GENERAL, SG_ALERT, "  (post) nodes = " << t.get_out_nodes().size());
    SG_LOG(SG_GENERAL, SG_ALERT, "finished second triangulation");
}
#endif

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
// This should be done in the nodes class itself, except for the need for the triangle type
// hopefully, this will get better when we have the area lookup via superpoly...
static void fix_point_heights( TGConstruct& c, const TGArray& array )
{
    int i;
    double z;
    TGPolyList clipped_polys = c.get_clipped_polys(); 
    point_list raw_nodes = c.get_nodes()->get_geod_nodes();

    SG_LOG(SG_GENERAL, SG_ALERT, "fixing node heights");
    
    for ( i = 0; i < (int)raw_nodes.size(); ++i ) {
        // found an elevation we want to preserve
        if ( c.get_nodes()->LookupFixedElevation( raw_nodes[i], &z ) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Found Fixed elevation at " << i << ":" << raw_nodes[i] << " z is " << z );
        } else {
            // interpolate point from DEM data.
            z = array.altitude_from_grid( raw_nodes[i].x() * 3600.0, 
                                          raw_nodes[i].y() * 3600.0 );
            SG_LOG(SG_GENERAL, SG_ALERT, "no    Fixed elevation at " << i << ":" << raw_nodes[i] << " z is " << z );
            c.get_nodes()->SetElevation( i, z );
        }
    }

    // now flatten some stuuf
    for (int i = 0; i < TG_MAX_AREA_TYPES; i++) {
        if ( is_lake_area( (AreaType)i ) ) {
            for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
           	    TGPolygon tri_poly = clipped_polys.superpolys[i][j].get_tris();
                for (int k=0; k< tri_poly.contours(); k++) {
                    if (tri_poly.contour_size(k) != 3) {
                        SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_poly.contour_size(k) );
                        exit(0);                        
                    }

                    double e1, e2, e3, min;
                    int    n1, n2, n3;

                    e1 = tri_poly.get_pt( k, 0 ).z();
                    n1 = c.get_nodes()->find( tri_poly.get_pt( k, 0 ) );

                    e2 = tri_poly.get_pt( k, 1 ).z();
                    n2 = c.get_nodes()->find( tri_poly.get_pt( k, 1 ) );

                    e3 = tri_poly.get_pt( k, 2 ).z();
                    n3 = c.get_nodes()->find( tri_poly.get_pt( k, 2 ) );

                    min = e1;
                    if ( e2 < min ) { min = e2; }                  
                    if ( e3 < min ) { min = e3; }                  

                    c.get_nodes()->SetElevation( n1, min );
                    c.get_nodes()->SetElevation( n2, min );
                    c.get_nodes()->SetElevation( n3, min );
                }
            }
        }

        if ( is_stream_area( (AreaType)i ) ) {
            for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
           	    TGPolygon tri_poly = clipped_polys.superpolys[i][j].get_tris();
                for (int k=0; k< tri_poly.contours(); k++) {
                    if (tri_poly.contour_size(k) != 3) {
                        SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_poly.contour_size(k) );
                        exit(0);                        
                    }

                    double e1, e2, e3, min;
                    int    n1, n2, n3;
                    Point3D p;

                    e1 = tri_poly.get_pt( k, 0 ).z();
                    n1 = c.get_nodes()->find( tri_poly.get_pt( k, 0 ) );

                    e2 = tri_poly.get_pt( k, 1 ).z();
                    n2 = c.get_nodes()->find( tri_poly.get_pt( k, 1 ) );

                    e3 = tri_poly.get_pt( k, 2 ).z();
                    n3 = c.get_nodes()->find( tri_poly.get_pt( k, 2 ) );

                    min = e1;
                    p   = raw_nodes[n1];

                    if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }                  
                    if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }                  

                    double d1 = distanceSphere( p, raw_nodes[n1] ); 
                    double d2 = distanceSphere( p, raw_nodes[n2] ); 
                    double d3 = distanceSphere( p, raw_nodes[n3] ); 

                    double max1 = d1 * 0.20 + min;
                    double max2 = d2 * 0.20 + min;
                    double max3 = d3 * 0.20 + min;

                    if ( max1 < e1 ) { c.get_nodes()->SetElevation( n1, max1 ); }
                    if ( max2 < e2 ) { c.get_nodes()->SetElevation( n2, max2 ); }
                    if ( max3 < e3 ) { c.get_nodes()->SetElevation( n3, max3 ); }
                }
            }
        }

        if ( is_road_area( (AreaType)i ) ) {
            for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
           	    TGPolygon tri_poly = clipped_polys.superpolys[i][j].get_tris();
                for (int k=0; k< tri_poly.contours(); k++) {
                    if (tri_poly.contour_size(k) != 3) {
                        SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_poly.contour_size(k) );
                        exit(0);                        
                    }

                    double e1, e2, e3, min;
                    int    n1, n2, n3;
                    Point3D p;

                    e1 = tri_poly.get_pt( k, 0 ).z();
                    n1 = c.get_nodes()->find( tri_poly.get_pt( k, 0 ) );

                    e2 = tri_poly.get_pt( k, 1 ).z();
                    n2 = c.get_nodes()->find( tri_poly.get_pt( k, 1 ) );

                    e3 = tri_poly.get_pt( k, 2 ).z();
                    n3 = c.get_nodes()->find( tri_poly.get_pt( k, 2 ) );

                    min = e1;
                    p   = raw_nodes[n1];

                    if ( e2 < min ) { min = e2; p = raw_nodes[n2]; }                  
                    if ( e3 < min ) { min = e3; p = raw_nodes[n3]; }                  

                    double d1 = distanceSphere( p, raw_nodes[n1] ); 
                    double d2 = distanceSphere( p, raw_nodes[n2] ); 
                    double d3 = distanceSphere( p, raw_nodes[n3] ); 

                    double max1 = d1 * 0.30 + min;
                    double max2 = d2 * 0.30 + min;
                    double max3 = d3 * 0.30 + min;

                    if ( max1 < e1 ) { c.get_nodes()->SetElevation( n1, max1 ); }
                    if ( max2 < e2 ) { c.get_nodes()->SetElevation( n2, max2 ); }
                    if ( max3 < e3 ) { c.get_nodes()->SetElevation( n3, max3 ); }
                }
            }
        }

        if ( is_ocean_area( (AreaType)i ) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, " Flatten the ocean - has " << clipped_polys.superpolys[i].size() << " polys" );

            for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
           	    TGPolygon tri_poly = clipped_polys.superpolys[i][j].get_tris();

                SG_LOG(SG_GENERAL, SG_ALERT, " Flatten the ocean - poly " << j << " has " << tri_poly.contours() << " contours" );

                for (int k=0; k< tri_poly.contours(); k++) {
                    if (tri_poly.contour_size(k) != 3) {
                        SG_LOG(SG_GENERAL, SG_ALERT, "triangle doesnt have 3 nodes" << tri_poly.contour_size(k) );
                        exit(0);                        
                    }

                    int    n1, n2, n3;

                    n1 = c.get_nodes()->find( tri_poly.get_pt( k, 0 ) );
                    n2 = c.get_nodes()->find( tri_poly.get_pt( k, 1 ) );
                    n3 = c.get_nodes()->find( tri_poly.get_pt( k, 2 ) );

                    SG_LOG(SG_GENERAL, SG_ALERT, "Set Ocean Triangle " << n1 << "," << n2 << "," << n3 << " to 0.0" );
                    c.get_nodes()->SetElevation( n1, 0.0 );
                    c.get_nodes()->SetElevation( n2, 0.0 );
                    c.get_nodes()->SetElevation( n3, 0.0 );
                }
            }
        }
    }
}


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


#if 0   // Handled by TGNodes
// build the wgs-84 point list
static void build_wgs_84_point_list( TGConstruct& c, const TGArray& array ) {
    point_list geod_nodes;
    point_list wgs84_nodes;
    int i;

    SG_LOG(SG_GENERAL, SG_ALERT, "generating wgs84 list");
    Point3D geod, radians, cart;

    point_list raw_nodes = c.get_tri_nodes().get_node_list();

    for ( i = 0; i < (int)raw_nodes.size(); ++i ) {
        geod = raw_nodes[i];

        // convert to radians
        radians = Point3D( geod.x() * SGD_DEGREES_TO_RADIANS,
                           geod.y() * SGD_DEGREES_TO_RADIANS,
                           geod.z() );

        cart = sgGeodToCart(radians);

        geod_nodes.push_back(geod);
        wgs84_nodes.push_back(cart);
    }

    c.set_geod_nodes( geod_nodes );
    c.set_wgs84_nodes( wgs84_nodes );
}
#endif

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

// generate the flight gear scenery file
static void do_output( TGConstruct& c, TGGenOutput& output ) {
    output.build_tris( c );
    output.write_tris( c );
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
        SG_LOG(SG_GENERAL, SG_ALERT, "ERROR: opening " << dest_ind << " for writing!");
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
        //SG_LOG(SG_GENERAL, SG_ALERT, "collecting custom objects from " << index_file);

        sg_gzifstream in( index_file );

	if ( ! in.is_open() ) {
	    //SG_LOG(SG_GENERAL, SG_ALERT, "No custom objects");
	} else {
	    while ( ! in.eof() ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "collecting custom objects from " << index_file);
                in.getline(line, 2048);
                SG_LOG(SG_GENERAL, SG_ALERT, "line = " << line);

                int result = sscanf( line, "%s %s", token, name );
                SG_LOG(SG_GENERAL, SG_ALERT, "scanf scanned " << result << " tokens" );

                if ( result > 0 ) {
                    SG_LOG(SG_GENERAL, SG_ALERT, "token = " << token << " name = " << name );

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

static void add_intermediate_nodes( TGConstruct& c ) {
    TGPolyList polys = c.get_clipped_polys();
    TGNodes*   nodes = c.get_nodes();

    int before, after;

    // traverse each poly, and add intermediate nodes
    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        SG_LOG( SG_CLIPPER, SG_INFO, "num polys of type (" << i << ") = " << polys.superpolys[i].size() );
        for( unsigned int j = 0; j < polys.superpolys[i].size(); ++j ) {
            TGPolygon current = polys.superpolys[i][j].get_poly();
            SG_LOG( SG_CLIPPER, SG_INFO, get_area_name( (AreaType)i ) << " = " << current.contours() );

            before = current.total_size();
            current = add_tgnodes_to_poly( current, nodes );
            after = current.total_size();

            if (before != after) {
                SG_LOG( SG_CLIPPER, SG_INFO, "add_tgnodes_to_poly modified poly " << j << " from " << before << " points to " << after << " points" );
            }

            /* Save it back */
            polys.superpolys[i][j].set_poly( current );

            if (before != after) {
                SG_LOG( SG_CLIPPER, SG_INFO, " VERIFY add_tgnodes_to_poly modified poly nodes is " << polys.superpolys[i][j].get_poly().total_size() );
            }
        }
    }

    c.set_clipped_polys(polys);

    SG_LOG( SG_CLIPPER, SG_INFO, "  add intermediate nodes finished." );
}

static TGPolygon area_tex_coords( const TGPolygon& tri, const SGBucket &b, const TGArray &array )
{
    TGPolygon result;
    result.erase();

    // lots of conversion needed to use simgear API - perhaps we need a new simgear API?        
    for (int c=0; c<tri.contours(); c++)
    {
        // get the points, and calculate the elevations
        point_list nodes = tri.get_contour(c);
        std::vector< SGGeod > conv_geods;
        point_list tex_coords;

        for (int i = 0; i < (int)nodes.size(); ++i ) 
        {
            SGGeod conv_geod = SGGeod::fromDegM( nodes[i].x(), nodes[i].y(), nodes[i].z() );
        	SG_LOG(SG_GENERAL, SG_DEBUG, "geod pt = " << nodes[i] );
           	conv_geods.push_back( conv_geod );
        }
        
        // now calculate texture coordinates
        // generate identity interger list...  
        std::vector< int > node_idx;
        for (int i = 0; i < (int)conv_geods.size(); i++) {
            node_idx.push_back(i);
        }

        std::vector< SGVec2f > tp_list = sgCalcTexCoords( b, conv_geods, node_idx );
        // generate a contour of texture coordinates from the tp list
        for (int i = 0; i < (int)tp_list.size(); i++)
        {
            tex_coords.push_back( Point3D::fromSGVec2( tp_list[i] ) );
        }
        result.add_contour( tex_coords, 0 );
    }

    return result;
}

static TGPolygon linear_tex_coords( const TGPolygon& tri, const TGTexParams& tp )
{
    TGPolygon result;
    int i, j;

    result.erase();

    Point3D ref = tp.get_ref();
    double width = tp.get_width();
    double length = tp.get_length();
    double heading = tp.get_heading();
    double minu = tp.get_minu();
    double maxu = tp.get_maxu();
    double minv = tp.get_minv();
    double maxv = tp.get_maxv();
    SG_LOG( SG_GENERAL, SG_DEBUG, "section ref = " << ref );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  width   = " << width );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  length  = " << length );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  heading = " << heading );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  minv    = " << minv );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  maxv    = " << maxv );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  heading = " << heading );

    Point3D p, t;
    double x, y, tx, ty;

    for ( i = 0; i < tri.contours(); ++i ) 
    {
    	for ( j = 0; j < tri.contour_size( i ); ++j ) 
        {
    	    p = tri.get_pt( i, j );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "tex coords for contour " << i << "point " << j << ": " << p );

    	    //
    	    // 1. Calculate distance and bearing from the center of
    	    // the feature
    	    //

    	    // given alt, lat1, lon1, lat2, lon2, calculate starting
    	    // and ending az1, az2 and distance (s).  Lat, lon, and
    	    // azimuth are in degrees.  distance in meters
    	    double az1, az2, dist;
    	    geo_inverse_wgs_84( 0, ref.y(), ref.x(), p.y(), p.x(),
    				&az1, &az2, &dist );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "basic course from ref = " << az2);

    	    //
    	    // 2. Rotate this back into a coordinate system where Y
    	    // runs the length of the runway and X runs crossways.
    	    //

    	    double course = az2 - heading;
    	    while ( course < -360 ) { course += 360; }
    	    while ( course > 360 ) { course -= 360; }
    	    SG_LOG( SG_GENERAL, SG_DEBUG,
                        "  course = " << course << "  dist = " << dist );

    	    //
    	    // 3. Convert from polar to cartesian coordinates
    	    //

    	    x = sin( course * SGD_DEGREES_TO_RADIANS ) * dist;
    	    y = cos( course * SGD_DEGREES_TO_RADIANS ) * dist;
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  x = " << x << " y = " << y);

    	    //
    	    // 4. Map x, y point into texture coordinates
    	    //
    	    double tmp;

            tmp = x / width;
            tx = tmp * (maxu - minu) + minu;

            if ( tx < -1.0 )  { tx = -1.0; }
            if ( tx > 1.0 ) { tx = 1.0; }

    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

            ty = (y/length) + minv;
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << ty << ")");

    	    t = Point3D( tx, ty, 0 );
    	    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ", " << ty << ")");

    	    result.add_node( i, t );
    	}
    }

    return result;
}

// fix the elevations of the geodetic nodes
// TODO : Get rid of this
#if 0
static point_list calc_elevations( TGConstruct& c, const TGArray& array, point_list &pl )
{
    int    i;
    double z;

    point_list raw_nodes1 = pl;
    point_list raw_nodes2 = c.get_nodes().get_geod_nodes();

    SG_LOG(SG_GENERAL, SG_ALERT, "rn1 " << raw_nodes1.size() << " rn2 " << raw_nodes2.size() );

    for ( i = 0; i < (int)raw_nodes1.size(); ++i ) {
        // found an elevation we want to preserve
        if ( c.get_nodes().LookupFixedElevation( raw_nodes1[i], &z ) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Found Fixed elevation at " << raw_nodes1[i] << " z is " << z );
        } else {
            // interpolate point from DEM data.
            z = array.altitude_from_grid( raw_nodes1[i].x() * 3600.0, 
                                          raw_nodes1[i].y() * 3600.0 );
            SG_LOG(SG_GENERAL, SG_ALERT, "no    Fixed elevation at " << raw_nodes1[i] << " z is " << z );
        }

        raw_nodes1[i].setz( z );
    }

    return raw_nodes1;
}
#endif

#if 0
static void calc_elevations( TGConstruct& c, const TGArray& array )
{
    int    i;
    double z;

    point_list raw_nodes = c.get_nodes()->get_geod_nodes();

    for ( i = 0; i < (int)raw_nodes.size(); ++i ) {
        // found an elevation we want to preserve
        if ( c.get_nodes()->LookupFixedElevation( raw_nodes[i], &z ) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Found Fixed elevation at " << raw_nodes[i] << " z is " << z );
        } else {
            // interpolate point from DEM data.
            z = array.altitude_from_grid( raw_nodes[i].x() * 3600.0, 
                                          raw_nodes[i].y() * 3600.0 );
            SG_LOG(SG_GENERAL, SG_ALERT, "no    Fixed elevation at " << raw_nodes[i] << " z is " << z );
        }

        c.get_nodes()->SetElevation( i, z );
    }

    return ;
}
#endif

// First up:
// Create new node class for the node list

// Second up:
//
// triangulate each poly sperately, then generate a TGTriangle with the same output as global...
// (add set_out_nodes, ele_list, and segs )
//

#if 0
// master construction routine
static void construct_tile( TGConstruct& c ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Construct tile, bucket = " << c.get_bucket() );

    // STEP 1) Load elevation data

    // load grid of elevation data (Array)
    TGArray array;
    load_array( c, array );

    // STEP 2) Clip 2D polygons against one another

    // load and clip 2d polygon data
    if ( load_polys( c, array ) == 0 ) {
        // don't build the tile if there is no 2d data ... it *must*
        // be ocean and the sim can build the tile on the fly.
        return;
    }

    // Step 3) Merge in Shared data (just add the nodes to the nodelist)
    // When this step is complete, some nodes will have normals (from shared tiles)
    // and some will not - need to indicate this in the new node class

    // Step 4) Add intermediate nodes
    // need to add another add intermediate nodes function that can handle the new node class

    // Step 5) Triangulate (not global)
    TGTriangle t;
    // triangulate the data for each polygon
    first_triangulate( c, array, t );
    SG_LOG(SG_GENERAL, SG_ALERT, "number of fitted nodes = " << t.get_out_nodes_size());

    // Step 6) Generate global triangle list
    // save the results of the triangulation
    c.set_tri_nodes( t.get_out_nodes() );
    c.set_tri_elements( t.get_elelist() );
    c.set_tri_segs( t.get_out_segs() );

    // Step 7) Flatten
    // calculate wgs84 (cartesian) form of node list
    fix_point_heights( c, array );

    // Step 8) Convert nodes to wgs_84
    build_wgs_84_point_list( c, array );

    // Step 9) Generate face_connected list
    // build the node -> element (triangle) reverse lookup table
    c.set_reverse_ele_lookup( gen_node_ele_lookup_table( c ) );

    // Step 10) Generate Face normals
    // build the face normal list
    c.set_face_normals( gen_face_normals( c ) );

    // Step 11) Generate node normals
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
        SG_LOG(SG_GENERAL, SG_ALERT, "oops, need to add normals :-(");
        point_list normals = c.get_point_normals();
        int start = normals.size();
        int end = c.get_tri_nodes().size();
        for ( int i = start; i < end; ++i ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "adding a normal for " << c.get_tri_nodes().get_node(i));
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

    // Step 12) calculate texture coordinates for each triangle

    // Step 13) Sort the triangle list by material (optional)

    // Step 14) Generate the output    
    // generate the output
    TGGenOutput output;
    do_output( c, output );

    array.close();

    // Step 15) Adding custome objects to the .stg file
    // collect custom objects and move to scenery area
    do_custom_objects( c );
}

#endif

// master construction routine
static void construct_tile( TGConstruct& c ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Construct tile, bucket = " << c.get_bucket() );

    // STEP 1) Load elevation data

    // load grid of elevation data (Array)
    TGArray array;
    load_array( c, array );
    
    // STEP 2) Clip 2D polygons against one another

    // load and clip 2d polygon data
    if ( load_polys( c, array ) == 0 ) {
        // don't build the tile if there is no 2d data ... it *must*
        // be ocean and the sim can build the tile on the fly.
        return;
    }    

    // Make sure we have the elavation nodes in the main node database
    // I'd like to do this first, but we get initial tgnodes from clipper
    point_list corner_list = array.get_corner_list();
    if ( corner_list.size() == 0 ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "corner list is 0 " );
        // exit(0);    
    }

    for (unsigned int i=0; i<corner_list.size(); i++) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Add corner node " << corner_list[i]  );
        //c.get_nodes()->unique_add_fixed_elevation(corner_list[i]);
        c.get_nodes()->unique_add(corner_list[i]);
    }

    point_list fit_list = array.get_fitted_list();
    for (unsigned int i=0; i<fit_list.size(); i++) {
        //c.get_nodes()->unique_add_fixed_elevation(fit_list[i]);
        c.get_nodes()->unique_add(fit_list[i]);
    }

    // Step 3) Merge in Shared data (just add the nodes to the nodelist)
    // When this step is complete, some nodes will have normals (from shared tiles)
    // and some will not - need to indicate this in the new node class
    
    SG_LOG(SG_GENERAL, SG_ALERT, "number of geod nodes = before adding adding shared edges" << c.get_nodes()->size() );

    TGMatch m;
    m.load_neighbor_shared( c );
    if ( c.get_use_own_shared_edges() ) {
        m.load_missing_shared( c );
    }
    m.add_shared_nodes( c );

    SG_LOG(SG_GENERAL, SG_ALERT, "number of geod nodes = after adding adding shared edges" << c.get_nodes()->size() );

    // Step 4) Add intermediate nodes
    // need to add another add intermediate nodes function that can handle the new node class
    add_intermediate_nodes( c );

    TGPolyList clipped_polys = c.get_clipped_polys();

    SG_LOG(SG_GENERAL, SG_ALERT, "number of geod nodes = before adding clipping intersections" << c.get_nodes()->size() );
    
    for (int i = 0; i < TG_MAX_AREA_TYPES; i++) {
        for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
            TGPolygon poly = clipped_polys.superpolys[i][j].get_poly();
            for (int k=0; k< poly.contours(); k++) {
                for (int l = 0; l < poly.contour_size(k); l++) {
                    // ensure we have all nodes...
                    c.get_nodes()->unique_add( poly.get_pt( k, l ) );
                }
            } 
        }
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "number of geod nodes = after adding clipping intersections" << c.get_nodes()->size() );

    // tesselate the polygons and prepair them for final output
    point_list extra = c.get_nodes()->get_geod_nodes();
    for (int i = 0; i < TG_MAX_AREA_TYPES; i++) {
        for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
            TGPolygon poly = clipped_polys.superpolys[i][j].get_poly();
            SG_LOG(SG_GENERAL, SG_INFO, "Tesselating poly " << i << ", " << j << " : material = " << clipped_polys.superpolys[i][j].get_material() << " : flag = " << clipped_polys.superpolys[i][j].get_flag());                       

            TGPolygon tri = polygon_tesselate_alt_with_extra( poly, extra, false );
            TGPolygon tc;

            if ( clipped_polys.superpolys[i][j].get_flag() == "textured" ) {
                // SG_LOG(SG_GENERAL, SG_DEBUG, "USE TEXTURE PARAMS for tex coord calculations" );
                // tc = linear_tex_coords( tri, clipped_polys.texparams[i][j] );
                tc = area_tex_coords( tri, c.get_bucket(), array );
            } else {
                // SG_LOG(SG_GENERAL, SG_DEBUG, "USE SIMGEAR for tex coord calculations" );
                tc = area_tex_coords( tri, c.get_bucket(), array );
            }

            clipped_polys.superpolys[i][j].set_tris( tri );
      	    clipped_polys.superpolys[i][j].set_texcoords( tc );
        }
    }

    // Add triangulation points
    for (int i = 0; i < TG_MAX_AREA_TYPES; i++) {
        for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
            TGPolygon tri_poly = clipped_polys.superpolys[i][j].get_tris();
            for (int k=0; k< tri_poly.contours(); k++) {
                for (int l = 0; l < tri_poly.contour_size(k); l++) {
                    // ensure we have all nodes...
                    c.get_nodes()->unique_add( tri_poly.get_pt( k, l ) );
                }
            } 
        }
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "number of geod nodes = after adding triangulation nodes" << c.get_nodes()->size() );

    c.set_clipped_polys( clipped_polys );

#if 0

//NEXT BIG STEP : TRIANGULATE EACH POLY INDEPENDENTLY

    // Step 5) Triangulate (not global)
    TGTriangle t;
    // triangulate the data for each polygon
    first_triangulate( c, array, t );
    SG_LOG(SG_GENERAL, SG_ALERT, "number of fitted nodes = " << t.get_out_nodes_size());

    // Step 6) Generate global triangle list
    // save the results of the triangulation
    // c.set_tri_nodes( t.get_out_nodes() );
    c.set_tri_elements( t.get_elelist() );
    c.set_tri_segs( t.get_out_segs() );

// SO WE WONT NEED TRI ELEMENTS AND TRI SEGS
// BUT: TBD : NEED FACE NORMALS - SO WE NEED NODE TO TRIANGLE LOOKUP TABLE.
// STORE TRIANGLES IN SUPERPOLY....

#endif

    // Step 7) Flatten
    fix_point_heights( c, array );

    for (unsigned int i=0; i<c.get_nodes()->size(); i++) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Point[" << i << "] is " << c.get_nodes()->get_geod_nodes()[i] );
    }

#if 0

    SG_LOG(SG_GENERAL, SG_ALERT, "REVERSE ELE LOOKUP ");

    // Step 9) Generate face_connected list
    // build the node -> element (triangle) reverse lookup table
    c.set_reverse_ele_lookup( gen_node_ele_lookup_table( c ) );

    SG_LOG(SG_GENERAL, SG_ALERT, "FACE NORMALS ");

    // Step 10) Generate Face normals
    // build the face normal list
    c.set_face_normals( gen_face_normals( c ) );

    // Step 11) Generate node normals
    // calculate the normals for each point in wgs84_nodes
    c.set_point_normals( gen_point_normals( c ) );

    // now we must retriangulate the pasted together tile points
    // second_triangulate( c, t );

    // save the results of the triangulation
    // c.set_tri_nodes( t.get_out_nodes() );
    // c.set_tri_elements( t.get_elelist() );
    // c.set_tri_segs( t.get_out_segs() );

    // double check on the off chance that the triangulator was forced
    // to introduce additional points
#if 0
    if ( c.get_tri_nodes().size() > c.get_point_normals().size() ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "oops, need to add normals :-(");
        point_list normals = c.get_point_normals();
        int start = normals.size();
        int end = c.get_tri_nodes().size();
        for ( int i = start; i < end; ++i ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "adding a normal for " << c.get_tri_nodes().get_node(i));
            Point3D p = tgFakeNormal( c.get_tri_nodes().get_node(i) );
            normals.push_back( p );
        }
        c.set_point_normals( normals );
    }
#endif

    if ( c.get_cover().size() > 0 ) {
        // Now for all the remaining "default" land cover polygons, assign
        // each one it's proper type from the land use/land cover
        // database.
        fix_land_cover_assignments( c );
    }

    // Step 12) calculate texture coordinates for each triangle

    // Step 13) Sort the triangle list by material (optional)
#endif

#if 1
    // write shared data
    m.split_tile( c );
    SG_LOG(SG_GENERAL, SG_ALERT, "Tile Split");

    if ( c.get_write_shared_edges() ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "write shared edges");

        m.write_shared( c );
    }
#endif

// TEMP TEMP TEMP TEMP

    TGTriNodes normals, texcoords;
    normals.clear();
    texcoords.clear();

    group_list pts_v; pts_v.clear();
    group_list pts_n; pts_n.clear();
    string_list pt_materials; pt_materials.clear();

    group_list tris_v; tris_v.clear();
    group_list tris_n; tris_n.clear();
    group_list tris_tc; tris_tc.clear();
    string_list tri_materials; tri_materials.clear();

    group_list strips_v; strips_v.clear();
    group_list strips_n; strips_n.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();

    int index;
    int_list pt_v, tri_v, strip_v;
    int_list pt_n, tri_n, strip_n;
    int_list tri_tc, strip_tc;

    // calculate "the" normal for this tile
    // temporary - generate a single normal
    Point3D p;
    p.setx( c.get_bucket().get_center_lon() * SGD_DEGREES_TO_RADIANS );
    p.sety( c.get_bucket().get_center_lat() * SGD_DEGREES_TO_RADIANS );
    p.setz( 0 );
    Point3D vnt = sgGeodToCart( p );
    
    SGVec3d tmp( vnt.x(), vnt.y(), vnt.z() );
    tmp = normalize(tmp);
    Point3D vn( tmp.x(), tmp.y(), tmp.z() );

    SG_LOG(SG_GENERAL, SG_DEBUG, "found normal for this airport = " << tmp);

    for (int i = 0; i < TG_MAX_AREA_TYPES; i++) {
        // only tesselate non holes
        if ( !is_hole_area( i ) ) {
            for (int j = 0; j < (int)clipped_polys.superpolys[i].size(); ++j ) {
            	SG_LOG(SG_GENERAL, SG_INFO, "tri " << i << ", " << j);
            	TGPolygon tri_poly = clipped_polys.superpolys[i][j].get_tris();
            	TGPolygon tri_txs = clipped_polys.superpolys[i][j].get_texcoords();
            	string material = clipped_polys.superpolys[i][j].get_material();
            	SG_LOG(SG_GENERAL, SG_INFO, "material = " << material);
            	SG_LOG(SG_GENERAL, SG_INFO, "poly size = " << tri_poly.contours());
            	SG_LOG(SG_GENERAL, SG_INFO, "texs size = " << tri_txs.contours());
            	for (int k = 0; k < tri_poly.contours(); ++k) 
                {
        	        tri_v.clear();
        	        tri_n.clear();
        	        tri_tc.clear();
        	        for (int l = 0; l < tri_poly.contour_size(k); ++l) 
                    {
            		    p = tri_poly.get_pt( k, l );
            		    index = c.get_nodes()->find( p );
                        if (index < 0) { 
                            SG_LOG(SG_GENERAL, SG_ALERT, "NODE NOT FOUND " << p);
                            exit(0);
                        }

            		    tri_v.push_back( index );
    
                		// use 'the' normal
                		index = normals.unique_add( vn );
                		tri_n.push_back( index );
    
                		Point3D tc = tri_txs.get_pt( k, l );
                		index = texcoords.unique_add( tc );
                		tri_tc.push_back( index );
            	    }
            	    tris_v.push_back( tri_v );
        	        tris_n.push_back( tri_n );
        	        tris_tc.push_back( tri_tc );
        	        tri_materials.push_back( material );
        	    }
            }
        }
    }
    
    std::vector< SGVec3d > wgs84_nodes = c.get_nodes()->get_wgs84_nodes_as_SGVec3d();
    SGSphered d;

    for (int i = 0; i < (int)wgs84_nodes.size(); ++i) 
    {
        d.expandBy(wgs84_nodes[ i ]);
    }
    
    SGVec3d gbs_center = d.getCenter();
    double gbs_radius = d.getRadius();
    SG_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
    SG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_n; fans_n.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    string base = c.get_output_base();
    SGBucket b = c.get_bucket();
    string binname = b.gen_index_str();
    binname += ".btg";
    string txtname = b.gen_index_str();
    txtname += ".txt";
    
    std::vector< SGVec3f > normals_3f;
    for (int i=0; i < (int)normals.get_node_list().size(); i++ ) 
    {
        Point3D node = normals.get_node_list()[i];
        normals_3f.push_back( node.toSGVec3f() );
    }

    std::vector< SGVec2f > texcoords_2f;
    for (int i=0; i < (int)texcoords.get_node_list().size(); i++ ) 
    {
        Point3D node = texcoords.get_node_list()[i];
        texcoords_2f.push_back( node.toSGVec2f() );
    }

    SGBinObject obj;
    
    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals_3f );
    obj.set_texcoords( texcoords_2f );
    obj.set_pts_v( pts_v );
    obj.set_pts_n( pts_n );
    obj.set_pt_materials( pt_materials );
    obj.set_tris_v( tris_v );
    obj.set_tris_n( tris_n );
    obj.set_tris_tc( tris_tc ); 
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_n( strips_n );
    obj.set_strips_tc( strips_tc ); 
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_n( fans_n );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );
    
    bool result;
    result = obj.write_bin( base, binname, b );
    if ( !result ) 
    {
        throw sg_exception("error writing file. :-(");
    }
    result = obj.write_ascii( base, txtname, b );
    if ( !result ) 
    {
        throw sg_exception("error writing file. :-(");
    }

// TEMP TEMP TEMP TEMP

#if 0
    // Step 14) Generate the output    
    // generate the output
    TGGenOutput output;
    do_output( c, output );
#endif

    array.close();

    // Step 15) Adding custome objects to the .stg file
    // collect custom objects and move to scenery area
    do_custom_objects( c );
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
    c.set_useUKGrid( useUKgrid );
    c.set_write_shared_edges( writeSharedEdges );
    c.set_use_own_shared_edges( useOwnSharedEdges );
    c.set_ignore_landmass( ignoreLandmass );

    c.set_min_nodes( 50 );
    c.set_max_nodes( (int)(TG_MAX_NODES * 0.8) );

    if (tile_id == -1) {
        if (xdist == -1 || ydist == -1) {
            // construct the tile around the specified location
            SG_LOG(SG_GENERAL, SG_ALERT, "Building single tile at " << lat << ',' << lon);
            SGBucket b( lon, lat );
            c.set_bucket( b );
            construct_tile( c );
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
            	c.set_bucket( b_min );
            	construct_tile( c );
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
                            c.set_bucket( b_cur );
                            construct_tile( c );
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
        c.set_bucket( b );
        construct_tile( c );
    }

    SG_LOG(SG_GENERAL, SG_ALERT, "[Finished successfully]");
    return 0;
}
