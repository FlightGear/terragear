// build.cxx -- routines to build polygon model of an airport from the runway
//              definition
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
//


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <simgear/compiler.h>

#include <stdio.h>
#include <stdlib.h>		// for atoi() atof()
#include <time.h>

#include <list>
#include STL_STRING

#include <plib/sg.h>			// plib include

#include <simgear/constants.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/fg_geodesy.hxx>
#include <simgear/misc/texcoord.hxx>

#include <Array/array.hxx>
#include <Build/poly_support.hxx>
#include <Build/trinodes.hxx>
#include <Polygon/index.hxx>
#include <Polygon/polygon.hxx>
#include <Triangulate/trieles.hxx>

#include "convex_hull.hxx"
#include "point2d.hxx"
#include "runway.hxx"
#include "scenery_version.hxx"


typedef vector < int_list > group_list;
typedef group_list::iterator group_list_iterator;
typedef group_list::const_iterator const_group_list_iterator;


void write_polygon( const FGPolygon& poly, const string& base ) {
    for ( int i = 0; i < poly.contours(); ++i ) {
	char name[256];
	sprintf(name, "%s%d", base.c_str(), i );
	FILE *fp = fopen( name, "w" );

	for ( int j = 0; j < poly.contour_size( i ); ++j ) {
	    Point3D p0 = poly.get_pt(i, j);
	    fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	}
	Point3D p0 = poly.get_pt(i, 0);
	fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	fclose(fp);
    }
}


#if 0
// calculate distance in meters between two lat/lon points
static double gc_dist( Point3D p1, Point3D p2 ) {
    Point3D r1( p1.x() * DEG_TO_RAD, p1.y() * DEG_TO_RAD, 0 );
    Point3D r2( p2.x() * DEG_TO_RAD, p2.y() * DEG_TO_RAD, 0 );

    // d=2*asin(sqrt((sin((lat1-lat2)/2))^2 + 
    //          cos(lat1)*cos(lat2)*(sin((lon1-lon2)/2))^2))
    double tmp1 = sin( (r1.y() - r2.y()) / 2.0 );
    double tmp2 = sin( (r1.x() - r2.x()) / 2.0 );

    // d=2*asin(sqrt((tmp1)^2 + cos(lat1)*cos(lat2)*(tmp2)^2))
    double clat1 = cos( r1.y() );
    double clat2 = cos( r1.y() );

    // d=2*asin(sqrt(tmp1*tmp1 + clat1*clat2*tmp2*tmp2))
    double tmp3 = sqrt(tmp1*tmp1 + clat1*clat2*tmp2*tmp2);

    // d=2*asin(tmp3)
    double d_rad = 2 * asin( tmp3 );
    // cout << "  dist (rad) = " << d_rad << endl;

    double d_nm = d_rad * RAD_TO_NM;
    // cout << "  dist (nm) = " << d_nm << endl;

    double d_m = d_nm * NM_TO_METER;
    // cout << "  dist (m) = " << d_m << endl;

    return d_m;
}


// calculate true course between two points given the distance in meters
static double gc_course( Point3D p1, Point3D p2, double d_m ) {
    double lon1 = p1.x() * DEG_TO_RAD;
    double lon2 = p2.x() * DEG_TO_RAD;
    double lat1 = p1.y() * DEG_TO_RAD;
    double lat2 = p2.y() * DEG_TO_RAD;

    double d_rad = d_m * METER_TO_NM * NM_TO_RAD;

    double tc1;

    if ( cos(lat1) < FG_EPSILON) {
	if ( lat1 > 0.0 ) {
	    tc1 = FG_PI;        //  starting from N pole
	} else {
	    tc1 = 0.0;		//  starting from S pole
	}
    }
	
    // For starting points other than the poles: 
	
    if ( sin(lon2 - lon1) < 0.0 ) {
	tc1 = acos( (sin(lat2)-sin(lat1)*cos(d_rad))/(sin(d_rad)*cos(lat1)));
    } else {
	tc1 = FG_2PI - 
	    acos((sin(lat2)-sin(lat1)*cos(d_rad))/(sin(d_rad)*cos(lat1)));
    }

    return tc1;
}
#endif


// calculate texture coordinates for a 1/2 runway.  Returns a mirror
// polygon to the runway, except each point is the texture coordinate
// of the corresponding point in the original polygon.
static FGPolygon rwy_calc_tex_coords( const FGRunway& rwy,
				      double hdg_offset,
				      const FGPolygon& in_poly )
{
    FGPolygon result;
    result.erase();
    double length = rwy.length * FEET_TO_METER;
    double width = rwy.width * FEET_TO_METER;

    Point3D center( rwy.lon, rwy.lat, 0 );
    cout << "runway heading = " << rwy.heading << endl;
    double angle = rwy.heading + hdg_offset;
    Point3D p, tp;
    double x, y, tx, ty;

    for ( int i = 0; i < in_poly.contours(); ++i ) {
	for ( int j = 0; j < in_poly.contour_size( i ); ++j ) {
	    p = in_poly.get_pt( i, j );

	    // dist = gc_dist( center, p );
	    // course = gc_course( center, p, dist ) + angle;

	    // given alt, lat1, lon1, lat2, lon2, calculate starting
	    // and ending az1, az2 and distance (s).  Lat, lon, and
	    // azimuth are in degrees.  distance in meters
	    double az1, az2, dist;
	    geo_inverse_wgs_84( 0, center.y(), center.x(), p.y(), p.x(),
				&az1, &az2, &dist );

	    cout << "basic course = " << az1 << endl;
	    double course = az1 + angle;
	    cout << "course = " << course << endl;
	     while ( course < -360 ) { course += 360; }
	     while ( course > 360 ) { course -= 360; }
	    // cout << "Dist = " << dist << endl;
	    // cout << "  Course = " << course * 180.0 / FG_PI << endl;

	    x = cos( course * DEG_TO_RAD ) * dist;
	    y = sin( course * DEG_TO_RAD ) * dist;
	    cout << "  x = " << x << " y = " << y << endl;

	    tx = x / (length / 2.0);
	    tx = ((int)(tx * 100)) / 100.0;
	    if ( tx < -1.0 ) { tx = -1.0; }
	    if ( tx > 1.0 ) { tx = 1.0; }

	    ty = (y + (width / 2.0)) / width;
	    ty = ((int)(ty * 100)) / 100.0;
	    if ( ty < -1.0 ) { ty = -1.0; }
	    if ( ty > 1.0 ) { ty = 1.0; }

	    tp = Point3D( tx, ty, 0 );
	    cout << "  (" << tx << ", " << ty << ")" << endl;

	    result.add_node( i, tp );
	}
    }

    return result;
}


#if 0
// Find a the specified point in the polygon and set the contour/index
// values for it.  Returns true if a match found, false otherwise
static bool find_in_polygon( const Point3D p, const FGPolygon poly, 
			     int *contour, int *index ) {
    *contour = *index = -1;
    Point3D tmp;

    for ( int i = 0; i < poly.contours(); ++i ) {
	for ( int j = 0; j < poly.contour_size( i ); ++j ) {
	    tmp = poly.get_pt( i, j );
	    if ( tmp == p ) {
		*contour = i;
		*index = j;
		return true;
	    }
	}
    }

    return false;
}
#endif


#if 0
// Add points to keep line segment lengths under 1000'
static FGPolygon add_points( const FGPolygon& in_poly ) {
    FGPolygon result;
    result.erase();

    for ( int i = 0; i < in_poly.contours(); ++i ) {
	for ( int j = 0; j < in_poly.contour_size( i ) - 1; ++j ) {
	    gc_dist( in_poly.get_pt( i, j ),
		     in_poly.get_pt( i, j+1 ) );
	}
	gc_dist( in_poly.get_pt( i, in_poly.contour_size( i ) - 1 ),
		 in_poly.get_pt( i, 0 ) );
    }

    return result;
}
#endif


// Divide segment if there are other existing points on it, return the
// new polygon
void add_intermediate_nodes( int contour, const Point3D& start, 
			     const Point3D& end, const FGTriNodes& tmp_nodes,
			     FGPolygon *result )
// FGPolygon add_extra( const point_list& nodes, int n1, int n2 )
{
    bool found_extra = false;
    int extra_index = 0;
    int counter;
    double m, m1, b, b1, y_err, x_err, y_err_min, x_err_min;
    const_point_list_iterator current, last;
    point_list nodes = tmp_nodes.get_node_list();

    Point3D p0 = start;
    Point3D p1 = end;

    cout << "  add_intermediate_nodes()" << endl;

    double xdist = fabs(p0.x() - p1.x());
    double ydist = fabs(p0.y() - p1.y());
    cout << "xdist = " << xdist << "  ydist = " << ydist << endl;
    x_err_min = xdist + 1.0;
    y_err_min = ydist + 1.0;

    if ( xdist > ydist ) {
	cout << "use y = mx + b" << endl;

	// sort these in a sensible order
	if ( p0.x() > p1.x() ) {
	    Point3D tmp = p0;
	    p0 = p1;
	    p1 = tmp;
	}

	m = (p0.y() - p1.y()) / (p0.x() - p1.x());
	b = p1.y() - m * p1.x();

	// if ( temp ) {
	cout << "m = " << m << " b = " << b << endl;
	// }

	current = nodes.begin();
	last = nodes.end();
	counter = 0;
	for ( ; current != last; ++current ) {
	    cout << counter << endl;

	    if ( (current->x() > (p0.x() + FG_EPSILON)) 
		 && (current->x() < (p1.x() - FG_EPSILON)) ) {

		cout << "found a potential candidate " << *current << endl;
		y_err = fabs(current->y() - (m * current->x() + b));
		cout << "y_err = " << y_err << endl;

		if ( y_err < FG_PROXIMITY_EPSILON ) {
		    cout << "FOUND EXTRA SEGMENT NODE (Y)" << endl;
		    cout << p0 << " < " << *current << " < "
		         << p1 << endl;
		    found_extra = true;
		    if ( y_err < y_err_min ) {
			extra_index = counter;
			y_err_min = y_err;
		    }
		}
	    }
	    ++counter;
	}
    } else {
	cout << "use x = m1 * y + b1" << endl;

	// sort these in a sensible order
	if ( p0.y() > p1.y() ) {
	    Point3D tmp = p0;
	    p0 = p1;
	    p1 = tmp;
	}

	m1 = (p0.x() - p1.x()) / (p0.y() - p1.y());
	b1 = p1.x() - m1 * p1.y();

	// bool temp = true;
	// if ( temp ) {
	cout << "  m1 = " << m1 << " b1 = " << b1 << endl;
	// }

	// cout << "  should = 0 = " << fabs(p0.x() - (m1 * p0.y() + b1)) << endl;;
	// cout << "  should = 0 = " << fabs(p1.x() - (m1 * p1.y() + b1)) << endl;;

	current = nodes.begin();
	last = nodes.end();
	counter = 0;
	for ( ; current != last; ++current ) {
	    if ( (current->y() > (p0.y() + FG_EPSILON)) 
		 && (current->y() < (p1.y() - FG_EPSILON)) ) {

		cout << "found a potential candidate " << *current << endl;

		x_err = fabs(current->x() - (m1 * current->y() + b1));
		cout << "x_err = " << x_err << endl;

		// if ( temp ) {
		//   cout << "  (" << counter << ") x_err = " << x_err << endl;
		// }

		if ( x_err < FG_PROXIMITY_EPSILON ) {
		    cout << "FOUND EXTRA SEGMENT NODE (X)" << endl;
		    cout << p0 << " < " << *current << " < "
		         << p1 << endl;
		    found_extra = true;
		    if ( x_err < x_err_min ) {
			extra_index = counter;
			x_err_min = x_err;
		    }
		}
	    }
	    ++counter;
	}
    }

    if ( found_extra ) {
	// recurse with two sub segments
	// cout << "dividing " << s.get_n1() << " " << extra_index 
	// << " " << s.get_n2() << endl;
	add_intermediate_nodes( contour, p0, nodes[extra_index], tmp_nodes, 
				result );

	result->add_node( contour, nodes[extra_index] );

	add_intermediate_nodes( contour, nodes[extra_index], p1, tmp_nodes,
				result );
    } else {
	// this segment does not need to be divided
    }
}


// Search each segment for additional vertex points that may have been
// created elsewhere that lie on the segment and split it there to
// avoid "T" intersections.

static FGPolygon add_nodes_to_poly( const FGPolygon& poly, 
				    const FGTriNodes& tmp_nodes ) {
    FGPolygon result;
    Point3D p0, p1;

    cout << "add_nodes_to_poly" << endl;

    for ( int i = 0; i < poly.contours(); ++i ) {
	for ( int j = 0; j < poly.contour_size(i) - 1; ++j ) {
	    p0 = poly.get_pt( i, j );
	    p1 = poly.get_pt( i, j + 1 );

	    // add start of segment
	    result.add_node( i, p0 );

	    // add intermediate points
	    add_intermediate_nodes( i, p0, p1, tmp_nodes, &result );

	    // end of segment is beginning of next segment
	}
	p0 = poly.get_pt( i, poly.contour_size(i) - 1 );
	p1 = poly.get_pt( i, 0 );

	// add start of segment
	result.add_node( i, p0 );

	// add intermediate points
	add_intermediate_nodes( i, p0, p1, tmp_nodes, &result );

	// end of segment is beginning of next segment
	// 5/9/2000 CLO - this results in duplicating the last point
	// of a contour so I have removed this line.
	// result.add_node( i, p1 );

	// maintain original hole flag setting
	result.set_hole_flag( i, poly.get_hole_flag( i ) );
    }

    return result;
}


#if 0
// print polygon
static void print_poly( const FGPolygon& poly ) {
    for ( int i = 0; i < poly.contours(); ++i ) {
	cout << "contour " << i << endl;
 	for ( int j = 0; j < poly.contour_size( i ); ++j ) {
	    cout << "  " << poly.get_pt( i, j ) << endl;
	}
    }
}
#endif



// calculate the center of a list of points, by taking the halfway
// point between the min and max points.
Point3D calc_center( point_list& wgs84_nodes ) {
    Point3D p, min, max;

    if ( wgs84_nodes.size() ) {
	min = max = wgs84_nodes[0];
    } else {
	min = max = Point3D( 0 );
    }

    for ( int i = 0; i < (int)wgs84_nodes.size(); ++i ) {
	p = wgs84_nodes[i];

	if ( p.x() < min.x() ) { min.setx( p.x() ); }
	if ( p.y() < min.y() ) { min.sety( p.y() ); }
	if ( p.z() < min.z() ) { min.setz( p.z() ); }

	if ( p.x() > max.x() ) { max.setx( p.x() ); }
	if ( p.y() > max.y() ) { max.sety( p.y() ); }
	if ( p.z() > max.z() ) { max.setz( p.z() ); }
    }

    return ( min + max ) / 2.0;
}

// calculate the global bounding sphere.  Center is the center of the
// tile and zero elevation
double calc_bounding_radius( Point3D center, point_list& wgs84_nodes ) {
    double dist_squared;
    double radius_squared = 0;
    
    for ( int i = 0; i < (int)wgs84_nodes.size(); ++i ) {
        dist_squared = center.distance3Dsquared( wgs84_nodes[i] );
	if ( dist_squared > radius_squared ) {
            radius_squared = dist_squared;
        }
    }

    return sqrt(radius_squared);
}


// fix node elevations
point_list calc_elevations( const string& root, const point_list& geod_nodes ) {
    bool done = false;
    point_list result = geod_nodes;
    int i;
    FGArray array;

    // set all elevations to -9999
    for ( i = 0; i < (int)result.size(); ++i ) {
	result[i].setz( -9999.0 );
    }

    while ( !done ) {
	// find first node with -9999 elevation
	i = 0;
	while ( result[i].z() > -9000 ) { ++i; }

	if ( i < (int)result.size() ) {
	    FGBucket b( result[i].x(), result[i].y() );
	    string base = b.gen_base_path();

	    // try 3 arcsec dems first
	    string dem_path = root + "/DEM-3/" + base 
		+ "/" + b.gen_index_str() + ".dem";
	    cout << "dem_path = " << dem_path << endl;
	
	    if ( ! array.open(dem_path) ) {
		cout << "ERROR: cannot open 3 arcsec file " << dem_path << endl;
		cout << "trying 30 arcsec file" << endl;
		
		// try 30 arcsec dem
		dem_path = root + "/DEM-30/" + base 
		    + "/" + b.gen_index_str() + ".dem";
		cout << "dem_path = " << dem_path << endl;
		if ( ! array.open(dem_path) ) {
		    cout << "ERROR: cannot open 3 arcsec file " 
			 << dem_path << endl;
		}
	    }
	    array.parse( b );

	    // update all the non-updated elevations that are inside
	    // this dem file
	    double elev;
	    done = true;
	    for ( int j = 0; j < (int)result.size(); ++j ) {
		if ( result[j].z() < -9000 ) {
		    done = false;
		    cout << "interpolating for " << result[j] << endl;
		    elev = array.interpolate_altitude( result[j].x() * 3600.0,
						   result[j].y() * 3600.0 );
		    if ( elev > -9000 ) {
			result[j].setz( elev );
		    }
		}
	    }
	    array.close();
	} else {
	    done = true;
	}
    }


    return result;
}


// write out the structures to a file.  We assume that the groups come
// to us sorted by material property.  If not, things don't break, but
// the result won't be as optimal.
void write( const string& base, const FGBucket& b, const string& name,
	    Point3D gbs_center, double gbs_radius,
	    const point_list& wgs84_nodes, const point_list& normals,
	    const point_list& texcoords, 
	    const group_list& strips_v, const group_list& strips_tc, 
	    const string_list& strip_materials,
	    const group_list& fans_v, const group_list& fans_tc,
	    const string_list& fan_materials )
{
    Point3D p;

    string dir = base + "/" + b.gen_base_path();
    string command = "mkdir -p " + dir;
    system(command.c_str());

    // string file = dir + "/" + b.gen_index_str();
    string file = dir + "/" + name;
    cout << "Output file = " << file << endl;

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "w" )) == NULL ) {
	cout << "ERROR: opening " << file << " for writing!" << endl;
	exit(-1);
    }

    cout << "strip size = " << strips_v.size() << "  strip_materials = " 
	 << strip_materials.size() << endl;
    cout << "points = " << wgs84_nodes.size() << endl;
    cout << "tex coords = " << texcoords.size() << endl;
    // write headers
    fprintf(fp, "# FGFS Scenery\n");
    fprintf(fp, "# Version %s\n", FG_SCENERY_FILE_FORMAT);

    time_t calendar_time = time(NULL);
    struct tm *local_tm;
    local_tm = localtime( &calendar_time );
    char time_str[256];
    strftime( time_str, 256, "%a %b %d %H:%M:%S %Z %Y", local_tm);
    fprintf(fp, "# Created %s\n", time_str );
    fprintf(fp, "\n");

    // write global bounding sphere
    fprintf(fp, "# gbs %.5f %.5f %.5f %.2f\n",
	    gbs_center.x(), gbs_center.y(), gbs_center.z(), gbs_radius);
    fprintf(fp, "\n");

    // dump vertex list
    fprintf(fp, "# vertex list\n");
    for ( int i = 0; i < (int)wgs84_nodes.size(); ++i ) {
	p = wgs84_nodes[i] - gbs_center;
	
	fprintf(fp,  "v %.5f %.5f %.5f\n", p.x(), p.y(), p.z() );
    }
    fprintf(fp, "\n");

    fprintf(fp, "# vertex normal list\n");
    for ( int i = 0; i < (int)normals.size(); ++i ) {
	p = normals[i];
	fprintf(fp,  "vn %.5f %.5f %.5f\n", p.x(), p.y(), p.z() );
    }
    fprintf(fp, "\n");

    // dump texture coordinates
    fprintf(fp, "# texture coordinate list\n");
    for ( int i = 0; i < (int)texcoords.size(); ++i ) {
	p = texcoords[i];
	fprintf(fp,  "vt %.5f %.5f\n", p.x(), p.y() );
    }
    fprintf(fp, "\n");

    // dump triangle groups
    fprintf(fp, "# triangle groups\n");

    int start = 0;
    int end = 1;
    string material;
    while ( start < (int)strip_materials.size() ) {
	// find next group
	material = strip_materials[start];
	while ( (end < (int)strip_materials.size()) && 
		(material == strip_materials[end]) )
	{
	    // cout << "end = " << end << endl;
	    end++;
	}
	// cout << "group = " << start << " to " << end - 1 << endl;

	// make a list of points for the group
	point_list group_nodes;
	group_nodes.clear();
	Point3D bs_center;
	double bs_radius;
	for ( int i = start; i < end; ++i ) {
	    for ( int j = 0; j < (int)strips_v[i].size(); ++j ) {
		group_nodes.push_back( wgs84_nodes[ strips_v[i][j] ] );
		bs_center = calc_center( group_nodes );
		bs_radius = calc_bounding_radius( bs_center, group_nodes );
	    }
	}

	// write group headers
	fprintf(fp, "\n");
	fprintf(fp, "# usemtl %s\n", material.c_str());
	fprintf(fp, "# bs %.4f %.4f %.4f %.2f\n",
		bs_center.x(), bs_center.y(), bs_center.z(), bs_radius);

	// write groups
	for ( int i = start; i < end; ++i ) {
	    fprintf(fp, "ts");
	    for ( int j = 0; j < (int)strips_v[i].size(); ++j ) {
		fprintf(fp, " %d/%d", strips_v[i][j], strips_tc[i][j] );
	    }
	    fprintf(fp, "\n");
	}

	start = end;
	end = start + 1;
    }
    fclose(fp);

    command = "gzip --force --best " + file;
    system(command.c_str());
}


// update index
void write_index(const string& base, const FGBucket& b, const string& name) {
    string dir = base + "/" + b.gen_base_path();
    string command = "mkdir -p " + dir;
    system(command.c_str());

    string file = dir + "/" + b.gen_index_str() + ".ind";
    // string file = dir + "/" + name;
    cout << "Output file = " << file << endl;

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "a" )) == NULL ) {
	cout << "ERROR: opening " << file << " for writing!" << endl;
	exit(-1);
    }

    fprintf( fp, "OBJECT %s\n", name.c_str() );
    fclose( fp );
}


void write_boundary( const string& base, const FGBucket& b, 
		     const FGPolygon& bounds, long int p_index )
{
    Point3D p;

    string dir = base + "/" + b.gen_base_path();
    string command = "mkdir -p " + dir;
    system(command.c_str());

    string file = dir + "/" + b.gen_index_str();

    char poly_index[256];
    sprintf( poly_index, "%ld", p_index );
    file += ".";
    file += poly_index;

    cout << "Output file = " << file << endl;

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "w" )) == NULL ) {
	cout << "ERROR: opening " << file << " for writing!" << endl;
	exit(-1);
    }

    fprintf( fp, "Hole\n" );

    fprintf( fp, "%d\n", bounds.contours() );
    for ( int i = 0; i < bounds.contours(); ++i ) {
	fprintf( fp, "%d\n", bounds.contour_size(i) );
	fprintf( fp, "%d\n", bounds.get_hole_flag(i) );
	for ( int j = 0; j < bounds.contour_size(i); ++j ) {
	    p = bounds.get_pt( i, j );
	    fprintf( fp, "%.15f  %.15f\n", p.x(), p.y() );
	}
    }
    fclose( fp );
}


// strip trailing spaces
static void my_chomp( string& str ) {
    cout << "my_chomp()" << endl;
    cout << "'" << str.substr( str.length() - 1, 1 ) << "'" << endl;
    while ( str.substr( str.length() - 1, 1 ) == " " ) {
	str = str.substr( 0, str.length() - 1 );
	cout << "'" << str.substr( str.length() - 1, 1 ) << "'" << endl;
    }
}


// build 3d airport
void build_airport( string airport_raw, string_list& runways_raw,
		    const string& root ) {

    poly_list rwy_nodes, rwy_strips, rwy_txs;
    FGPolygon runway, runway_a, runway_b, result, result_a, result_b;
    FGPolygon base;
    point_list apt_pts;
    Point3D p;

    FGPolygon accum;
    accum.erase();

    // parse main airport information
    double apt_lon, apt_lat;
    int elev;

    cout << airport_raw << endl;
    string apt_type = airport_raw.substr(0, 1);
    string apt_code = airport_raw.substr(2, 4); my_chomp( apt_code );
    string apt_lat_str = airport_raw.substr(7, 10);
    apt_lat = atof( apt_lat_str.c_str() );
    string apt_lon_str = airport_raw.substr(18, 11);
    apt_lon = atof( apt_lon_str.c_str() );
    string apt_elev = airport_raw.substr(30, 5);
    elev = atoi( apt_elev.c_str() );
    string apt_use = airport_raw.substr(36, 1);
    string apt_twr = airport_raw.substr(37, 1);
    string apt_bldg = airport_raw.substr(38, 1);
    string apt_name = airport_raw.substr(40);

    /*
    cout << "  type = " << apt_type << endl;
    cout << "  code = " << apt_code << endl;
    cout << "  lat  = " << apt_lat << endl;
    cout << "  lon  = " << apt_lon << endl;
    cout << "  elev = " << apt_elev << " " << elev << endl;
    cout << "  use  = " << apt_use << endl;
    cout << "  twr  = " << apt_twr << endl;
    cout << "  bldg = " << apt_bldg << endl;
    cout << "  name = " << apt_name << endl;
    */

    FGBucket b( apt_lon, apt_lat );
    Point3D center_geod( b.get_center_lon() * DEG_TO_RAD,
			 b.get_center_lat() * DEG_TO_RAD, 0 );
    Point3D gbs_center = fgGeodToCart( center_geod );
    cout << b.gen_base_path() << "/" << b.gen_index_str() << endl;

    // Ignore any seaplane bases
    if ( apt_type == "S" ) {
	return;
    }

    // parse runways and generate the vertex list
    runway_list runways;
    runways.clear();
    string rwy_str;

    for (int i = 0; i < (int)runways_raw.size(); ++i ) {
	rwy_str = runways_raw[i];

	FGRunway rwy;

	cout << rwy_str << endl;
	rwy.rwy_no = rwy_str.substr(2, 4);

	string rwy_lat = rwy_str.substr(6, 10);
	rwy.lat = atof( rwy_lat.c_str() );

	string rwy_lon = rwy_str.substr(17, 11);
	rwy.lon = atof( rwy_lon.c_str() );

	string rwy_hdg = rwy_str.substr(29, 7);
	rwy.heading = atof( rwy_hdg.c_str() );

	string rwy_len = rwy_str.substr(36, 7);
	rwy.length = atoi( rwy_len.c_str() );

	string rwy_width = rwy_str.substr(43, 4);
	rwy.width = atoi( rwy_width.c_str() );

	rwy.surface_flags = rwy_str.substr(47, 4);
	rwy.end1_flags = rwy_str.substr(52, 8);
	rwy.end2_flags = rwy_str.substr(61, 8);

	/*
	cout << "  no    = " << rwy_no << endl;
	cout << "  lat   = " << rwy_lat << " " << lat << endl;
	cout << "  lon   = " << rwy_lon << " " << lon << endl;
	cout << "  hdg   = " << rwy_hdg << " " << hdg << endl;
	cout << "  len   = " << rwy_len << " " << len << endl;
	cout << "  width = " << rwy_width << " " << width << endl;
	cout << "  sfc   = " << rwy_sfc << endl;
	cout << "  end1  = " << rwy_end1 << endl;
	cout << "  end2  = " << rwy_end2 << endl;
	*/

	runways.push_back( rwy );
    }

    for ( int i = 0; i < (int)runways.size(); ++i ) {
	runway = gen_runway_w_mid( runways[i] );

	// runway half "a"
	runway_a.erase();
	runway_a.add_node( 0, runway.get_pt(0, 0) );
	runway_a.add_node( 0, runway.get_pt(0, 1) );
	runway_a.add_node( 0, runway.get_pt(0, 2) );
	runway_a.add_node( 0, runway.get_pt(0, 5) );

	// runway half "b"
	runway_b.erase();
	runway_b.add_node( 0, runway.get_pt(0, 5) );
	runway_b.add_node( 0, runway.get_pt(0, 2) );
	runway_b.add_node( 0, runway.get_pt(0, 3) );
	runway_b.add_node( 0, runway.get_pt(0, 4) );
	
	cout << "raw runway pts (a half)" << endl;
	for ( int j = 0; j < runway_a.contour_size( 0 ); ++j ) {
	    p = runway_a.get_pt(0, j);
	    cout << " point = " << p << endl;
	}
	cout << "raw runway pts (b half)" << endl;
	for ( int j = 0; j < runway_b.contour_size( 0 ); ++j ) {
	    p = runway_b.get_pt(0, j);
	    cout << " point = " << p << endl;
	}
	
	result_a = polygon_diff( runway_a, accum );
	rwy_nodes.push_back( result_a );
	cout << "result_a = " << result_a.contours() << endl;
	accum = polygon_union( runway_a, accum );

	result_b = polygon_diff( runway_b, accum );
	rwy_nodes.push_back( result_b );
	cout << "result_b = " << result_b.contours() << endl;
	accum = polygon_union( runway_b, accum );

	char tmpa[256], tmpb[256];
	sprintf( tmpa, "a%d", i );
	sprintf( tmpb, "b%d", i );
	write_polygon( result_a, tmpa );
	write_polygon( result_b, tmpb );

	// print runway points
	cout << "clipped runway pts (a)" << endl;
	for ( int j = 0; j < result_a.contours(); ++j ) {
	    for ( int k = 0; k < result_a.contour_size( j ); ++k ) {
		p = result_a.get_pt(j, k);
		cout << " point = " << p << endl;
	    }
	}

	// print runway points
	cout << "clipped runway pts (b)" << endl;
	for ( int j = 0; j < result_b.contours(); ++j ) {
	    for ( int k = 0; k < result_b.contour_size( j ); ++k ) {
		p = result_b.get_pt(j, k);
		cout << " point = " << p << endl;
	    }
	}

	base = gen_runway_area( runways[i], 1.01, 1.5 );

	// add base to apt_pts (for convex hull of airport area)
	for ( int j = 0; j < base.contour_size( 0 ); ++j ) {
	    p = base.get_pt(0, j);
	    // cout << " point = " << p << endl;
	    apt_pts.push_back( p );
	}
    }

    if ( apt_pts.size() == 0 ) {
	cout << "no airport points generated" << endl;
	return;
    }

    // generate convex hull
    FGPolygon hull = convex_hull(apt_pts);
    FGPolygon base_nodes = polygon_diff( hull, accum );

    // add segments to polygons to remove any possible "T"
    // intersections
    FGTriNodes tmp_nodes;

    // build temporary node list
    for ( int k = 0; k < (int)rwy_nodes.size(); ++k ) {
	for ( int i = 0; i < rwy_nodes[k].contours(); ++i ) {
	    for ( int j = 0; j < rwy_nodes[k].contour_size( i ); ++j ) {
		tmp_nodes.unique_add( rwy_nodes[k].get_pt(i, j) );
	    }
	}
    }
    for ( int i = 0; i < base_nodes.contours(); ++i ) {
	for ( int j = 0; j < base_nodes.contour_size( i ); ++j ) {
	    tmp_nodes.unique_add( base_nodes.get_pt(i, j) );
	}
    }

    point_list ttt = tmp_nodes.get_node_list();
    for ( int i = 0; i < (int)ttt.size(); ++i ) {
	char name[256];
	sprintf(name, "p%d", i );
	FILE *fp = fopen( name, "w" );
	fprintf(fp, "%.8f %.8f\n", ttt[i].x(), ttt[i].y());
	fclose(fp);
    }

    for ( int i = 0; i < base_nodes.contours(); ++i ) {
	char name[256];
	sprintf(name, "l%d", i );
	FILE *fp = fopen( name, "w" );

	for ( int j = 0; j < base_nodes.contour_size( i ) - 1; ++j ) {
	    Point3D p0 = base_nodes.get_pt(i, j);
	    Point3D p1 = base_nodes.get_pt(i, j + 1);
	    fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	    fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	}
	Point3D p0 = base_nodes.get_pt(i, base_nodes.contour_size( i ) - 1);
	Point3D p1 = base_nodes.get_pt(i, 0);
	fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	fclose(fp);
    }

    for ( int k = 0; k < (int)rwy_nodes.size(); ++k ) {
	rwy_nodes[k] = add_nodes_to_poly( rwy_nodes[k], tmp_nodes );
    }
    base_nodes = add_nodes_to_poly( base_nodes, tmp_nodes );
   
    // new stripper approach
    cout << "Ready to try new striper" << endl;
    cout << "First calculate a 'point inside' for each contour and hole" 
	 << endl;
    write_polygon( base_nodes, "base" );

    /* 1 */ calc_points_inside( base_nodes );
    for ( int i = 0; i < base_nodes.contours(); ++i ) {
	cout << base_nodes.get_point_inside( i ) << endl;
    }
    /* 2 */ triele_list base_tris = polygon_tesselate( base_nodes, -1 );

    // generate convex hull and strip version
    FGPolygon base_strips = polygon_to_tristrip( base_nodes );

    for ( int i = 0; i < base_strips.contours(); ++i ) {
	char name[256];
	sprintf(name, "s%d", i );
	FILE *fp = fopen( name, "w" );

	for ( int j = 0; j < base_strips.contour_size( i ) - 1; ++j ) {
	    Point3D p0 = base_strips.get_pt(i, j);
	    Point3D p1 = base_strips.get_pt(i, j + 1);
	    fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	    fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	}
	Point3D p0 = base_strips.get_pt(i, base_strips.contour_size( i ) - 1);
	Point3D p1 = base_strips.get_pt(i, 0);
	fprintf(fp, "%.8f %.8f\n", p0.x(), p0.y());
	fprintf(fp, "%.8f %.8f\n", p1.x(), p1.y());
	fclose(fp);
    }

    // generate strips and texture coordinates
    for ( int i = 0; i < (int)runways.size(); ++i ) {
	FGPolygon strip_a, strip_b, tc_a, tc_b;

	strip_a = polygon_to_tristrip( rwy_nodes[2 * i] );
	tc_a = rwy_calc_tex_coords( runways[i], 0.0, strip_a );
	rwy_strips.push_back( strip_a );
	rwy_txs.push_back( tc_a );

	strip_b = polygon_to_tristrip( rwy_nodes[2 * i + 1] );
	tc_b = rwy_calc_tex_coords( runways[i], 180.0, strip_b );
	rwy_strips.push_back( strip_b );
	rwy_txs.push_back( tc_b );
    }

    //
    // We should now have the runway polygons all generated with their
    // corresponding strips and texture coordinates, and the
    // surrounding base area.
    //
    // Next we need to create the output lists ... vertices, normals,
    // texture coordinates, and tri-strips
    //

    // traverse the strip list and create ordered node and texture
    // coordinate lists

    FGTriNodes nodes, texcoords;
    nodes.clear();
    texcoords.clear();

    group_list strips_v; strips_v.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();

    Point3D tc;
    int index;
    int_list strip_v;
    int_list strip_tc;

    for ( int k = 0; k < (int)rwy_strips.size(); ++k ) {
	cout << "strip " << k << endl;
	FGPolygon strip_poly = rwy_strips[k];
	for ( int i = 0; i < strip_poly.contours(); ++i ) {
	    strip_v.clear();
	    strip_tc.clear();
	    for ( int j = 0; j < strip_poly.contour_size(i); ++j ) {
		p = strip_poly.get_pt( i, j );
		index = nodes.unique_add( p );
		strip_v.push_back( index );
		tc = rwy_txs[k].get_pt( i, j );
		index = texcoords.unique_add( tc );
		strip_tc.push_back( index );
	    }
	    strips_v.push_back( strip_v );
	    strips_tc.push_back( strip_tc );
	    strip_materials.push_back( "Concrete" );
	}
    }
    
    // add base points
    point_list base_txs; 
    int_list base_tc;
    for ( int i = 0; i < base_strips.contours(); ++i ) {
	strip_v.clear();
	strip_tc.clear();
	for ( int j = 0; j < base_strips.contour_size(i); ++j ) {
	    p = base_strips.get_pt( i, j );
	    index = nodes.unique_add( p );
	    strip_v.push_back( index );
	}
	strips_v.push_back( strip_v );
	strip_materials.push_back( "Grass" );

	base_txs.clear();
	base_txs = calc_tex_coords( b, nodes.get_node_list(), strip_v );

	base_tc.clear();
	for ( int j = 0; j < (int)base_txs.size(); ++j ) {
	    tc = base_txs[j];
	    cout << "base_tc = " << tc << endl;
	    index = texcoords.simple_add( tc );
	    base_tc.push_back( index );
	}
	strips_tc.push_back( base_tc );
    }

    // calculate node elevations
    point_list geod_nodes = calc_elevations( root, nodes.get_node_list() );

    // calculate wgs84 mapping of nodes
    point_list wgs84_nodes;
    for ( int i = 0; i < (int)geod_nodes.size(); ++i ) {
	p.setx( geod_nodes[i].x() * DEG_TO_RAD );
	p.sety( geod_nodes[i].y() * DEG_TO_RAD );
	p.setz( geod_nodes[i].z() );
	wgs84_nodes.push_back( fgGeodToCart( p ) );
    }
    double gbs_radius = calc_bounding_radius( gbs_center, wgs84_nodes );

    // calculate normal(s) for this airport
    p.setx( rwy_strips[0].get_pt(0, 0).x() * DEG_TO_RAD );
    p.sety( rwy_strips[0].get_pt(0, 0).y() * DEG_TO_RAD );
    p.setz( 0 );
    Point3D tmp = fgGeodToCart( p );
    // cout << "geod = " << p << endl;
    // cout << "cart = " << tmp << endl;

    sgdVec3 vn;
    sgdSetVec3( vn, tmp.x(), tmp.y(), tmp.z() );
    sgdNormalizeVec3( vn );
    point_list normals;
    normals.clear();
    for ( int i = 0; i < (int)nodes.size(); ++i ) {
	normals.push_back( Point3D( vn[0], vn[1], vn[2] ) );
    }

    group_list fans_v; fans_v.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials;
    fan_materials.clear();

    string objpath = root + "/AirportObj";
    string name = apt_code;

    write( objpath, b, name, gbs_center, gbs_radius, 
	   wgs84_nodes, normals,
	   texcoords.get_node_list(), 
	   strips_v, strips_tc, strip_materials, 
	   fans_v, fans_tc, fan_materials );

    write_index( objpath, b, name );

    string holepath = root + "/AirportArea";
    long int poly_index = poly_index_next();
    write_boundary( holepath, b, hull, poly_index );
}
