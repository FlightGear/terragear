// output.cxx -- routines to output a polygon model of an airport
//
// Written by Curtis Olson, started September 1999.
//
// Copyright (C) 1999 - 2000  Curtis L. Olson  - curt@flightgear.org
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


#ifndef _TG_OUTPUT_H
#define _TG_OUTPUT_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <simgear/compiler.h>

#include <stdio.h>
#include <time.h>

#include <list>
#include STL_STRING

#include <simgear/bucket/newbucket.hxx>

#include <Polygon/polygon.hxx>

#include "scenery_version.hxx"

#include "output.hxx"

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

FG_USING_STD( cout );
FG_USING_STD( endl );


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


// calculate the center of a list of points, by taking the halfway
// point between the min and max points.
static Point3D calc_center( point_list& wgs84_nodes ) {
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


// write out the structures to a file.  We assume that the groups come
// to us sorted by material property.  If not, things don't break, but
// the result won't be as optimal.
void write_obj( const string& base, const FGBucket& b, const string& name,
		Point3D gbs_center, double gbs_radius,
		const point_list& wgs84_nodes, const point_list& normals,
		const point_list& texcoords, 
		const group_list& tris_v, const group_list& tris_tc, 
		const string_list& tri_materials,
		const group_list& strips_v, const group_list& strips_tc, 
		const string_list& strip_materials,
		const group_list& fans_v, const group_list& fans_tc,
		const string_list& fan_materials )
{
    Point3D p;
    int i, j;

    string dir = base + "/" + b.gen_base_path();
    string command = "mkdir -p " + dir;
#ifdef _MSC_VER
    fg_mkdir( dir.c_str() );
#else
    system(command.c_str());
#endif

    // string file = dir + "/" + b.gen_index_str();
    string file = dir + "/" + name;
    cout << "Output file = " << file << endl;

    FILE *fp;
    if ( (fp = fopen( file.c_str(), "w" )) == NULL ) {
	cout << "ERROR: opening " << file << " for writing!" << endl;
	exit(-1);
    }

    cout << "triangles size = " << tris_v.size() << "  tri_materials = " 
	 << tri_materials.size() << endl;
    cout << "strips size = " << strips_v.size() << "  strip_materials = " 
	 << strip_materials.size() << endl;
    cout << "fans size = " << fans_v.size() << "  fan_materials = " 
	 << fan_materials.size() << endl;

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
    for ( i = 0; i < (int)wgs84_nodes.size(); ++i ) {
	p = wgs84_nodes[i] - gbs_center;
	
	fprintf(fp,  "v %.5f %.5f %.5f\n", p.x(), p.y(), p.z() );
    }
    fprintf(fp, "\n");

    fprintf(fp, "# vertex normal list\n");
    for ( i = 0; i < (int)normals.size(); ++i ) {
	p = normals[i];
	fprintf(fp,  "vn %.5f %.5f %.5f\n", p.x(), p.y(), p.z() );
    }
    fprintf(fp, "\n");

    // dump texture coordinates
    fprintf(fp, "# texture coordinate list\n");
    for ( i = 0; i < (int)texcoords.size(); ++i ) {
	p = texcoords[i];
	fprintf(fp,  "vt %.5f %.5f\n", p.x(), p.y() );
    }
    fprintf(fp, "\n");

    // dump individual triangles if they exist
    if ( tris_v.size() > 0 ) {
	fprintf(fp, "# triangle groups\n");

	int start = 0;
	int end = 1;
	string material;
	while ( start < (int)tri_materials.size() ) {
	    // find next group
	    material = tri_materials[start];
	    while ( (end < (int)tri_materials.size()) && 
		    (material == tri_materials[end]) )
	    {
		// cout << "end = " << end << endl;
		end++;
	    }
	    // cout << "group = " << start << " to " << end - 1 << endl;

	    // make a list of points for the group
	    point_list group_nodes;
	    group_nodes.clear();
	    Point3D bs_center;
	    double bs_radius = 0;
	    for ( i = start; i < end; ++i ) {
		for ( j = 0; j < (int)tris_v[i].size(); ++j ) {
		    group_nodes.push_back( wgs84_nodes[ tris_v[i][j] ] );
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
	    for ( i = start; i < end; ++i ) {
		fprintf(fp, "f");
		for ( j = 0; j < (int)tris_v[i].size(); ++j ) {
		    fprintf(fp, " %d/%d", tris_v[i][j], tris_tc[i][j] );
		}
		fprintf(fp, "\n");
	    }

	    start = end;
	    end = start + 1;
	}
    }

    // dump triangle groups
    if ( strips_v.size() > 0 ) {
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
	    double bs_radius = 0;
	    for ( i = start; i < end; ++i ) {
		for ( j = 0; j < (int)strips_v[i].size(); ++j ) {
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
	    for ( i = start; i < end; ++i ) {
		fprintf(fp, "ts");
		for ( j = 0; j < (int)strips_v[i].size(); ++j ) {
		    fprintf(fp, " %d/%d", strips_v[i][j], strips_tc[i][j] );
		}
		fprintf(fp, "\n");
	    }
	    
	    start = end;
	    end = start + 1;
	}
    }

    // close the file
    fclose(fp);

    command = "gzip --force --best " + file;
    system(command.c_str());
}


// update index
void write_index(const string& base, const FGBucket& b, const string& name) {
    string dir = base + "/" + b.gen_base_path();
#ifdef _MSC_VER
    fg_mkdir( dir.c_str() );
#else
    string command = "mkdir -p " + dir;
    system(command.c_str());
#endif

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
#ifdef _MSC_VER
    fg_mkdir( dir.c_str() );
#else
    string command = "mkdir -p " + dir;
    system(command.c_str());
#endif

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


#endif // _TG_OUTPUT_H
