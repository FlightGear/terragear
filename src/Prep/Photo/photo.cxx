// photo.cxx -- main loop
//
// Written by Curtis Olson, started May 2001.
//
// Copyright (C) 2001  Curtis L. Olson  - curt@flightgear.org
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


#include <simgear/compiler.h>

#include <stdio.h>
#include <stdlib.h>

#include STL_STRING

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include <Array/array.hxx>
#include <Geometry/trinodes.hxx>
#include <Output/output.hxx>
#include <Polygon/index.hxx>
#include <Polygon/split.hxx>
#include <Polygon/polygon.hxx>

SG_USING_STD(string);


const int MAX_XDIV = 16;
const int MAX_YDIV = 16;


// fix node elevations
point_list calc_elevations( const string& root, const point_list& geod_nodes ) {
    bool done = false;
    point_list result = geod_nodes;
    int i, j;
    FGArray array;

    // set all elevations to -9999
    for ( i = 0; i < (int)result.size(); ++i ) {
	result[i].setz( -9999.0 );
    }

    while ( !done ) {
	// find first node with -9999 elevation
	i = 0;
	while ( (result[i].z() > -9000) && (i < (int)result.size()) ) {
	    ++i;
	}

	if ( i < (int)result.size() ) {
	    SGBucket b( result[i].x(), result[i].y() );
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
	    for ( j = 0; j < (int)result.size(); ++j ) {
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


int main( int argc, char **argv ) {
    int i, j;

    sglog().setLogLevels( SG_ALL, SG_DEBUG );

    // check args
    if ( argc != 13 ) {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Usage " << argv[0]
                << " root image(base) xdiv ydiv x0 y0 x1 y1 x2 y2 x3 y3" );
        exit(-1);
    }

    string root = argv[1];
    string image = argv[2];

    int xdiv = atoi( argv[3] );
    int ydiv = atoi( argv[4] );
    if ( xdiv > MAX_XDIV || xdiv < 1 || ydiv > MAX_YDIV || ydiv < 1 ) {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "{x,y}div must be in the range of 1 - " << MAX_XDIV );
        exit(-1);
    }
        
    double x0 = atof( argv[5] );
    double y0 = atof( argv[6] );

    double x1 = atof( argv[7] );
    double y1 = atof( argv[8] );

    double x2 = atof( argv[9] );
    double y2 = atof( argv[10] );

    double x3 = atof( argv[11] );
    double y3 = atof( argv[12] );

    // generate vertices

    FGTriNodes nodes; nodes.clear();

    double dx0 = (x3 - x0) / ydiv;
    double dy0 = (y3 - y0) / ydiv;
    double dx1 = (x2 - x1) / ydiv;
    double dy1 = (y2 - y1) / ydiv;

    double xarray[32][32];
    double yarray[32][32];
    for ( i = 0; i <= xdiv; ++i ) {
        double tx0 = x0 + i * dx0;
        double ty0 = y0 + i * dy0;
        double tx1 = x1 + i * dx1;
        double ty1 = y1 + i * dy1;
        double dx = (tx1 - tx0) / xdiv;
        double dy = (ty1 - ty0) / xdiv;
        for ( j = 0; j <= ydiv; ++j ) {
            xarray[i][j] = tx0 + j * dx;
            yarray[i][j] = ty0 + j * dy;
            cout << "(" << xarray[i][j] << "," << yarray[i][j] << ")" << endl;
            nodes.simple_add( Point3D(xarray[i][j], yarray[i][j], 0) );
        }
        cout << endl;
    }

    // create the object structures

    // geodetic nodes
    point_list geod_nodes = calc_elevations( root, nodes.get_node_list() );

    // texture coordinates
    FGTriNodes texcoords; texcoords.clear();
    texcoords.simple_add( Point3D( 0.0, 0.0, 0.0 ) );
    texcoords.simple_add( Point3D( 1.0, 0.0, 0.0 ) );
    texcoords.simple_add( Point3D( 1.0, 1.0, 0.0 ) );
    texcoords.simple_add( Point3D( 0.0, 1.0, 0.0 ) );

    // triangles
    group_list strips_v; strips_v.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();
    int_list strip_v;
    int_list strip_tc;

    int count = 0;
    for ( i = 0; i < xdiv; ++i ) {
        for ( j = 0; j < ydiv; ++j ) {
            strip_v.clear();
            strip_v.push_back( count );
            strip_v.push_back( count + 1 );
            strip_v.push_back( count + xdiv + 1 );
            strip_v.push_back( count + xdiv + 2 );

            strip_tc.clear();
            strip_tc.push_back( 0 );
            strip_tc.push_back( 1 );
            strip_tc.push_back( 3 );
            strip_tc.push_back( 2 );

            char bufx[5], bufy[5];
            snprintf( bufx, 5, "%X", i );
            snprintf( bufy, 5, "%X", j );
            string material = image;
            material += bufx;
            material += bufy;
            material += ".png";

            strips_v.push_back( strip_v );
            strips_tc.push_back( strip_tc );
            strip_materials.push_back( material );

            ++count;
        }
        ++count;
    }

    // wgs84 cartesian nodes
    point_list wgs84_nodes; wgs84_nodes.clear();
    for ( i = 0; i < (int)geod_nodes.size(); ++i ) {
        Point3D p;
	p.setx( geod_nodes[i].x() * SGD_DEGREES_TO_RADIANS );
	p.sety( geod_nodes[i].y() * SGD_DEGREES_TO_RADIANS );
	p.setz( geod_nodes[i].z() );
	wgs84_nodes.push_back( sgGeodToCart( p ) );
    }

    // bounding sphere
    Point3D center_geod = Point3D( ((x0 + x2) / 2) * SGD_DEGREES_TO_RADIANS,
                                   ((y0 + y2) / 2) * SGD_DEGREES_TO_RADIANS,
                                   0 );
    Point3D gbs_center = sgGeodToCart( center_geod );
    cout << "gbs center = " << gbs_center << endl;
    float gbs_radius = sgCalcBoundingRadius( gbs_center, wgs84_nodes );

    // normals
    point_list normals = wgs84_nodes;
    sgdVec3 vn;
    for ( i = 0; i < (int)normals.size(); ++i ) {
        sgdSetVec3( vn, normals[i].x(), normals[i].y(), normals[i].z() );
        sgdNormalizeVec3( vn );
 	normals[i] = Point3D( vn[0], vn[1], vn[2] );
        cout << normals[i] << endl;
    }

    // build the object
    SGBinObject obj;

    // null structures
    group_list tris_v; tris_v.clear();
    group_list tris_tc; tris_tc.clear();
    string_list tri_materials; tri_materials.clear();

    group_list fans_v; fans_v.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    obj.set_gbs_center( gbs_center );
    cout << "gbs center = " << gbs_center << endl;
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals );
    obj.set_texcoords( texcoords.get_node_list() );
    obj.set_tris_v( tris_v );
    obj.set_tris_tc( tris_tc ); 
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_tc( strips_tc ); 
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );

    // write the object
    string objpath = root + "/PhotoObj";
    string name = image;
    SGBucket b( center_geod.x() * SGD_RADIANS_TO_DEGREES,
                center_geod.y() * SGD_RADIANS_TO_DEGREES);

    bool result = obj.write_bin( objpath, name, b );
    if ( !result ) {
	cout << "error writing file. :-(" << endl;
	exit(-1);
    }

    // write the index entry
    write_index( objpath, b, name );

    // write the 'hole' polygon
    FGPolygon hole; hole.erase();
    Point3D p;

    p = Point3D( x0, y0, 0 ); hole.add_node( 0, p );
    p = Point3D( x1, y1, 0 ); hole.add_node( 0, p );
    p = Point3D( x2, y2, 0 ); hole.add_node( 0, p );
    p = Point3D( x3, y3, 0 ); hole.add_node( 0, p );

    // initialize persistant polygon counter
    string counter_file = root + "/poly_counter";
    poly_index_init( counter_file );

    string holepath = root + "/PhotoArea";
    split_polygon( holepath, HoleArea, hole );

    return 0;
}
