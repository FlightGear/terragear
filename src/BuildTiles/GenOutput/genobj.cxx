// genobj.hxx -- Generate the flight gear "obj" file format from the
//               triangle output
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
// $Id: genobj.cxx,v 1.26 2004-11-19 22:25:49 curt Exp $


#include <time.h>

#include <simgear/compiler.h>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/misc/texcoord.hxx>
#include <simgear/debug/logstream.hxx>

#include <Output/output.hxx>
#include <Clipper/priorities.hxx>

#include <Osgb36/osgbtc.hxx>
#include <Osgb36/uk.hxx>

#include "genobj.hxx"

using std::cout;
using std::endl;
using std::string;


// calculate the global bounding sphere.  Center is the center of the
// tile and zero elevation
void TGGenOutput::calc_gbs( TGConstruct& c ) {
    point_list wgs84_nodes = c.get_wgs84_nodes();
    gbs_center = SGVec3d::fromGeod( c.get_bucket().get_center() );

    double dist_squared, radius_squared = 0;
    for ( unsigned int i = 0; i < wgs84_nodes.size(); ++i ) {
        dist_squared = distSqr(gbs_center, wgs84_nodes[i].toSGVec3d());
        if ( dist_squared > radius_squared ) {
            radius_squared = dist_squared;
        }
    }
    gbs_radius = sqrt(radius_squared);
}

// build the necessary output structures based on the triangulation
// data
int TGGenOutput::build_fans( TGConstruct& c ) {
//  TGTriNodes trinodes = c.get_tri_nodes();
    TGNodes* nodes = c.get_nodes();

    string tile_id = c.get_bucket().gen_index_str();

    // copy the geodetic node list into this class
    geod_nodes = nodes->get_geod_nodes();

    // copy the triangle list into this class
    tri_elements = c.get_tri_elements();

    // build the trifan list
    cout << "total triangles = " << tri_elements.size() << endl;

    TGGenFans f;

    // Sort the triangles by area type
    // TODO: Need to sort on Material type - going to make the attribute an index
    //       into an array of texparams / material names
    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        triele_list area_tris;
        area_tris.erase( area_tris.begin(), area_tris.end() );

        const_triele_list_iterator t_current = tri_elements.begin();
        const_triele_list_iterator t_last = tri_elements.end();

        for ( ; t_current != t_last; ++t_current ) {
            if ( t_current->get_attribute() == i ) {
                area_tris.push_back( *t_current );
            }
        }

        if ( (int)area_tris.size() > 0 ) {
            cout << "generating fans for area = " << i << endl;
            primitives[i] = f.greedy_build( area_tris );
        }
    }

    // build the texture coordinate list and make a parallel structure
    // to the fan list for pointers into the texture list
    cout << "calculating texture coordinates" << endl;
    cout << "c.get_useUKGrid() = " << c.get_useUKGrid() << endl;
    tex_coords.clear();
    std::vector < SGGeod > convGeodNodes;
    for ( unsigned int k = 0; k < geod_nodes.size(); k++ ) {
        Point3D node = geod_nodes[k];
        convGeodNodes.push_back( SGGeod::fromDegM( node.x(), node.y(), node.z() ) );
    }

    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        // cout << " area = " << i << endl;
        for ( unsigned int j = 0; j < primitives[i].size(); ++j ) {
            SG_LOG( SG_CLIPPER, SG_INFO, tile_id << ": Texturing " << get_area_name( (AreaType)i ) << " primitive " << j+1 << " of " << primitives[i].size() );

            SGBucket b = c.get_bucket();
            Point3D ourPosition;

            ourPosition.setlon(b.get_chunk_lon());
            ourPosition.setlat(b.get_chunk_lat());

            int_list ti_list;
            ti_list.clear();

            //dcl - here read the flag to check if we are building UK grid
            //If so - check if the bucket is within the UK lat & lon
            if( (c.get_useUKGrid()) && (isInUK(ourPosition)) ) {
                point_list tp_list;
                tp_list = UK_calc_tex_coords( b, geod_nodes, primitives[i][j], 1.0 );
                for ( unsigned int k = 0; k < tp_list.size(); ++k ) {
                    // cout << "  tc = " << tp_list[k] << endl;
                    int index = tex_coords.simple_add( tp_list[k] );
                    ti_list.push_back( index );
                }
            } else {
                std::vector< SGVec2f > tp_list = sgCalcTexCoords( b, convGeodNodes, primitives[i][j] );
                for ( unsigned int k = 0; k < tp_list.size(); ++k ) {
                    SGVec2f tc = tp_list[k];
                    // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
                    int index = tex_coords.simple_add( Point3D( tc.x(), tc.y(), 0 ) );
                    ti_list.push_back( index );
                }
            }

            textures[i].push_back( ti_list );
        }
    }

    // calculate the global bounding sphere
    calc_gbs( c );
    cout << "center = " << gbs_center << " radius = " << gbs_radius << endl;

    return 1;
}


static opt_list tgGenTris( const triele_list tris ) {
    triele_list remaining = tris;
    opt_list tri_lists;
    int_list tri;

    tri_lists.clear();
	for ( unsigned int i = 0; i < tris.size(); ++i ) {
        tri.clear();

        tri.push_back( tris[i].get_n1() );
        tri.push_back( tris[i].get_n2() );
        tri.push_back( tris[i].get_n3() );

        tri_lists.push_back( tri );
	}

    return tri_lists;
}

int TGGenOutput::build_tris( TGConstruct& c ) {
//  TGTriNodes trinodes = c.get_tri_nodes();
    TGNodes* nodes = c.get_nodes();

    string tile_id = c.get_bucket().gen_index_str();

    // copy the geodetic node list into this class
    geod_nodes = nodes->get_geod_nodes();

    // copy the triangle list into this class
    tri_elements = c.get_tri_elements();

    // build the trifan list
    cout << "total triangles = " << tri_elements.size() << endl;

    // Sort the triangles by area type
    // TODO: Need to sort on Material type - going to make the attribute an index
    //       into an array of texparams / material names
    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        triele_list area_tris;
        area_tris.erase( area_tris.begin(), area_tris.end() );

        const_triele_list_iterator t_current = tri_elements.begin();
        const_triele_list_iterator t_last = tri_elements.end();

        for ( ; t_current != t_last; ++t_current ) {
            if ( t_current->get_attribute() == i ) {
                area_tris.push_back( *t_current );
            }
        }

        if ( area_tris.size() > 0 ) {
            cout << "generating tris for area = " << i << endl;
            primitives[i] = tgGenTris( area_tris );
        }
    }

    // build the texture coordinate list and make a parallel structure
    // to the fan list for pointers into the texture list
    cout << "calculating texture coordinates" << endl;
    cout << "c.get_useUKGrid() = " << c.get_useUKGrid() << endl;
    tex_coords.clear();
    std::vector < SGGeod > convGeodNodes;
    for ( unsigned int k = 0; k < geod_nodes.size(); k++ ) {
        Point3D node = geod_nodes[k];
        convGeodNodes.push_back( SGGeod::fromDegM( node.x(), node.y(), node.z() ) );
    }

    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
        // cout << " area = " << i << endl;
        for ( unsigned int j = 0; j < primitives[i].size(); ++j ) {
            SG_LOG( SG_CLIPPER, SG_INFO, tile_id << ": Texturing " << get_area_name( (AreaType)i ) << " primitive " << j+1 << " of " << primitives[i].size() );

            SGBucket b = c.get_bucket();
            Point3D ourPosition;

            ourPosition.setlon(b.get_chunk_lon());
            ourPosition.setlat(b.get_chunk_lat());

            int_list ti_list;
            ti_list.clear();

            //dcl - here read the flag to check if we are building UK grid
            //If so - check if the bucket is within the UK lat & lon
            if( (c.get_useUKGrid()) && (isInUK(ourPosition)) ) {
                point_list tp_list;
                tp_list = UK_calc_tex_coords( b, geod_nodes, primitives[i][j], 1.0 );
                for ( unsigned int k = 0; k < tp_list.size(); ++k ) {
                    // cout << "  tc = " << tp_list[k] << endl;
                    int index = tex_coords.simple_add( tp_list[k] );
                    ti_list.push_back( index );
                }
            } else {
                std::vector< SGVec2f > tp_list = sgCalcTexCoords( b, convGeodNodes, primitives[i][j] );
                for ( unsigned int k = 0; k < tp_list.size(); ++k ) {
                    SGVec2f tc = tp_list[k];
                    // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
                    int index = tex_coords.simple_add( Point3D( tc.x(), tc.y(), 0 ) );
                    ti_list.push_back( index );
                }
            }

            textures[i].push_back( ti_list );
        }
    }

    // calculate the global bounding sphere
    calc_gbs( c );
    cout << "center = " << gbs_center << " radius = " << gbs_radius << endl;

    return 1;
}

// calculate the bounding sphere for a list of triangle faces
void TGGenOutput::calc_group_bounding_sphere( TGConstruct& c, 
					      const opt_list& primitives, 
					      Point3D *center, double *radius )
{
    cout << "calculate group bounding sphere for " << primitives.size() << " primitives." << endl;

    point_list wgs84_nodes = c.get_wgs84_nodes();

    // generate a list of unique points from the triangle list
    TGTriNodes nodes;

    const_opt_list_iterator f_current = primitives.begin();
    const_opt_list_iterator f_last = primitives.end();
    for ( ; f_current != f_last; ++f_current ) {
	const_int_list_iterator i_current = f_current->begin();
	const_int_list_iterator i_last = f_current->end();
	for ( ; i_current != i_last; ++i_current ) {
	    Point3D p1 = wgs84_nodes[ *i_current ];
	    nodes.unique_add(p1);
	}
    }

    // find average of point list
    *center = Point3D( 0.0 );
    point_list points = nodes.get_node_list();
    // cout << "found " << points.size() << " unique nodes" << endl;
    point_list_iterator p_current = points.begin();
    point_list_iterator p_last = points.end();
    for ( ; p_current != p_last; ++p_current ) {
	*center += *p_current;
    }
    *center /= points.size();

    // find max radius
    double dist_squared;
    double max_squared = 0;

    p_current = points.begin();
    p_last = points.end();
    for ( ; p_current != p_last; ++p_current ) {
	dist_squared = (*center).distance3Dsquared(*p_current);
	if ( dist_squared > max_squared ) {
	    max_squared = dist_squared;
	}
    }

    *radius = sqrt(max_squared);
}


// calculate the bounding sphere for the specified triangle face
void TGGenOutput::calc_bounding_sphere( TGConstruct& c, const TGTriEle& t, 
					Point3D *center, double *radius )
{
    point_list wgs84_nodes = c.get_wgs84_nodes();

    *center = Point3D( 0.0 );

    Point3D p1 = wgs84_nodes[ t.get_n1() ];
    Point3D p2 = wgs84_nodes[ t.get_n2() ];
    Point3D p3 = wgs84_nodes[ t.get_n3() ];

    *center = p1 + p2 + p3;
    *center /= 3;

    double dist_squared;
    double max_squared = 0;

    dist_squared = (*center).distance3Dsquared(p1);
    if ( dist_squared > max_squared ) {
	max_squared = dist_squared;
    }

    dist_squared = (*center).distance3Dsquared(p2);
    if ( dist_squared > max_squared ) {
	max_squared = dist_squared;
    }

    dist_squared = (*center).distance3Dsquared(p3);
    if ( dist_squared > max_squared ) {
	max_squared = dist_squared;
    }

    *radius = sqrt(max_squared);
}

// write out the fgfs scenery file (using fans)
int TGGenOutput::write_fans( TGConstruct &c ) {
    // Assemble all the data into the final format

    SGBucket b = c.get_bucket();
    string base = c.get_output_base();
    string name = b.gen_index_str();
    name += ".btg";

    std::vector< SGVec3d > wgs84_nodes;
    for ( unsigned int i = 0; i < c.get_wgs84_nodes().size(); i++ ) {
        Point3D node = c.get_wgs84_nodes()[i];
        wgs84_nodes.push_back( node.toSGVec3d() );
    }
    std::vector< SGVec3f > normals;
    for ( unsigned int i = 0; i < c.get_point_normals().size(); i++ ) {
        Point3D node = c.get_point_normals()[i];
        normals.push_back( node.toSGVec3f() );
    }
    cout << "dumping normals = " << normals.size() << endl;
    /* for ( i = 0; i < (int)normals.size(); ++i ) {
	Point3D p = normals[i];
	printf("vn %.5f %.5f %.5f\n", p.x(), p.y(), p.z());
    } */
    std::vector< SGVec2f > texcoords;
    for ( unsigned int i = 0; i < tex_coords.get_node_list().size(); i++ ) {
        Point3D node = tex_coords.get_node_list()[i];
        texcoords.push_back( node.toSGVec2f() );
    }

    // allocate and initialize triangle group structures
    group_list tris_v;   group_list tris_tc;   string_list tri_materials;
    tris_v.clear();      tris_tc.clear();      tri_materials.clear();

    group_list strips_v; group_list strips_tc; string_list strip_materials;
    strips_v.clear();    strips_tc.clear();    strip_materials.clear();

    group_list fans_v;   group_list fans_tc;   string_list fan_materials;
    fans_v.clear();      fans_tc.clear();      fan_materials.clear();

    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
	if ( primitives[i].size() > 0 ) {
	    cout << "creating " << primitives[i].size() << " fans of type " << i << endl;
	    string attr_name = get_area_name( (AreaType)i );

	    int_list vs, tcs;
	    for ( unsigned int j = 0; j < primitives[i].size(); ++j ) {
		vs.clear(); tcs.clear();
		for ( unsigned int k = 0; k < primitives[i][j].size(); ++k ) {
		    vs.push_back( primitives[i][j][k] );
		    tcs.push_back( textures[i][j][k] );
		}
		fans_v.push_back( vs );
		fans_tc.push_back( tcs );
		fan_materials.push_back( attr_name );
	    }
	}
    }

    SGBinObject obj;

    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals );
    obj.set_texcoords( texcoords );
    obj.set_tris_v( tris_v );
    obj.set_tris_tc( tris_tc ); 
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_tc( strips_tc ); 
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );

    obj.write_bin( base, name, b );

    return 1;
}

// write out the fgfs scenery file (using tris)
int TGGenOutput::write_tris( TGConstruct &c ) {
    // Assemble all the data into the final format

    SGBucket b = c.get_bucket();
    string base = c.get_output_base();
    string name = b.gen_index_str();
    name += ".btg";

    std::vector< SGVec3d > wgs84_nodes;
    for ( unsigned int i = 0; i < c.get_wgs84_nodes().size(); i++ ) {
        Point3D node = c.get_wgs84_nodes()[i];
        wgs84_nodes.push_back( node.toSGVec3d() );
    }
    std::vector< SGVec3f > normals;
    for ( unsigned int i = 0; i < c.get_point_normals().size(); i++ ) {
        Point3D node = c.get_point_normals()[i];
        normals.push_back( node.toSGVec3f() );
    }
    cout << "dumping normals = " << normals.size() << endl;
    /* for ( i = 0; i < (int)normals.size(); ++i ) {
	Point3D p = normals[i];
	printf("vn %.5f %.5f %.5f\n", p.x(), p.y(), p.z());
    } */
    std::vector< SGVec2f > texcoords;
    for ( unsigned int i = 0; i < tex_coords.get_node_list().size(); i++ ) {
        Point3D node = tex_coords.get_node_list()[i];
        texcoords.push_back( node.toSGVec2f() );
    }

    // allocate and initialize triangle group structures
    group_list tris_v;   group_list tris_tc;   string_list tri_materials;
    tris_v.clear();      tris_tc.clear();      tri_materials.clear();

    group_list strips_v; group_list strips_tc; string_list strip_materials;
    strips_v.clear();    strips_tc.clear();    strip_materials.clear();

    group_list fans_v;   group_list fans_tc;   string_list fan_materials;
    fans_v.clear();      fans_tc.clear();      fan_materials.clear();

    for ( unsigned int i = 0; i < TG_MAX_AREA_TYPES; ++i ) {
	if ( primitives[i].size() > 0 ) {
	    cout << "creating " << primitives[i].size() << " tris of type " << i << endl;
	    string attr_name = get_area_name( (AreaType)i );

	    int_list vs, tcs;
	    for ( unsigned int j = 0; j < primitives[i].size(); ++j ) {
		vs.clear(); tcs.clear();
		for ( unsigned int k = 0; k < primitives[i][j].size(); ++k ) {
		    vs.push_back( primitives[i][j][k] );
		    tcs.push_back( textures[i][j][k] );
		}
		tris_v.push_back( vs );
		tris_tc.push_back( tcs );
		tri_materials.push_back( attr_name );
	    }
	}
    }

    SGBinObject obj;

    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals );
    obj.set_texcoords( texcoords );
    obj.set_tris_v( tris_v );
    obj.set_tris_tc( tris_tc ); 
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_tc( strips_tc ); 
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );

    obj.write_bin( base, name, b );

    return 1;
}
