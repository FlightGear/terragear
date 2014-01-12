// tgconstruct_output.cxx --Handle writing out the btg and stg files
//
// Written by Curtis Olson, started May 1999.
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
// $Id: construct.cxx,v 1.4 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/math/SGGeometry.hxx>
#include <simgear/misc/sg_dir.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>

#include <terragear/tg_unique_vec3f.hxx>
#include <terragear/tg_unique_vec2f.hxx>

#include "tgconstruct.hxx"

using std::string;

// collect custom objects and move to scenery area
void TGConstruct::AddCustomObjects( void ) {
    // Create/open the output .stg file for writing
    SGPath dest_d(output_base.c_str());
    dest_d.append(bucket.gen_base_path().c_str());
    string dest_dir = dest_d.str_native();
    SGPath dest_i(dest_d);
    dest_i.append(bucket.gen_index_str());
    dest_i.concat(".stg");
    string dest_ind = dest_i.str_native();

    FILE *fp;
    
    lock->lock();
    if ( (fp = fopen( dest_ind.c_str(), "w" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "ERROR: opening " << dest_ind << " for writing!" );
        exit(-1);
    }

    // Start with the default custom object which is the base terrain
    fprintf(fp, "OBJECT_BASE %s.btg\n", bucket.gen_index_str().c_str());

    char line[2048];             // big enough?
    char token[256];
    char name[256];

    for ( int i = 0; i < (int)load_dirs.size(); ++i ) {
        SGPath base(work_base.c_str());
        base.append(load_dirs[i]);
        base.append( bucket.gen_base_path() );
        SGPath index(base);
        index.append( bucket.gen_index_str() );
        index.concat(".ind");
        string index_file = index.str_native();

        sg_gzifstream in( index_file );

        if ( ! in.is_open() ) {
            //No custom objects
        } else {
            while ( ! in.eof() ) {
                SG_LOG( SG_GENERAL, SG_DEBUG, "Collecting custom objects from " << index_file );
                in.getline(line, 2048);
                SG_LOG( SG_GENERAL, SG_DEBUG, "line = " << line );

                int result = sscanf( line, "%s %s", token, name );
                SG_LOG( SG_GENERAL, SG_DEBUG, "scanf scanned " << result << " tokens" );

                if ( result > 0 ) {
                    SG_LOG( SG_GENERAL, SG_DEBUG, "token = " << token << " name = " << name );

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
                        SG_LOG( SG_GENERAL, SG_DEBUG, "running " << command );
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
    
    lock->unlock();
}

void TGConstruct::WriteBtgFile( void )
{
    UniqueSGVec3fSet normals;
    UniqueSGVec2fSet texcoords;

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

    for (unsigned int area = 0; area < area_defs.size(); area++) {
        // only tesselate non holes
        if ( !area_defs.is_hole_area(area) ) {
            for (unsigned int p = 0; p < polys_clipped.area_size(area); p++ ) {
                SG_LOG( SG_CLIPPER, SG_DEBUG, "Ouput nodes for " << area_defs.get_area_name(area) << ":" << p+1 << " of " << polys_clipped.area_size(area) );

                tgPolygon   poly      = polys_clipped.get_poly(area, p);
                string      material  = poly.GetMaterial();

                for (unsigned int k = 0; k < poly.Triangles(); ++k) {
                    tri_v.clear();
                    tri_n.clear();
                    tri_tc.clear();
                    for (int l = 0; l < 3; ++l) {
                        index = poly.GetTriIdx( k, l );
                        tri_v.push_back( index );

                        // add the node's normal
                        index = normals.add( nodes.GetNormal( index ) );
                        tri_n.push_back( index );

                        index = texcoords.add( poly.GetTriTexCoord( k, l ) );
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

    std::vector< SGVec3d > wgs84_nodes;
    nodes.get_wgs84_nodes( wgs84_nodes );
    SGVec3d gbs_center = SGVec3d::fromGeod( bucket.get_center() );
    double dist_squared, radius_squared = 0;
    for (int i = 0; i < (int)wgs84_nodes.size(); ++i)
    {
        dist_squared = distSqr(gbs_center, wgs84_nodes[i]);
        if ( dist_squared > radius_squared ) {
            radius_squared = dist_squared;
        }
    }
    double gbs_radius = sqrt(radius_squared);

    SG_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
    SG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_n; fans_n.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    string base = output_base;
    string binname = bucket.gen_index_str();
    binname += ".btg";
    string txtname = bucket.gen_index_str();
    txtname += ".txt";

    SGBinObject obj;

    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals.get_list() );
    obj.set_texcoords( texcoords.get_list() );
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
    
    lock->lock();
    result = obj.write_bin( base, binname, bucket );
    lock->unlock();
    
    if ( !result )
    {
        throw sg_exception("error writing file. :-(");
    }
    if (debug_all || debug_shapes.size())
    {
        lock->lock();
        result = obj.write_ascii( base, txtname, bucket );
        lock->unlock();
        
        if ( !result )
        {
            throw sg_exception("error writing file. :-(");
        }
    }
}
