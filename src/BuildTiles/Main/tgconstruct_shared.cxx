// construct_intermediate.cxx -- Handle all intermediate and shared data files
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

#include <iomanip>

#include <simgear/misc/sg_dir.hxx>
#include <simgear/debug/logstream.hxx>

#include <Geometry/poly_support.hxx>

#include "tgconstruct.hxx"

using std::string;

void TGConstruct::SaveSharedEdgeData( int stage )
{
    string dir;
    string file;

    switch( stage ) {
        case 1:
        {
            point_list north, south, east, west;
            int nCount;

            nodes.get_geod_edge( bucket, north, south, east, west );

            dir  = share_base + "/stage1/" + bucket.gen_base_path();

            SGPath sgp( dir );
            sgp.append( "dummy" );
            sgp.create_dir( 0755 );

            file = dir + "/" + bucket.gen_index_str() + "_edges";
            std::ofstream ofs_e( file.c_str() );

            // first, set the precision
            ofs_e << std::setprecision(12);
            ofs_e << std::fixed;

            // north
            nCount = north.size();
            ofs_e << nCount << "\n";
            for (int i=0; i<nCount; i++) {
                ofs_e << north[i];
            }

            // south
            nCount = south.size();
            ofs_e << nCount << "\n";
            for (int i=0; i<nCount; i++) {
                ofs_e << south[i];
            }

            // east
            nCount = east.size();
            ofs_e << nCount << "\n";
            for (int i=0; i<nCount; i++) {
                ofs_e << east[i];
            }

            // west
            nCount = west.size();
            ofs_e << nCount << "\n";
            for (int i=0; i<nCount; i++) {
                ofs_e << west[i];
            }

            ofs_e.close();
        }
        break;

        case 2:
        {
            // for stage 2, we need enough info on a node to average out elevation and
            // generate a correct normal
            // we will use the Point3D, as stage1 above, then a point list per node
            // to store the neighboors.
            //
            // NOTE: Some neighboors (likely 2) are on the shared border.
            // Before calculating face normals for the node, all elevation data for
            // neighboors needs to be completed.  So after all border nodes' elevations
            // are updated, we'll need to traverse all of these point lists, and update
            // any border nodes elevation as well
            SaveSharedEdgeDataStage2();
        }
        break;
    }
}

void TGConstruct::WriteNeighborFaces( std::ofstream& ofs_e, Point3D pt )
{
    // find all neighboors of this point
    int    n    = nodes.find( pt );
    TGNode node = nodes.get_node( n );
    TGFaceList faces  = node.GetFaces();

    // write the number of neighboor faces
    ofs_e << faces.size() << "\n";

    // write out each face normal and size
    for (unsigned int j=0; j<faces.size(); j++) {
        // for each connected face, get the nodes
        unsigned int at      = faces[j].area;
        unsigned int shape   = faces[j].shape;
        unsigned int segment = faces[j].seg;
        unsigned int tri     = faces[j].tri;

        int_list face_nodes = polys_clipped.get_tri_idxs( at, shape, segment ).get_contour( tri ) ;
        {
            Point3D p1 = nodes.get_node( face_nodes[0] ).GetPosition();
            Point3D p2 = nodes.get_node( face_nodes[1] ).GetPosition();
            Point3D p3 = nodes.get_node( face_nodes[2] ).GetPosition();

            Point3D wgs_p1 = nodes.get_node( face_nodes[0] ).GetWgs84AsPoint3D();
            Point3D wgs_p2 = nodes.get_node( face_nodes[1] ).GetWgs84AsPoint3D();
            Point3D wgs_p3 = nodes.get_node( face_nodes[2] ).GetWgs84AsPoint3D();

            double  face_area   = triangle_area( p1, p2, p3 );
            Point3D face_normal = Point3D::fromSGVec3( calc_normal( face_area, wgs_p1, wgs_p2, wgs_p3 ) );

            ofs_e << face_area << " ";
            ofs_e << face_normal;
        }
    }
    ofs_e << "\n";
}

TGNeighborFaces* TGConstruct::FindNeighborFaces( Point3D node )
{
    TGNeighborFaces* faces = NULL;

    for (unsigned int i=0; i<neighbor_faces.size(); i++) {
        if ( node == neighbor_faces[i].node ) {
            faces = &neighbor_faces[i];
            break;
        }
    }

    return faces;
}

TGNeighborFaces* TGConstruct::AddNeighborFaces( Point3D node )
{
    TGNeighborFaces faces;
    faces.node = node;

    neighbor_faces.push_back( faces );

    return &neighbor_faces[neighbor_faces.size()-1];
}

void TGConstruct::ReadNeighborFaces( std::ifstream& in )
{
    int count;

    // read the count
    in >> count;

    for (int i=0; i<count; i++) {
        TGNeighborFaces* pFaces;
        Point3D          node;
        int              num_faces;

        in >> node;

        // look to see if we already have this node
        // If we do, (it's a corner) add more faces to it.
        // otherwise, initialize it with our elevation data
        pFaces = FindNeighborFaces( node );
        if ( !pFaces ) {
            pFaces = AddNeighborFaces( node );

            // new face - let's add our elevation first
            int idx = nodes.find( node );
            if (idx >= 0) {
                TGNode local = nodes.get_node( idx );
                pFaces->elevations.push_back( local.GetPosition().z() );
            }
        }

        // remember all of the elevation data for the node, so we can average
        pFaces->elevations.push_back( node.z() );

        in >> num_faces;
        for (int j=0; j<num_faces; j++)
        {
            double  area;
            Point3D normal;

            in >> area;
            pFaces->face_areas.push_back( area );

            in >> normal;
            pFaces->face_normals.push_back( normal );
        }
    }
}

void TGConstruct::SaveSharedEdgeDataStage2( void )
{
    string dir;
    string file;
    point_list north, south, east, west;
    std::ofstream ofs_e;
    int nCount;

    nodes.get_geod_edge( bucket, north, south, east, west );

    dir  = share_base + "/stage2/" + bucket.gen_base_path();

    SGPath sgp( dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );


    // north edge
    file = dir + "/" + bucket.gen_index_str() + "_north_edge";
    ofs_e.open( file.c_str() );
    ofs_e << std::setprecision(12);
    ofs_e << std::fixed;

    nCount = north.size();
    ofs_e << nCount << "\n";
    for (int i=0; i<nCount; i++) {
        // write the 3d point
        ofs_e << north[i];
        WriteNeighborFaces( ofs_e, north[i] );
    }
    ofs_e.close();

    // south edge
    file = dir + "/" + bucket.gen_index_str() + "_south_edge";
    ofs_e.open( file.c_str() );
    ofs_e << std::setprecision(12);
    ofs_e << std::fixed;

    nCount = south.size();
    ofs_e << nCount << "\n";
    for (int i=0; i<nCount; i++) {
        ofs_e << south[i];
        WriteNeighborFaces( ofs_e, south[i] );
    }
    ofs_e.close();

    // east edge
    file = dir + "/" + bucket.gen_index_str() + "_east_edge";
    ofs_e.open( file.c_str() );
    ofs_e << std::setprecision(12);
    ofs_e << std::fixed;

    nCount = east.size();
    ofs_e << nCount << "\n";
    for (int i=0; i<nCount; i++) {
        ofs_e << east[i];
        WriteNeighborFaces( ofs_e, east[i] );
    }
    ofs_e.close();

    // west egde
    file = dir + "/" + bucket.gen_index_str() + "_west_edge";
    ofs_e.open( file.c_str() );
    ofs_e << std::setprecision(12);
    ofs_e << std::fixed;

    nCount = west.size();
    ofs_e << nCount << "\n";
    for (int i=0; i<nCount; i++) {
        ofs_e << west[i];
        WriteNeighborFaces( ofs_e, west[i] );
    }
    ofs_e.close();
}

void TGConstruct::LoadSharedEdgeDataStage2( void )
{
    string dir;
    string file;
    std::ifstream ifs_edge;
    double     clon = bucket.get_center_lon();
    double     clat = bucket.get_center_lat();
    SGBucket   b;

    // Read Northern tile and add its southern node faces
    b    = sgBucketOffset(clon, clat, 0, 1);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_south_edge";
    ifs_edge.open( file.c_str() );
    if ( ifs_edge.is_open() ) {
        ReadNeighborFaces( ifs_edge );
    }
    ifs_edge.close();

    // Read Southern tile and add its northern node faces
    b    = sgBucketOffset(clon, clat, 0, -1);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_north_edge";
    ifs_edge.open( file.c_str() );
    if ( ifs_edge.is_open() ) {
        ReadNeighborFaces( ifs_edge );
    }
    ifs_edge.close();

    // Read Eastern tile and add its western node faces
    b    = sgBucketOffset(clon, clat, 1, 0);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_west_edge";
    ifs_edge.open( file.c_str() );
    if ( ifs_edge.is_open() ) {
        ReadNeighborFaces( ifs_edge );
    }
    ifs_edge.close();

    // Read Western tile and add its eastern node faces
    b    = sgBucketOffset(clon, clat, -1, 0);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_east_edge";
    ifs_edge.open( file.c_str() );
    if ( ifs_edge.is_open() ) {
        ReadNeighborFaces( ifs_edge );
    }
    ifs_edge.close();
}


void TGConstruct::SaveToIntermediateFiles( int stage )
{
    string dir;
    string file;

    switch( stage ) {
        case 1:     // Save the clipped polys and node list
        {
            dir  = share_base + "/stage1/" + bucket.gen_base_path();

            SGPath sgp( dir );
            sgp.append( "dummy" );
            sgp.create_dir( 0755 );

            file = dir + "/" + bucket.gen_index_str() + "_clipped_polys";
            std::ofstream ofs_cp( file.c_str() );

            // first, set the precision
            ofs_cp << std::setprecision(15);
            ofs_cp << std::fixed;
            ofs_cp << polys_clipped;
            ofs_cp.close();


            file = dir + "/" + bucket.gen_index_str() + "_nodes";
            std::ofstream ofs_n( file.c_str() );

            // first, set the precision
            ofs_n << std::setprecision(15);
            ofs_n << std::fixed;
            ofs_n << nodes;
            ofs_n.close();
            break;
        }

        case 2:     // Save the clipped polys and node list
        {
            dir  = share_base + "/stage2/" + bucket.gen_base_path();

            SGPath sgp( dir );
            sgp.append( "dummy" );
            sgp.create_dir( 0755 );

            file = dir + "/" + bucket.gen_index_str() + "_clipped_polys";
            std::ofstream ofs_cp( file.c_str() );

            // first, set the precision
            ofs_cp << std::setprecision(15);
            ofs_cp << std::fixed;
            ofs_cp << polys_clipped;
            ofs_cp.close();

            file = dir + "/" + bucket.gen_index_str() + "_nodes";
            std::ofstream ofs_n( file.c_str() );

            // first, set the precision
            ofs_n << std::setprecision(15);
            ofs_n << std::fixed;
            ofs_n << nodes;
            ofs_n.close();
            break;
        }
    }
}

void TGConstruct::LoadNeighboorEdgeDataStage1( SGBucket& b, point_list& north, point_list& south, point_list& east, point_list& west )
{
    string dir;
    string file;
    Point3D pt;
    int nCount;

    dir  = share_base + "/stage1/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_edges";
    std::ifstream ifs_edges( file.c_str() );

    north.clear();
    south.clear();
    east.clear();
    west.clear();

    if ( ifs_edges.is_open() ) {
        // North
        ifs_edges >> nCount;
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " north boundary");
        for (int i=0; i<nCount; i++) {
            ifs_edges >> pt;
            north.push_back(pt);
        }

        // South
        ifs_edges >> nCount;
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " south boundary");
        for (int i=0; i<nCount; i++) {
            ifs_edges >> pt;
            south.push_back(pt);
        }

        // East
        ifs_edges >> nCount;
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " east boundary");
        for (int i=0; i<nCount; i++) {
            ifs_edges >> pt;
            east.push_back(pt);
        }

        // West
        ifs_edges >> nCount;
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " west boundary");
        for (int i=0; i<nCount; i++) {
            ifs_edges >> pt;
            west.push_back(pt);
        }

        ifs_edges.close();
    }
}

void TGConstruct::LoadSharedEdgeData( int stage )
{
    switch( stage ) {
        case 1:
        {
            // we need to read just 4 buckets for stage 1 - 1 for each edge
            point_list north, south, east, west;
            SGBucket   nb, sb, eb, wb;
            double     clon = bucket.get_center_lon();
            double     clat = bucket.get_center_lat();

            // Read North tile and add its southern nodes
            nb = sgBucketOffset(clon, clat, 0, 1);
            LoadNeighboorEdgeDataStage1( nb, north, south, east, west );
            // Add southern nodes from northern tile
            for (unsigned int i=0; i<south.size(); i++) {
                nodes.unique_add( south[i] );
            }

            // Read South Tile and add its northern nodes
            sb = sgBucketOffset(clon, clat, 0, -1);
            LoadNeighboorEdgeDataStage1( sb, north, south, east, west );
            for (unsigned  int i=0; i<north.size(); i++) {
                nodes.unique_add( north[i] );
            }

            // Read East Tile and add its western nodes
            eb = sgBucketOffset(clon, clat, 1, 0);
            LoadNeighboorEdgeDataStage1( eb, north, south, east, west );
            for (unsigned  int i=0; i<west.size(); i++) {
                nodes.unique_add( west[i] );
            }

            // Read West Tile and add its eastern nodes
            wb = sgBucketOffset(clon, clat, -1, 0);
            LoadNeighboorEdgeDataStage1( wb, north, south, east, west );
            for (unsigned  int i=0; i<east.size(); i++) {
                nodes.unique_add( east[i] );
            }
        }
        break;
    }
}

void TGConstruct::LoadFromIntermediateFiles( int stage )
{
    string dir;
    string file;

    switch( stage ) {
        case 1:     // Load the clipped polys and node list
        {
            dir  = share_base + "/stage1/" + bucket.gen_base_path();
            file = dir        + "/"        + bucket.gen_index_str() + "_clipped_polys";

            std::ifstream ifs_cp( file.c_str() );
            ifs_cp >> polys_clipped;
            ifs_cp.close();

            file = dir + "/" + bucket.gen_index_str() + "_nodes";

            std::ifstream ifs_n( file.c_str() );
            ifs_n >> nodes;
            ifs_n.close();
            break;
        }

        case 2:     // Load the clipped polys and node list
        {
            dir  = share_base + "/stage2/" + bucket.gen_base_path();
            file = dir        + "/"        + bucket.gen_index_str() + "_clipped_polys";

            std::ifstream ifs_cp( file.c_str() );
            ifs_cp >> polys_clipped;
            ifs_cp.close();

            file = dir + "/" + bucket.gen_index_str() + "_nodes";

            std::ifstream ifs_n( file.c_str() );
            ifs_n >> nodes;
            ifs_n.close();
            break;
        }
    }
}
