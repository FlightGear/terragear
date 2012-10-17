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
#include <simgear/io/lowlevel.hxx>

#include <Geometry/poly_support.hxx>

#include "tgconstruct.hxx"

using std::string;

void TGConstruct::SaveSharedEdgeData( int stage )
{
    string filepath;

    switch( stage ) {
        case 1:
        {
            point_list north, south, east, west;
            int nCount;

            nodes.get_geod_edge( bucket, north, south, east, west );

            filepath = share_base + "/stage1/" + bucket.gen_base_path() + "/" + bucket.gen_index_str() + "_edges";
            SGPath file(filepath);
            file.create_dir( 0755 );

            gzFile fp;
            if ( (fp = gzopen( filepath.c_str(), "wb9" )) == NULL ) {
                SG_LOG( SG_GENERAL, SG_INFO, "ERROR: opening " << file.str() << " for writing!" );
                return;
            }

            sgClearWriteError();

            // north
            nCount = north.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWritePoint3D( fp, north[i] );
            }

            // south
            nCount = south.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWritePoint3D( fp, south[i] );
            }

            // east
            nCount = east.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWritePoint3D( fp, east[i] );
            }

            // west
            nCount = west.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWritePoint3D( fp, west[i] );
            }

            gzclose(fp);
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

void TGConstruct::WriteNeighborFaces( gzFile& fp, Point3D pt )
{
    // find all neighboors of this point
    int    n    = nodes.find( pt );
    TGNode node = nodes.get_node( n );
    TGFaceList faces  = node.GetFaces();

    // write the number of neighboor faces
    sgWriteInt( fp, faces.size() );

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

            sgWriteDouble( fp, face_area );
            sgWritePoint3D( fp, face_normal );
        }
    }
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

void TGConstruct::ReadNeighborFaces( gzFile& fp )
{
    int count;

    // read the count
    sgReadInt( fp, &count );

    for (int i=0; i<count; i++) {
        TGNeighborFaces* pFaces;
        Point3D          node;
        int              num_faces;

        sgReadPoint3D( fp, node );

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

        sgReadInt( fp, &num_faces );
        for (int j=0; j<num_faces; j++)
        {
            double  area;
            Point3D normal;

            sgReadDouble( fp, &area );
            pFaces->face_areas.push_back( area );

            sgReadPoint3D( fp, normal );
            pFaces->face_normals.push_back( normal );
        }
    }
}

void TGConstruct::SaveSharedEdgeDataStage2( void )
{
    string dir;
    string file;
    point_list north, south, east, west;
    int nCount;

    nodes.get_geod_edge( bucket, north, south, east, west );

    dir  = share_base + "/stage2/" + bucket.gen_base_path();

    SGPath sgp( dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );


    // north edge
    file = dir + "/" + bucket.gen_index_str() + "_north_edge";
    gzFile fp;
    if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
        return;
    }
    sgClearWriteError();

    nCount = north.size();
    sgWriteInt( fp, nCount );
    for (int i=0; i<nCount; i++) {
        // write the 3d point
        sgWritePoint3D( fp, north[i] );
        WriteNeighborFaces( fp, north[i] );
    }
    gzclose(fp);

    // south edge
    file = dir + "/" + bucket.gen_index_str() + "_south_edge";
    if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
        return;
    }
    sgClearWriteError();

    nCount = south.size();
    sgWriteInt( fp, nCount );
    for (int i=0; i<nCount; i++) {
        sgWritePoint3D( fp, south[i] );
        WriteNeighborFaces( fp, south[i] );
    }
    gzclose(fp);

    // east edge
    file = dir + "/" + bucket.gen_index_str() + "_east_edge";
    if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
        return;
    }
    sgClearWriteError();

    nCount = east.size();
    sgWriteInt( fp, nCount );
    for (int i=0; i<nCount; i++) {
        sgWritePoint3D( fp, east[i] );
        WriteNeighborFaces( fp, east[i] );
    }
    gzclose(fp);

    // west egde
    file = dir + "/" + bucket.gen_index_str() + "_west_edge";
    if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
        SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
        return;
    }
    sgClearWriteError();

    nCount = west.size();
    sgWriteInt( fp, nCount );
    for (int i=0; i<nCount; i++) {
        sgWritePoint3D( fp, west[i] );
        WriteNeighborFaces( fp, west[i] );
    }
    gzclose(fp);
}

void TGConstruct::LoadSharedEdgeDataStage2( void )
{
    string dir;
    string file;
    double     clon = bucket.get_center_lon();
    double     clat = bucket.get_center_lat();
    gzFile     fp;
    SGBucket   b;

    // Read Northern tile and add its southern node faces
    b    = sgBucketOffset(clon, clat, 0, 1);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_south_edge";
    fp = gzopen( file.c_str(), "rb" );
    if (fp) {
        sgClearReadError();
        ReadNeighborFaces( fp );
        gzclose( fp );
    }

    // Read Southern tile and add its northern node faces
    b    = sgBucketOffset(clon, clat, 0, -1);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_north_edge";
    fp = gzopen( file.c_str(), "rb" );
    if (fp) {
        sgClearReadError();
        ReadNeighborFaces( fp );
        gzclose( fp );
    }

    // Read Eastern tile and add its western node faces
    b    = sgBucketOffset(clon, clat, 1, 0);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_west_edge";
    fp = gzopen( file.c_str(), "rb" );
    if (fp) {
        sgClearReadError();
        ReadNeighborFaces( fp );
        gzclose( fp );
    }

    // Read Western tile and add its eastern node faces
    b    = sgBucketOffset(clon, clat, -1, 0);
    dir  = share_base + "/stage2/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_east_edge";
    fp = gzopen( file.c_str(), "rb" );
    if (fp) {
        sgClearReadError();
        ReadNeighborFaces( fp );
        gzclose( fp );
    }
}


void TGConstruct::SaveToIntermediateFiles( int stage )
{
    string dir;
    string file;
    gzFile fp;

    switch( stage ) {
        case 1:     // Save the clipped polys and node list
        {
            /* Only create the file this isn't an ocean tile */
            if ( !IsOceanTile() ) {
                dir  = share_base + "/stage1/" + bucket.gen_base_path();

                SGPath sgp( dir );
                sgp.append( "dummy" );
                sgp.create_dir( 0755 );

                file = dir + "/" + bucket.gen_index_str() + "_clipped_polys";
                if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
                    return;
                }
                sgClearWriteError();
                polys_clipped.SaveToGzFile( fp );
                gzclose( fp );

                file = dir + "/" + bucket.gen_index_str() + "_nodes";
                if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
                    return;
                }
                sgClearWriteError();
                nodes.SaveToGzFile( fp );
                gzclose( fp );
            }

            break;
        }

        case 2:     // Save the clipped polys and node list
        {
            if ( !IsOceanTile() ) {
                dir  = share_base + "/stage2/" + bucket.gen_base_path();

                SGPath sgp( dir );
                sgp.append( "dummy" );
                sgp.create_dir( 0755 );

                file = dir + "/" + bucket.gen_index_str() + "_clipped_polys";
                if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
                    return;
                }
                sgClearWriteError();
                polys_clipped.SaveToGzFile( fp );
                gzclose( fp );

                file = dir + "/" + bucket.gen_index_str() + "_nodes";
                if ( (fp = gzopen( file.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file.c_str() << " for writing!" );
                    return;
                }
                sgClearWriteError();
                nodes.SaveToGzFile( fp );
                gzclose( fp );
            }
            break;
        }
    }
}

void TGConstruct::LoadNeighboorEdgeDataStage1( SGBucket& b, point_list& north, point_list& south, point_list& east, point_list& west )
{
    string dir;
    string file;
    gzFile fp;
    Point3D pt;
    int nCount;

    dir  = share_base + "/stage1/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_edges";
    fp = gzopen( file.c_str(), "rb" );

    north.clear();
    south.clear();
    east.clear();
    west.clear();

    if (fp) {
        // North
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " north boundary");
        for (int i=0; i<nCount; i++) {
            sgReadPoint3D( fp, pt );
            north.push_back(pt);
        }

        // South
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " south boundary");
        for (int i=0; i<nCount; i++) {
            sgReadPoint3D( fp, pt );
            south.push_back(pt);
        }

        // East
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " east boundary");
        for (int i=0; i<nCount; i++) {
            sgReadPoint3D( fp, pt );
            east.push_back(pt);
        }

        // West
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_INFO, "loading " << nCount << "Points on " << b.gen_index_str() << " west boundary");
        for (int i=0; i<nCount; i++) {
            sgReadPoint3D( fp, pt );
            west.push_back(pt);
        }

        gzclose( fp );
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
    gzFile fp;
    bool   read_ok = false;

    switch( stage ) {
        case 1:     // Load the clipped polys and node list
        {
            dir  = share_base + "/stage1/" + bucket.gen_base_path();
            file = dir        + "/"        + bucket.gen_index_str() + "_clipped_polys";
            fp = gzopen( file.c_str(), "rb" );

            if ( fp ) {
                polys_clipped.LoadFromGzFile( fp );
                gzclose( fp );

                file = dir + "/" + bucket.gen_index_str() + "_nodes";
                fp = gzopen( file.c_str(), "rb" );

                if ( fp ) {
                    nodes.LoadFromGzFile( fp );
                    gzclose( fp );

                    read_ok = true;
                }
            }

            break;
        }

        case 2:     // Load the clipped polys and node list
        {
            dir  = share_base + "/stage2/" + bucket.gen_base_path();
            file = dir        + "/"        + bucket.gen_index_str() + "_clipped_polys";
            fp = gzopen( file.c_str(), "rb" );

            if ( fp ) {
                polys_clipped.LoadFromGzFile( fp );
                gzclose( fp );

                file = dir + "/" + bucket.gen_index_str() + "_nodes";
                fp = gzopen( file.c_str(), "rb" );

                if ( fp ) {
                    nodes.LoadFromGzFile( fp );
                    gzclose( fp );

                    read_ok = true;
                }
            }

            break;
        }
    }

    if ( !read_ok ) {
        SetOceanTile();
    }
}
