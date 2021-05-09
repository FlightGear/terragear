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
#include <simgear/io/sg_binobj.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/io/lowlevel.hxx>

#include "tgconstruct.hxx"

using std::string;

void TGConstruct::CreateMatchedEdgeFiles( std::vector<SGBucket>& bucketList )
{
    // todo - add to work queue
    for ( unsigned int i = 0; i < bucketList.size(); ++i ) {
        SGBucket b = bucketList[i];
        nodes.clear();
        
        // load the .btg from the match directory
        SGPath file = match_base + "/" + b.gen_base_path() + "/" + b.gen_index_str() + ".btg.gz";
        SGBinObject obj;
        
        obj.read_bin( file.str() );
        std::vector<SGVec3d> wgs84_nodes(obj.get_wgs84_nodes() );
        
        string filepath;
        std::vector<SGGeod> north, south, east, west;
        int nCount;

        // read in all of the .btg nodes
        for ( unsigned int j = 0; j < wgs84_nodes.size(); ++j ) {
            SGGeod pos = SGGeod::fromCart( wgs84_nodes[j] + obj.get_gbs_center() );
            nodes.unique_add( pos );
        }

        SG_LOG( SG_GENERAL, SG_DEBUG, "Read " << nodes.size() << " from file " << file.str()  );
        
        nodes.init_spacial_query();
        nodes.get_geod_edge( b, north, south, east, west );

        filepath = share_base + "/match1/" + b.gen_base_path() + "/" + b.gen_index_str() + "_edges";
        SGPath file2(filepath);

        lock->lock();
            
        file2.create_dir( 0755 );

        gzFile fp;
        if ( (fp = gzopen( filepath.c_str(), "wb9" )) == NULL ) {
            SG_LOG( SG_GENERAL, SG_INFO, "ERROR: opening " << file.str() << " for writing!" );
            return;
        }

        // north
        nCount = north.size();
        SG_LOG( SG_GENERAL, SG_DEBUG, "write " << north.size() << " northern nodes to file " << filepath.c_str()  );
        sgWriteInt( fp, nCount );
        for (int j = 0; j < nCount; ++j) {
            sgWriteGeod( fp, north[j] );
        }

        // south
        nCount = south.size();
        SG_LOG( SG_GENERAL, SG_DEBUG, "write " << south.size() << " southern nodes to file " << filepath.c_str()  );
        sgWriteInt( fp, nCount );
        for (int j = 0; j < nCount; ++j) {
            sgWriteGeod( fp, south[j] );
        }

        // east
        nCount = east.size();
        SG_LOG( SG_GENERAL, SG_DEBUG, "write " << east.size() << " eastern nodes to file " << filepath.c_str()  );
        sgWriteInt( fp, nCount );
        for (int j = 0; j < nCount; ++j) {
            sgWriteGeod( fp, east[j] );
        }

        // west
        nCount = west.size();
        SG_LOG( SG_GENERAL, SG_DEBUG, "write " << west.size() << " western nodes to file " << filepath.c_str()  );
        sgWriteInt( fp, nCount );
        for (int j = 0; j < nCount; ++j) {
            sgWriteGeod( fp, west[j] );
        }        

        gzclose(fp);
            
        lock->unlock();
    }
}

void TGConstruct::LoadMatchedEdgeFiles()
{
    // try to load matched edges - on successful load, the edge is marked immutable
    
    // we need to read just 4 buckets for stage 1 - 1 for each edge
    std::vector<SGGeod> north, south, east, west;
    SGBucket   nb, sb, eb, wb;

    // Read Northern tile and add its southern nodes
    nb = bucket.sibling(0, 1);
    LoadNeighboorMatchDataStage1( nb, north, south, east, west );
    if ( !south.empty() ) {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Read " << south.size() << " northern matched nodes " );
        
        // Add southern nodes from northern tile
        for (unsigned int i=0; i<south.size(); i++) {
            nodes.unique_add( south[i] );
            nm_north.push_back( south[i] );
        }
    }

    // Read Southern Tile and add its northern nodes
    sb = bucket.sibling(0, -1);
    LoadNeighboorMatchDataStage1( sb, north, south, east, west );
    if ( !north.empty() ) {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Read " << north.size() << " southern matched nodes " );

        for (unsigned int i=0; i<north.size(); i++) {
            nodes.unique_add( north[i] );
            nm_south.push_back( north[i] );
        }
    }

    // Read Eastern Tile and add its western nodes
    eb = bucket.sibling(1, 0);
    LoadNeighboorMatchDataStage1( eb, north, south, east, west );
    SG_LOG( SG_GENERAL, SG_DEBUG, "Read " << west.size() << " eastern matched nodes " );
    
    if ( !west.empty() ) {
        for (unsigned int i=0; i<west.size(); i++) {
            nodes.unique_add( west[i] );
            nm_east.push_back( west[i] );
        }
    }

    // Read Western Tile and add its eastern nodes
    wb = bucket.sibling(-1, 0);
    SG_LOG( SG_GENERAL, SG_DEBUG, "Read " << east.size() << " western matched nodes " );

    LoadNeighboorMatchDataStage1( wb, north, south, east, west );
    if ( !east.empty() ) {
        for (unsigned int i=0; i<east.size(); i++) {
            nodes.unique_add( east[i] );
            nm_west.push_back( east[i] );
        }
    }
}


void TGConstruct::SaveSharedEdgeData( int stage )
{
    switch( stage ) {
        case 1:
        {
            string filepath;
            std::vector<SGGeod> north, south, east, west;
            int nCount;

            nodes.get_geod_edge( bucket, north, south, east, west );

            filepath = share_base + "/stage1/" + bucket.gen_base_path() + "/" + bucket.gen_index_str() + "_edges";
            SGPath file(filepath);

            lock->lock();
            
            file.create_dir( 0755 );

            gzFile fp;
            if ( (fp = gzopen( filepath.c_str(), "wb9" )) == NULL ) {
                SG_LOG( SG_GENERAL, SG_INFO, "ERROR: opening " << file.str() << " for writing!" );
                return;
            }

            // north
            nCount = north.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWriteGeod( fp, north[i] );
            }

            // south
            nCount = south.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWriteGeod( fp, south[i] );
            }

            // east
            nCount = east.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWriteGeod( fp, east[i] );
            }

            // west
            nCount = west.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWriteGeod( fp, west[i] );
            }

            gzclose(fp);
            
            lock->unlock();
        }
        break;

        case 2:
        {
            // for stage 2, we need enough info on a node to average out elevation and
            // generate a correct normal
            // we will use a geod, as stage1 above, then a geod list per node
            // to store the neighboors.
            //
            // NOTE: Some neighboors (likely 2) are on the shared border.
            // Before calculating face normals for the node, all elevation data for
            // neighboors needs to be completed.  So after all border nodes' elevations
            // are updated, we'll need to traverse all of these point lists, and update
            // any border nodes elevation as well
            string dir;
            string file_north, file_south, file_east, file_west;
            gzFile fp;
            std::vector<SGGeod> north, south, east, west;
            int nCount;

            nodes.get_geod_edge( bucket, north, south, east, west );

            dir  = share_base + "/stage2/" + bucket.gen_base_path();
            file_north = dir + "/" + bucket.gen_index_str() + "_north_edge";
            file_south = dir + "/" + bucket.gen_index_str() + "_south_edge";
            file_east  = dir + "/" + bucket.gen_index_str() + "_east_edge";
            file_west  = dir + "/" + bucket.gen_index_str() + "_west_edge";
            
            SGPath sgp( dir );
            sgp.append( "dummy" );
            
            lock->lock();
            sgp.create_dir( 0755 );

            // north edge
            if ( (fp = gzopen( file_north.c_str(), "wb9" )) == NULL ) {
                SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_north.c_str() << " for writing!" );
                return;
            }

            nCount = north.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                // write the 3d point
                sgWriteGeod( fp, north[i] );
                WriteNeighborFaces( fp, north[i] );
            }
            gzclose(fp);

            // south edge
            if ( (fp = gzopen( file_south.c_str(), "wb9" )) == NULL ) {
                SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_south.c_str() << " for writing!" );
                return;
            }

            nCount = south.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWriteGeod( fp, south[i] );
                WriteNeighborFaces( fp, south[i] );
            }
            gzclose(fp);

            // east edge
            if ( (fp = gzopen( file_east.c_str(), "wb9" )) == NULL ) {
                SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_east.c_str() << " for writing!" );
                return;
            }

            nCount = east.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWriteGeod( fp, east[i] );
                WriteNeighborFaces( fp, east[i] );
            }
            gzclose(fp);

            // west egde
            if ( (fp = gzopen( file_west.c_str(), "wb9" )) == NULL ) {
                SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_west.c_str() << " for writing!" );
                return;
            }

            nCount = west.size();
            sgWriteInt( fp, nCount );
            for (int i=0; i<nCount; i++) {
                sgWriteGeod( fp, west[i] );
                WriteNeighborFaces( fp, west[i] );
            }
            gzclose(fp);
            
            lock->unlock();
        }
        break;
    }
}

void TGConstruct::LoadSharedEdgeData( int stage )
{
    switch( stage ) {
        case 1:
        {
            // we need to read just 4 buckets for stage 1 - 1 for each edge
            std::vector<SGGeod> north, south, east, west;
            SGBucket   nb, sb, eb, wb;

            // Read North tile and add its southern nodes
            nb = bucket.sibling(0, 1);
            LoadNeighboorEdgeDataStage1( nb, north, south, east, west );
            // Add southern nodes from northern tile
            for (unsigned int i=0; i<south.size(); i++) {
                nodes.unique_add( south[i] );
            }

            // Read South Tile and add its northern nodes
            sb = bucket.sibling(0, -1);
            LoadNeighboorEdgeDataStage1( sb, north, south, east, west );
            for (unsigned  int i=0; i<north.size(); i++) {
                nodes.unique_add( north[i] );
            }

            // Read East Tile and add its western nodes
            eb = bucket.sibling(1, 0);
            LoadNeighboorEdgeDataStage1( eb, north, south, east, west );
            for (unsigned  int i=0; i<west.size(); i++) {
                nodes.unique_add( west[i] );
            }

            // Read West Tile and add its eastern nodes
            wb = bucket.sibling(-1, 0);
            LoadNeighboorEdgeDataStage1( wb, north, south, east, west );
            for (unsigned  int i=0; i<east.size(); i++) {
                nodes.unique_add( east[i] );
            }
        }
        break;

        case 2:
        {
            string dir;
            string file;
            gzFile     fp;
            SGBucket   b;

            // Read Northern tile and add its southern node faces
            b    = bucket.sibling(0, 1);
            dir  = share_base + "/stage2/" + b.gen_base_path();
            file = dir + "/" + b.gen_index_str() + "_south_edge";
            fp = gzopen( file.c_str(), "rb" );
            if (fp) {
                ReadNeighborFaces( fp );
                gzclose( fp );
            }

            // Read Southern tile and add its northern node faces
            b    = bucket.sibling(0, -1);
            dir  = share_base + "/stage2/" + b.gen_base_path();
            file = dir + "/" + b.gen_index_str() + "_north_edge";
            fp = gzopen( file.c_str(), "rb" );
            if (fp) {
                ReadNeighborFaces( fp );
                gzclose( fp );
            }

            // Read Eastern tile and add its western node faces
            b    = bucket.sibling(1, 0);
            dir  = share_base + "/stage2/" + b.gen_base_path();
            file = dir + "/" + b.gen_index_str() + "_west_edge";
            fp = gzopen( file.c_str(), "rb" );
            if (fp) {
                ReadNeighborFaces( fp );
                gzclose( fp );
            }

            // Read Western tile and add its eastern node faces
            b    = bucket.sibling(-1, 0);
            dir  = share_base + "/stage2/" + b.gen_base_path();
            file = dir + "/" + b.gen_index_str() + "_east_edge";
            fp = gzopen( file.c_str(), "rb" );
            if (fp) {
                ReadNeighborFaces( fp );
                gzclose( fp );
            }
        }
        break;
    }
}

// Neighbor faces
void TGConstruct::WriteNeighborFaces( gzFile& fp, const SGGeod& pt ) const
{
    // find all neighboors of this point
    int               n     = nodes.find( pt );
    TGNode const&     node  = nodes.get_node( n );
    TGFaceList const& faces = node.GetFaces();

    // write the number of neighboor faces
    sgWriteInt( fp, faces.size() );

    // write out each face normal and size
    for (unsigned int j=0; j<faces.size(); j++) {
        // for each connected face, get the nodes
        unsigned int tri      = faces[j].tri;
        tgPolygon const& poly = polys_clipped.get_poly( faces[j].area, faces[j].poly );

        SGGeod const& p1 = nodes[ poly.GetTriIdx( tri, 0) ].GetPosition();
        SGGeod const& p2 = nodes[ poly.GetTriIdx( tri, 1) ].GetPosition();
        SGGeod const& p3 = nodes[ poly.GetTriIdx( tri, 2) ].GetPosition();

        SGVec3d const& wgs_p1 = nodes[ poly.GetTriIdx( tri, 0) ].GetWgs84();
        SGVec3d const& wgs_p2 = nodes[ poly.GetTriIdx( tri, 0) ].GetWgs84();
        SGVec3d const& wgs_p3 = nodes[ poly.GetTriIdx( tri, 0) ].GetWgs84();

        double  face_area   = tgTriangle::area( p1, p2, p3 );
        SGVec3f face_normal = calc_normal( face_area, wgs_p1, wgs_p2, wgs_p3 );

        sgWriteDouble( fp, face_area );
        sgWriteVec3( fp, face_normal );
    }
}

TGNeighborFaces* TGConstruct::FindNeighborFaces( const SGGeod& node )
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

TGNeighborFaces* TGConstruct::AddNeighborFaces( const SGGeod& node )
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
        SGGeod           node;
        int              num_faces;

        sgReadGeod( fp, node );

        // look to see if we already have this node
        // If we do, (it's a corner) add more faces to it.
        // otherwise, initialize it with our elevation data
        pFaces = FindNeighborFaces( node );
        if ( !pFaces ) {
            pFaces = AddNeighborFaces( node );

            // new face - let's add our elevation first
            int idx = nodes.find( node );
            if (idx >= 0) {
                TGNode const& local = nodes.get_node( idx );
                pFaces->elevations.push_back( local.GetPosition().getElevationM() );
            }
        }

        // remember all of the elevation data for the node, so we can average
        pFaces->elevations.push_back( node.getElevationM() );

        sgReadInt( fp, &num_faces );
        for (int j=0; j<num_faces; j++)
        {
            double  area;
            SGVec3f normal;

            sgReadDouble( fp, &area );
            pFaces->face_areas.push_back( area );

            sgReadVec3( fp, normal );
            pFaces->face_normals.push_back( normal );
        }
    }
}


// Tile data
void TGConstruct::SaveToIntermediateFiles( int stage )
{
    string dir;
    string file_clipped;
    string file_nodes;
    gzFile fp;

    switch( stage ) {
        case 1:     // Save the clipped polys and node list
        {
            /* Only create the file this isn't an ocean tile */
            if ( !IsOceanTile() ) {                
                dir  = share_base + "/stage1/" + bucket.gen_base_path();
                SGPath sgp( dir );
                sgp.append( "dummy" );
                file_clipped = dir + "/" + bucket.gen_index_str() + "_clipped_polys";
                file_nodes = dir + "/" + bucket.gen_index_str() + "_nodes";
                
                lock->lock();                
                
                sgp.create_dir( 0755 );
                if ( (fp = gzopen( file_clipped.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_clipped.c_str() << " for writing!" );
                    return;
                }

                polys_clipped.SaveToGzFile( fp );
                gzclose( fp );

                if ( (fp = gzopen( file_nodes.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_nodes.c_str() << " for writing!" );
                    return;
                }

                nodes.SaveToGzFile( fp );
                gzclose( fp );
                
                lock->unlock();
            }

            break;
        }

        case 2:     // Save the clipped polys and node list
        {
            if ( !IsOceanTile() ) {
                dir  = share_base + "/stage2/" + bucket.gen_base_path();

                SGPath sgp( dir );
                sgp.append( "dummy" );
                file_clipped = dir + "/" + bucket.gen_index_str() + "_clipped_polys";
                file_nodes = dir + "/" + bucket.gen_index_str() + "_nodes";
                
                lock->lock();
                sgp.create_dir( 0755 );

                if ( (fp = gzopen( file_clipped.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_clipped.c_str() << " for writing!" );
                    return;
                }

                polys_clipped.SaveToGzFile( fp );
                gzclose( fp );

                if ( (fp = gzopen( file_nodes.c_str(), "wb9" )) == NULL ) {
                    SG_LOG( SG_GENERAL, SG_INFO,"ERROR: opening " << file_nodes.c_str() << " for writing!" );
                    return;
                }

                nodes.SaveToGzFile( fp );
                gzclose( fp );
                
                lock->unlock();
            }
            break;
        }
    }
}

void TGConstruct::LoadNeighboorEdgeDataStage1( SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west )
{
    string dir;
    string file;
    gzFile fp;
    SGGeod pt;
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
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " north boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            north.push_back(pt);
        }

        // South
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " south boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            south.push_back(pt);
        }

        // East
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " east boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            east.push_back(pt);
        }

        // West
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " west boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            west.push_back(pt);
        }

        gzclose( fp );
    }
}

void TGConstruct::LoadNeighboorMatchDataStage1( SGBucket& b, std::vector<SGGeod>& north, std::vector<SGGeod>& south, std::vector<SGGeod>& east, std::vector<SGGeod>& west )
{
    string dir;
    string file;
    gzFile fp;
    SGGeod pt;
    int nCount;

    dir  = share_base + "/match1/" + b.gen_base_path();
    file = dir + "/" + b.gen_index_str() + "_edges";
    fp = gzopen( file.c_str(), "rb" );

    north.clear();
    south.clear();
    east.clear();
    west.clear();

    if (fp) {
        // North
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " north boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            north.push_back(pt);
        }

        // South
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " south boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            south.push_back(pt);
        }

        // East
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " east boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            east.push_back(pt);
        }

        // West
        sgReadInt( fp, &nCount );
        SG_LOG( SG_CLIPPER, SG_DEBUG, "loading " << nCount << "Points on " << b.gen_index_str() << " west boundary");
        for (int i=0; i<nCount; i++) {
            sgReadGeod( fp, pt );
            west.push_back(pt);
        }

        gzclose( fp );
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
        isOcean = true;
    }
}
