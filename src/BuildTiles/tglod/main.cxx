// main.cxx -- top level construction routines
//
// Written by Peter Sadrozinski, started Dec 2014.
//
// Copyright (C) 2014  Curtis L. Olson  - http://www.flightgear.org/~curt
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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef _MSC_VER
#  include <windows.h>
#endif

#include <cstdio>

#include "tg_btg_mesh.hxx"

#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/debug/logstream.hxx>

#include <terragear/BucketBox.hxx>
#include <terragear/tg_shapefile.hxx>

#include "tg_geometry_arrays.hxx"

#include <Include/version.h>

// display usage and exit
#if 0
static void usage( const std::string name ) {
    SG_LOG(SG_GENERAL, SG_ALERT, "Usage: " << name);
    exit(-1);
}
#endif

// usage tglod minx, miny, maxx, maxy, level input_dir output_dir
// 

// first test : malta - generate 2 level 8 ( 0.25 x 0.25 ) tiles
//              14.00,35.75 - 14.25,36.00
//              14.25,35.75 - 14.50,36.00

void
collectBtgFiles(const BucketBox& bucketBox, const std::string& sceneryPath, std::list<std::string>& files, std::list<SGBucket>& ocean)
{
    if (bucketBox.getIsBucketSize()) {
        std::string fileName = sceneryPath;
        fileName += bucketBox.getBucket().gen_base_path();
        fileName += std::string("/");
        fileName += bucketBox.getBucket().gen_index_str();
        fileName += std::string(".btg.gz");
        if (SGPath(fileName).exists())
            files.push_back(fileName);
        else
            ocean.push_back(bucketBox.getBucket());
    } else {
        BucketBox bucketBoxList[100];
        unsigned numTiles = bucketBox.getSubDivision(bucketBoxList, 100);
        for (unsigned i = 0; i < numTiles; ++i) {
            collectBtgFiles(bucketBoxList[i], sceneryPath, files, ocean);
        }
    }
}

int
collapseBtg(const std::string& outfile, const std::list<std::string>& infiles, const std::list<SGBucket>& ocean)
{
    Arrays arrays;
    
    for (std::list<std::string>::const_iterator i = infiles.begin(); i != infiles.end(); ++i) {
        SGBinObject binObj;
        if (!binObj.read_bin(*i)) {
            std::cerr << "Error Reading file " << *i << std::endl;
            return EXIT_FAILURE;
        }

        arrays.insert(binObj);
    }

    for (std::list<SGBucket>::const_iterator i = ocean.begin(); i != ocean.end(); ++i) {
        std::vector<SGGeod>  geod;
        int_list             geod_idxs;
        std::vector<SGVec3d> vertices;
        std::vector<SGVec3f> normals;

        for (unsigned j = 0; j < 4; ++j) {
            geod.push_back(i->get_corner(j));
            geod_idxs.push_back(j);
            
            vertices.push_back(SGVec3d::fromGeod(geod.back()));
            normals.push_back(toVec3f(normalize(vertices.back())));
        }

        std::vector<SGVec2f> texCoords = sgCalcTexCoords(*i, geod, geod_idxs);
        
        arrays.insertFanGeometry("Ocean", SGVec3d::zeros(), vertices, normals, texCoords, geod_idxs, geod_idxs, geod_idxs);
    }

    // TODO create mesh from Arrays
    tgBtgMesh mesh;
    tgReadArraysAsMesh( arrays, mesh );                    
    
#if 0    
    SGBinObject bigBinObj;
    bigBinObj.set_wgs84_nodes(arrays.vertices);
    std::vector<SGVec4f> colors;
    colors.push_back(SGVec4f(1, 1, 1, 1));
    bigBinObj.set_colors(colors);
    bigBinObj.set_normals(arrays.normals);
    bigBinObj.set_texcoords(arrays.texcoords);
    bigBinObj.set_tris_v(arrays.indices);
    bigBinObj.set_tris_n(arrays.indices);
    bigBinObj.set_tris_tc(arrays.indices);
    bigBinObj.set_tri_materials(arrays.materials);

    SGBox<double> box;
    for (size_t i = 0; i < arrays.vertices.size(); ++i)
        box.expandBy(arrays.vertices[i]);
    bigBinObj.set_gbs_center(box.getCenter());
    bigBinObj.set_gbs_radius(length(box.getHalfSize()));

    for (size_t i = 0; i < arrays.vertices.size(); ++i)
        arrays.vertices[i] -= outBinObj.get_gbs_center();

    if (!outBinObj.write_bin_file(SGPath(outfile))) {
        std::cerr << "Error Writing file " << outfile << std::endl;
        return EXIT_FAILURE;
    }
#endif    

    return EXIT_SUCCESS;
}


int
createTree(const BucketBox& bucketBox, const std::string& sceneryPath, const std::string& outPath, unsigned level)
{
    if (bucketBox.getStartLevel() == level) {
        // We want an other level of indirection for paging
        std::list<std::string> files;
        std::list<SGBucket>    ocean;
        collectBtgFiles(bucketBox, sceneryPath, files, ocean);
        if (files.empty()) {
            return EXIT_SUCCESS;
        }
        
        std::stringstream ss;
        ss << outPath << "/";
        for (unsigned i = 3; i < level; i += 2) {
            ss << bucketBox.getParentBox(i) << "/";
        }
        
        SGPath(ss.str()).create_dir(0755);
        ss << bucketBox << ".btg.gz";
        collapseBtg(ss.str(), files, ocean);
   
    } else {
        BucketBox bucketBoxList[100];
        unsigned numTiles = bucketBox.getSubDivision(bucketBoxList, 100);
        for (unsigned i = 0; i < numTiles; ++i) {
            if (EXIT_FAILURE == createTree(bucketBoxList[i], sceneryPath, outPath, level)) {
                return EXIT_FAILURE;
            }
        }
    }
    
    return EXIT_SUCCESS;
}


int main(int argc, char **argv) 
{
    std::string outfile;
    std::string sceneryPath = "/share/scenery/svn/Terrain/";
    unsigned level = ~0u;
    int c;
    while ((c = getopt(argc, argv, "l:o:p:S:")) != EOF) {
        switch (c) {
            case 'l':
                level = atoi(optarg);
                break;
            case 'o':
                outfile = optarg;
                break;
            case 'p':
                sglog().set_log_classes(SG_ALL);
                sglog().set_log_priority(sgDebugPriority(atoi(optarg)));
                break;
            case 'S':
                sceneryPath = optarg;
                break;
        }
    }
    
    if (outfile.empty()) {
        std::cerr << "No output file or directory given." << std::endl;
        return EXIT_FAILURE;
    }
    
    if (level <= 8) {
        return createTree(BucketBox(-180, -90, 360, 180), sceneryPath, outfile, level);
    } else {
        std::list<std::string> infiles;
        for (int i = optind; i < argc; ++i)
            infiles.push_back(argv[i]);
        
        std::list<SGBucket> ocean;
        if (infiles.empty()) {
            std::stringstream ss;
            std::string::size_type pos = outfile.find_last_of("/\\");
            if (pos != std::string::npos)
                ss.str(outfile.substr(pos + 1));
            else
                ss.str(outfile);
            BucketBox bucketBox;
            ss >> bucketBox;
            if (ss.fail())
                return EXIT_FAILURE;
            
            collectBtgFiles(bucketBox, sceneryPath, infiles, ocean);
        }
        
        if (infiles.empty())
            return EXIT_FAILURE;
        
        return collapseBtg(outfile, infiles, ocean);
    }

    return 0;
}

#if 0        
    //
    // Parse the command-line arguments.
    //
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
        std::string arg = argv[arg_pos];

        if (arg.find("--output-dir=") == 0) {
            output_dir = arg.substr(13);
        } else if (arg.find("--work-dir=") == 0) {
            work_dir = arg.substr(11);
        } else if ( arg.find("--min-lon=") == 0 ) {
            min_lon = atof( arg.substr(10).c_str() );
        } else if ( arg.find("--max-lon=") == 0 ) {
            max_lon = atof( arg.substr(10).c_str() );
        } else if ( arg.find("--min-lat=") == 0 ) {
            min_lat = atof( arg.substr(10).c_str() );
        } else if ( arg.find("--max-lat=") == 0 ) {
            max_lat = atof( arg.substr(10).c_str() );
        } else if (arg.find("--level=") == 0) {
            level = atoi( arg.substr(8).c_str() );
        } else {
            SG_LOG( SG_GENERAL, SG_ALERT, "unknown param " << arg );
        }
    }
    
    min = SGGeod::fromDeg( min_lon, min_lat );
    max = SGGeod::fromDeg( max_lon, max_lat );
    
    if (min.isValid() && max.isValid() && (min != max))
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Longitude = " << min.getLongitudeDeg() << ':' << max.getLongitudeDeg());
        SG_LOG(SG_GENERAL, SG_ALERT, "Latitude = " << min.getLatitudeDeg() << ':' << max.getLatitudeDeg());
    } else
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Lon/Lat unset or wrong");
        exit(1);
    }

    // tile work queue
    std::vector<SGBucket> bucketList;

    // First generate the workqueue of buckets to construct
    // build all the tiles in an area
    SG_LOG(SG_GENERAL, SG_ALERT, "Building tile(s) within given bounding box");
    
    SGBucket b_min( min );
    SGBucket b_max( max );
    
    if ( b_min == b_max ) {
        bucketList.push_back( b_min );
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "  construction area spans tile boundaries");
        sgGetBuckets( min, max, bucketList );
    }

    for ( unsigned int i=0; i<bucketList.size(); i++ ) {
        SGBucket    b = bucketList[i];
        SGPath      infile  = work_dir + "/" + b.gen_base_path() + "/" + b.gen_index_str() + ".btg.gz";
        SGPath      outfile = output_dir + "/" + b.gen_base_path() + "/" + b.gen_index_str() + ".btg.gz";
        SGBinObject inobj;
            
        if ( infile.exists() ) {
            char cmd[256];
                        
            SG_LOG(SG_GENERAL, SG_ALERT, "Readding tile " << b.gen_index_str() << " : " << i << " of " << bucketList.size() );
            if ( inobj.read_bin( infile.str() ) ) {
                tgBtgMesh mesh;
                SG_LOG(SG_GENERAL, SG_ALERT, "Converting tile " << b.gen_index_str() );
                tgReadBtgAsMesh( inobj, mesh );                    
                SG_LOG(SG_GENERAL, SG_ALERT, "Simplifying tile " << b.gen_index_str() );
                tgBtgSimplify( mesh, 0.25f, 0.5f, 0.5f, 0.0f, b.get_center_lat(), b.gen_index_str() );
                SG_LOG(SG_GENERAL, SG_ALERT, "Writing  tile " << b.gen_index_str() );
                tgWriteMeshAsBtg( mesh, b.get_center(), outfile );
                
                SGPath inSTG = work_dir + "/" + b.gen_base_path() + "/" + b.gen_index_str() + ".stg";
                SGPath outSTG = output_dir + "/" + b.gen_base_path() + "/" + b.gen_index_str() + ".stg";
                
                sprintf( cmd, "cp %s %s", inSTG.c_str(), outSTG.c_str() );
                system( cmd );                
            }
        }
    }
}    
#endif
        
