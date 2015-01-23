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

#include <simgear/math/SGGeometry.hxx>
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
collectBtgFiles(const BucketBox& bucketBox, const std::string& sceneryPath, const std::string& outPath, std::list<std::string>& files, std::list<SGBucket>& ocean)
{
    unsigned int level = bucketBox.getStartLevel();

    // if this bucket is level 8, then children are bucket size - read normal BTG tiles
    SG_LOG(SG_GENERAL, SG_ALERT, "collectBTGFiles : level is " << level << " file is " << bucketBox );
    if (level == 8) {
        BucketBox bucketBoxList[100];
        unsigned numTiles = bucketBox.getSubDivision(bucketBoxList, 100);

        for (unsigned i = 0; i < numTiles; ++i) {        
            std::string fileName = sceneryPath;
            fileName += bucketBoxList[i].getBucket().gen_base_path();
            fileName += std::string("/");
            fileName += bucketBoxList[i].getBucket().gen_index_str();
            fileName += std::string(".btg.gz");

            SG_LOG(SG_GENERAL, SG_ALERT, "\t" << fileName );

            if (SGPath(fileName).exists()) {
                files.push_back(fileName);
            } else {
                ocean.push_back(bucketBoxList[i].getBucket());
            }
        }
    } else {
        BucketBox bucketBoxList[100];
        unsigned numTiles = bucketBox.getSubDivision(bucketBoxList, 100);
        
        for (unsigned i = 0; i < numTiles; ++i) {
            std::string fileName = outPath;
            fileName += std::string("/");            
            for (unsigned j = 3; j <= level; j += 2) {
                fileName += bucketBoxList[i].getParentBox(j);
                fileName += std::string("/");
            }
            fileName += std::string(".btg.gz");
            
            SG_LOG(SG_GENERAL, SG_ALERT, "\t" << fileName );

            if (SGPath(fileName).exists()) {
                files.push_back(fileName);
            } else {
                // need to get all of the ocean sub buckets
                ocean.push_back(bucketBoxList[i].getBucket());
            }
            
            // collectBtgFiles(bucketBoxList[i], sceneryPath, files, ocean);
        }
    }
}

int
collapseBtg(int level, const std::string& outfile, const std::list<std::string>& infiles, const std::list<SGBucket>& ocean)
{
    Arrays arrays;
    
    for (std::list<std::string>::const_iterator i = infiles.begin(); i != infiles.end(); ++i) {
        SGBinObject binObj;
        if (!binObj.read_bin(*i)) {
            std::cerr << "Error Reading file " << *i << std::endl;
            return EXIT_FAILURE;
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "Read  tile " << *i );
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
    
    tgBtgSimplify( mesh, 0.25f, 0.5f, 0.5f, 0.0f, 0.0f, outfile );
    SG_LOG(SG_GENERAL, SG_ALERT, "Writing  tile " << outfile );
    tgWriteMeshAsBtg( mesh, SGPath(outfile) );

    return EXIT_SUCCESS;
}


int
createTree(const BucketBox& bucketBox, const std::string& sceneryPath, const std::string& outPath, unsigned level)
{
    if (bucketBox.getStartLevel() == level) {
        // We want an other level of indirection for paging
        std::list<std::string> files;
        std::list<SGBucket>    ocean;
        
        // collectBtgFiles collects all children BTGs - and ocean btgs where files are not found.  
        // TODO get the ration of land / ocean to determine what simplification to use
        collectBtgFiles(bucketBox, sceneryPath, files, ocean);
        if (files.empty()) {
            return EXIT_SUCCESS;
        }

        SG_LOG(SG_GENERAL, SG_ALERT, "Found " << files.size() << "non ocean tiles " );
        
        std::stringstream ss;
        ss << outPath << "/";
        for (unsigned i = 3; i < level; i += 2) {
            ss << bucketBox.getParentBox(i) << "/";
        }
        
        SGPath(ss.str()).create_dir(0755);
        ss << bucketBox << ".btg.gz";
        collapseBtg(level, ss.str(), files, ocean);
   
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
    
    if (level <= 9) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Create level " << level );

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
        
        return collapseBtg(level, outfile, infiles, ocean);
    }

    return 0;
}