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

struct subDivision {
public:
    std::string fileName;
    SGGeod min;
    SGGeod max;
    unsigned int numOcean;
    std::vector<SGBucket> land;
    std::vector<SGBucket> ocean;
};

// this recurses under given bucketbox, pushing land and ocean puckets
void
collectLandAndOcean(const BucketBox& bucketBox, const std::string& sceneryPath, const std::string& outPath, subDivision& subTile, bool saveOceanBuckets)
{
    if (bucketBox.getIsBucketSize()) {
        std::string fileName = sceneryPath;
        fileName += bucketBox.getBucket().gen_base_path();
        fileName += std::string("/");
        fileName += bucketBox.getBucket().gen_index_str();
        fileName += std::string(".btg.gz");
        if (SGPath(fileName).exists()) {
            subTile.land.push_back( bucketBox.getBucket() );
        } else {
            subTile.numOcean++;
            if ( saveOceanBuckets ) {
                subTile.ocean.push_back( bucketBox.getBucket() );
            }
        }
    } else {
        BucketBox bucketBoxList[100];
        unsigned numTiles = bucketBox.getSubDivision(bucketBoxList, 100);
        for (unsigned i = 0; i < numTiles; ++i) {
            collectLandAndOcean(bucketBoxList[i], sceneryPath, outPath, subTile, saveOceanBuckets);
        }
    }
}

// this is now non - recursive. This is only called when we wish to get the immediate submesh beneath this bucketbox
bool
collectBtgFiles(const BucketBox& bucketBox, const std::string& sceneryPath, const std::string& outPath, std::vector<subDivision>& subTiles)
{
    unsigned int level = bucketBox.getStartLevel();
    bool hasLand = false;
    bool saveOceanBuckets = false;
    
    // if this bucket is level 8, then children are bucket size - read normal BTG tiles
    if (level == 8) {
        BucketBox bucketBoxList[100];
        unsigned numTiles = bucketBox.getSubDivision(bucketBoxList, 100);
        
        for (unsigned i = 0; i < numTiles; ++i) {
            subDivision st;
            st.numOcean = 0;
            
            std::string fileName = sceneryPath;
            fileName += bucketBoxList[i].getBucket().gen_base_path();
            fileName += std::string("/");
            fileName += bucketBoxList[i].getBucket().gen_index_str();
            fileName += std::string(".btg.gz");

            if (SGPath(fileName).exists()) {
                hasLand = true;
                st.fileName = fileName;
                st.land.push_back(bucketBoxList[i].getBucket());
            } else {
                st.numOcean++;
                st.ocean.push_back(bucketBoxList[i].getBucket());                
            }

            st.min = SGGeod::fromDeg( bucketBoxList[i].getLongitudeDeg(), 
                                      bucketBoxList[i].getLatitudeDeg() );
            st.max = SGGeod::fromDeg( bucketBoxList[i].getLongitudeDeg() + bucketBoxList[i].getWidthDeg(), 
                                      bucketBoxList[i].getLatitudeDeg() + bucketBoxList[i].getHeightDeg() );
            
            subTiles.push_back(st);
        }
    } else {
        BucketBox bucketBoxList[100];
        unsigned numTiles = bucketBox.getSubDivision(bucketBoxList, 100);
        
        for (unsigned i = 0; i < numTiles; ++i) {
            subDivision st;
            st.numOcean = 0;
            
            std::stringstream ss;
            ss << outPath << "/";
            for (unsigned j = 3; j <= level; j += 2) {
                ss << bucketBoxList[i].getParentBox(j) << "/";
            }
            ss << bucketBoxList[i] << ".btg.gz";
            
            std::string fileName = ss.str();

            if (SGPath(fileName).exists()) {
                hasLand = true;
                st.fileName = fileName;
                saveOceanBuckets = false;
            } else {
                // we need to remember any ocean buckets under us
                saveOceanBuckets = true;
            }
            
            st.min = SGGeod::fromDeg( bucketBoxList[i].getLongitudeDeg(), 
                                      bucketBoxList[i].getLatitudeDeg() );
            st.max = SGGeod::fromDeg( bucketBoxList[i].getLongitudeDeg() + bucketBoxList[i].getWidthDeg(), 
                                      bucketBoxList[i].getLatitudeDeg() + bucketBoxList[i].getHeightDeg() );

            // find all of the land / ocean tiles beneath this subTile
            collectLandAndOcean( bucketBoxList[i], sceneryPath, outPath, st, saveOceanBuckets );
            
            subTiles.push_back(st);
        }
    }
    
    return hasLand;
}

int
collapseBtg(int level, const std::string& outfile, std::vector<subDivision>& subTiles)
{
    Arrays arrays;
    
    for (unsigned int i = 0; i < subTiles.size(); i++ ) {
        if ( !subTiles[i].fileName.empty() ) {
            SGBinObject binObj;
            if (!binObj.read_bin(subTiles[i].fileName)) {
                std::cerr << "Error Reading file " << subTiles[i].fileName << std::endl;
                return EXIT_FAILURE;
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "Read  tile " << subTiles[i].fileName );
            }

            arrays.insert(subTiles[i].min, subTiles[i].max, binObj);
        }
    }

    for (unsigned int i = 0; i < subTiles.size(); i++ ) {
        for ( unsigned int j = 0; j <  subTiles[i].ocean.size(); j++ ) {
            std::vector<SGGeod>  geod;
            int_list             geod_idxs;
            std::vector<SGVec3d> vertices;
            std::vector<SGVec3f> normals;

            for (unsigned k = 0; k < 4; ++k) {
                geod.push_back(subTiles[i].ocean[j].get_corner(k));
                geod_idxs.push_back(k);
            
                vertices.push_back(SGVec3d::fromGeod(geod.back()));
                normals.push_back(toVec3f(normalize(vertices.back())));
            }

            std::vector<SGVec2f> texCoords = sgCalcTexCoords(subTiles[i].ocean[j], geod, geod_idxs);
        
            arrays.insertFanGeometry("Ocean", geod[0], geod[2], SGVec3d::zeros(), vertices, normals, texCoords, geod_idxs, geod_idxs, geod_idxs);
        }
    }
    
    // determine the simplification ratio
    unsigned int num_subTiles = subTiles.size();
    float denom = 0.0f;
    for ( unsigned int i=0; i<num_subTiles; i++ ) {
        // if 100% land, then the ratio should be 1/num_subtiles.
        // but - each ocean tile decreases the simplification ratio of a subtile
        
        // example 2 sub meshes : each one consists of 2 btgs
        // 1 btg is both land, the other is 1 land, 1 ocean
        // ratio for full land is 1/2
        // ration here is 1/( 2/2 + 1/2 ) = 1/1.5 .666 
        
        denom += ( subTiles[i].land.size() / ( subTiles[i].land.size() + subTiles[i].numOcean ) );
    }
    
    // float simpRatio = 1.0f/denom;
    float simpRatio = 1.0f/num_subTiles;
    
    // TODO create mesh from Arrays
    tgBtgMesh mesh;
    tgReadArraysAsMesh( arrays, mesh, outfile );                    
    
    SG_LOG(SG_GENERAL, SG_ALERT, "Simplifying tile " << outfile << " with ratio " << simpRatio );

    tgBtgSimplify( mesh, simpRatio, 0.5f, 0.5f, 0.0f, 0.0f, outfile );
    tgWriteMeshAsBtg( mesh, SGPath(outfile) );

    return EXIT_SUCCESS;
}

int
createTree(const BucketBox& bucketBox, const std::string& sceneryPath, const std::string& outPath, unsigned level)
{
    if (bucketBox.getStartLevel() == level) {
        // We want an other level of indirection for paging
        //std::list<std::string> files;   // actual files to be read as mesh for collapse
        //std::list<SGBucket>    land;    // level 9 buckets that are non-ocean
        //std::list<SGBucket>    ocean;   // level 9 buckets that are ocean
        std::vector<subDivision>   subTiles;
        
        // collectBtgFiles collects all children BTGs - and ocean btgs where files are not found.  
        // TODO get the ration of land / ocean to determine what simplification to use
        bool hasLand = collectBtgFiles(bucketBox, sceneryPath, outPath, subTiles);
        if (!hasLand) {
            return EXIT_SUCCESS;
        }
                
        std::stringstream ss;
        ss << outPath << "/";
        for (unsigned i = 3; i < level; i += 2) {
            ss << bucketBox.getParentBox(i) << "/";
        }
        
        SGPath(ss.str()).create_dir(0755);
        ss << bucketBox << ".btg.gz";
        collapseBtg(level, ss.str(), subTiles);
   
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
        SG_LOG(SG_GENERAL, SG_ALERT, "Create level " << level );
        return createTree(BucketBox(-180, -90, 360, 180), sceneryPath, outfile, level);
    }

    return 0;
}