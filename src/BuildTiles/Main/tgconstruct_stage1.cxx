// construct.cxx -- Class to manage the primary data used in the
//                  construction process
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

#include <boost/foreach.hpp>

#include <simgear/misc/sg_dir.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/debug/logstream.hxx>

#include <terragear/tg_array.hxx>

#include "tgconstruct_stage1.hxx"

// Constructor
tgConstructFirst::tgConstructFirst( const std::string& pfile, SGLockedQueue<SGBucket>& q, tgMutex* l) :
        workQueue(q)
{
    totalTiles = q.size();   
    lock = l;

    /* initialize tgMesh for the number of layers we have */
    if ( areaDefs.init( pfile ) ) {
        exit( -1 );
    }

    std::vector<std::string> area_names = areaDefs.get_name_array();
    tileMesh.initPriorities( area_names );  
    tileMesh.setLock( lock );
}


// Destructor
tgConstructFirst::~tgConstructFirst() { 
}

// setup
void tgConstructFirst::setPaths( const std::string& work, const std::string& dem, const std::string& share, const std::string& debug) {
    workBase   = work;
    demBase    = dem;
    shareBase  = share;
    debugBase  = debug;
}

void tgConstructFirst::safeMakeDirectory( const std::string& directory )
{
    lock->lock();
    std::string dummy = directory + "/dummy";
    SGPath sgp( dummy );
    sgp.create_dir( 0755 );
    lock->unlock();
}

void tgConstructFirst::run()
{
    unsigned int tilesComplete;

    // as long as we have feometry to parse, do so
    while ( !workQueue.empty() ) {
        bucket = workQueue.pop();
        tilesComplete = totalTiles - workQueue.size();

        SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Stage1 Construct in " << bucket.gen_base_path() << " tile " << tilesComplete << " of " << totalTiles << " using thread " << current() );

        // assume non ocean tile until proven otherwise
        isOcean = false;

        // clear mesh
        tileMesh.clear();

        if ( !debugBase.empty() ) {
            std::string debugPath = debugBase + "/tgconstruct_debug/stage1/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();                
            safeMakeDirectory( debugPath );

            tileMesh.initDebug( debugPath );
        }

        tileMesh.clipAgainstBucket( bucket );

        // STEP 1 - read in the polygon soup for this tile
        loadLandclassPolys( workBase );

        // Step 2 - add the fitted nodes ( important elevation points )
        // add them to the mesh - which adds them in triangulation
        loadElevation( demBase );

        // generate the tile
        tileMesh.generate();

        // save the intermediate data
        std::string sharedPath = shareBase + "/stage1/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
        safeMakeDirectory( sharedPath );

        lock->lock();
        tileMesh.save( sharedPath );
        lock->unlock();
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, bucket.gen_index_str() << " Thread " << current() << " finished");
}

int tgConstructFirst::loadLandclassPolys( const std::string& path )
{
    std::string         poly_path;
    tgPolygonSetList    polys;
    unsigned int        numPolys = 0;

    // load 2D polygons from correct path
    poly_path = path + "/" + bucket.gen_base_path() + '/' + bucket.gen_index_str();

    simgear::Dir d(poly_path);
    if (d.exists() ) {
        simgear::PathList files = d.children(simgear::Dir::TYPE_FILE);
        SG_LOG( SG_GENERAL, SG_DEBUG, files.size() << " Files in " << d.path() );

        BOOST_FOREACH(const SGPath& p, files) {
            std::string lext = p.complete_lower_extension();

            // look for .shp files to load
            if (lext == "shp") {
                SG_LOG(SG_GENERAL, SG_DEBUG, "load: " << p);

                // shapefile contains multiple polygons.
                // read an array of them
                tgPolygonSet::fromShapefile( p, polys );
                numPolys += polys.size();
                for ( unsigned int i=0; i<polys.size(); i++ ) {
                    std::string material = polys[i].getMeta().getMaterial();

                    int area = areaDefs.get_area_priority( material );                    
                    tileMesh.addPoly( area, polys[i] );
                }
            }
        }

        if ( !tileMesh.empty() ) {
            // add pcean polygon
            addOceanPoly();
        }
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "loadLandclassPolys - loaded " << numPolys << " polys.  mesh is empty: " << tileMesh.empty() );

    return numPolys;
}

void tgConstructFirst::loadElevation( const std::string& path ) {        
    std::string array_path = path + "/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
    tgArray     array;

    if ( array.open(array_path) ) {
        std::vector<cgalPoly_Point>  elevationPoints;

        SG_LOG(SG_GENERAL, SG_DEBUG, "Opened Array file " << array_path);

        array.parse( bucket );
        array.remove_voids( );

        std::vector<SGGeod> const& corner_list = array.get_corner_list();
        for (unsigned int i=0; i<corner_list.size(); i++) {
            elevationPoints.push_back( cgalPoly_Point(corner_list[i].getLongitudeDeg(), corner_list[i].getLatitudeDeg()) );
        }

        std::vector<SGGeod> const& fit_list = array.get_fitted_list();
        for (unsigned int i=0; i<fit_list.size(); i++) {
            elevationPoints.push_back( cgalPoly_Point(fit_list[i].getLongitudeDeg(), fit_list[i].getLatitudeDeg()) );
        }

        tileMesh.addPoints( elevationPoints );
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "Failed to open Array file " << array_path);
    }
}

#define CORRECTION  (0.0005)
void tgConstructFirst::addOceanPoly( void )
{
    // set up clipping tile
    cgalPoly_Point pt[4];

    pt[0] = cgalPoly_Point( bucket.get_corner( SG_BUCKET_SW ).getLongitudeDeg()-CORRECTION, bucket.get_corner( SG_BUCKET_SW ).getLatitudeDeg()-CORRECTION );
    pt[1] = cgalPoly_Point( bucket.get_corner( SG_BUCKET_SE ).getLongitudeDeg()+CORRECTION, bucket.get_corner( SG_BUCKET_SE ).getLatitudeDeg()-CORRECTION );
    pt[2] = cgalPoly_Point( bucket.get_corner( SG_BUCKET_NE ).getLongitudeDeg()+CORRECTION, bucket.get_corner( SG_BUCKET_NE ).getLatitudeDeg()+CORRECTION );
    pt[3] = cgalPoly_Point( bucket.get_corner( SG_BUCKET_NW ).getLongitudeDeg()-CORRECTION, bucket.get_corner( SG_BUCKET_NW ).getLatitudeDeg()+CORRECTION );    

    cgalPoly_Polygon poly( pt, pt+4 );
    tgPolygonSetMeta meta(tgPolygonSetMeta::META_TEXTURED, areaDefs.get_ocean_area_name() );

    tileMesh.addPoly( areaDefs.get_ocean_area_priority(), tgPolygonSet( poly, meta ) );
}