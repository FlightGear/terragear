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

#include "tgconstruct_stage2.hxx"

// Constructor
tgConstructSecond::tgConstructSecond( const std::string& pfile, SGLockedQueue<SGBucket>& q, tgMutex* l) :
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
tgConstructSecond::~tgConstructSecond() { 
}

// setup
void tgConstructSecond::setPaths( const std::string& work, const std::string& dem, const std::string& share, const std::string& debug) {
    workBase   = work;
    demBase    = dem;
    shareBase  = share;
    debugBase  = debug;
}

void tgConstructSecond::safeMakeDirectory( const std::string& directory )
{
    lock->lock();
    std::string dummy = directory + "/dummy";
    SGPath sgp( dummy );
    sgp.create_dir( 0755 );
    lock->unlock();
}

void tgConstructSecond::run()
{
    unsigned int tilesComplete;

    // as long as we have feometry to parse, do so
    while ( !workQueue.empty() ) {
        bucket = workQueue.pop();
        tilesComplete = totalTiles - workQueue.size();

        SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Stage 2 Construct in " << bucket.gen_base_path() << " tile " << tilesComplete << " of " << totalTiles << " using thread " << current() );

        // and clear
        tileMesh.clear();

        if ( !debugBase.empty() ) {
            std::string debugPath = debugBase + "/tgconstruct_debug/stage2/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
            safeMakeDirectory( debugPath );

            tileMesh.initDebug( debugPath );
        }

        std::string sharedStage1Base = shareBase + "/stage1/";

        // STEP 1 - read in the stage 1 tile mesh triangulation, and the shared edge nodes - remesh to fit shared edges
        isOcean = tileMesh.loadStage1( sharedStage1Base, bucket );

        if ( !isOcean ) {
#if 0
            // Step 2 - calculate elevation
            tileMesh.calcElevation( demBase );
#endif

            // save the intermediate data
            std::string sharedStage2 = shareBase + "/stage2/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
            safeMakeDirectory( sharedStage2 );

            lock->lock();
            tileMesh.save2( sharedStage2 );
            lock->unlock();
        }
    }
}

void tgConstructSecond::loadElevation( const std::string& path ) {        
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