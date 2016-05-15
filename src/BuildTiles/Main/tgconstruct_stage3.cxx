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

#include "tgconstruct_stage3.hxx"

// Constructor
tgConstructThird::tgConstructThird( const std::string& pfile, SGLockedQueue<SGBucket>& q, tgMutex* l) :
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
tgConstructThird::~tgConstructThird() { 
}

// setup
void tgConstructThird::setPaths( const std::string& work, const std::string& dem, const std::string& share, const std::string& debug, const std::string& output ) {
    workBase   = work;
    demBase    = dem;
    shareBase  = share;
    debugBase  = debug;
    outputBase = output;
}

void tgConstructThird::run()
{
    unsigned int tilesComplete;

    SG_LOG(SG_GENERAL, SG_ALERT, " - Construct thread started " );
    
    // as long as we have feometry to parse, do so
    while ( !workQueue.empty() ) {
        SG_LOG(SG_GENERAL, SG_ALERT, " - workqueue not empty " );
        
        bucket = workQueue.pop();

        tilesComplete = totalTiles - workQueue.size();

        // assume non ocean tile until proven otherwise
        isOcean = false;

#if 0        
        if (   ( bucket.gen_index() != 3006851 )
            && ( bucket.gen_index() != 3023235 )
            && ( bucket.gen_index() != 3039619 )
            && ( bucket.gen_index() != 3056003 )
            && ( bucket.gen_index() != 3072387 )
            && ( bucket.gen_index() != 3105155 )
            && ( bucket.gen_index() != 3121539 )
#else
        if ( true
#endif            
        ) {       
            if ( !debugBase.empty() ) {
                SG_LOG(SG_GENERAL, SG_ALERT, " - Generate debug " );
                
                std::string debugPath = debugBase + "/tgconstruct_debug/stage2" + bucket.gen_base_path() + "/" + bucket.gen_index_str();

                lock->lock();
                std::string dummy = debugPath + "/dummy";
                SGPath sgp( dummy );
                sgp.create_dir( 0755 );            
                lock->unlock();
                
                SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Construct in " << bucket.gen_base_path() << " tile " << tilesComplete << " of " << totalTiles << " debug path is " << debugPath );
                tileMesh.initDebug( debugPath );
            }
            
            std::string sharedStage2Base = shareBase + "/stage12";
                        
            SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Construct in " << bucket.gen_base_path() << " tile " << tilesComplete << " of " << totalTiles << " using thread " << current() );

            // STEP 1 - read in the stage 1 tile mesh triangulation, and the shared edge nodes - remesh to fit shared edges
            loadMesh( sharedStage2Base );
            
            // Step 2 - calculate elevation
            tileMesh.calcFaceNormals();
            
            // and clear
            tileMesh.clear();
        }
    }
}

int tgConstructThird::loadMesh( const std::string& path )
{    
    tileMesh.loadStage2( path, bucket );    
    
    return 0;
}