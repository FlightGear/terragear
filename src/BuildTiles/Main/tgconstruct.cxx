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

#include <iomanip>

#include <simgear/debug/logstream.hxx>
#include "tgconstruct.hxx"

const double TGConstruct::gSnap = 0.00000001;      // approx 1 mm

// Constructor
TGConstruct::TGConstruct( const TGAreaDefinitions& areas, unsigned int s, SGLockedQueue<SGBucket>& q, SGMutex* l) :
        area_defs(areas),
        workQueue(q),
        ds_id((void*)-1),
        l_id(nullptr),
        ds_name(""),
        layer_name(""),
        feature_name(""),
        lock(l)
{
    total_tiles = q.size();
    stage = s;
    ignoreLandmass = false;
    nudge = 0.0;
    debug_all = false;
    isOcean = false;
    num_areas = areas.size();
}


// Destructor
TGConstruct::~TGConstruct() { 
    // All Nodes
    nodes.clear();
}

// TGConstruct: Setup
void TGConstruct::set_paths( const std::string& work, const std::string& share, 
                             const std::string& match, const std::string& output, 
                             const std::vector<std::string>& load ) {
    work_base   = work;
    share_base  = share;
    match_base  = match;
    output_base = output;
    load_dirs   = load;
}

void TGConstruct::set_options( bool ignore_lm, double n ) {
    ignoreLandmass = ignore_lm;
    nudge          = n;
}

void TGConstruct::run()
{
    // as long as we have feometry to parse, do so
    while ( !workQueue.empty() ) {
        bucket = workQueue.pop();
        unsigned int tiles_complete = total_tiles - workQueue.size();

        // assume non ocean tile until proven otherwise
        isOcean = false;

        // Initialize the landclass lists with the number of area definitions
        polys_in.init( num_areas );
        polys_clipped.init( num_areas );

        SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Construct in " << bucket.gen_base_path() << " tile " << tiles_complete << " of " << total_tiles << " using thread " << current() );

        // Init debug shapes and area for this bucket
        get_debug();
        if ( debug_shapes.size() || debug_all ) {
            sprintf(ds_name, "%s/constructdbg_%s", debug_path.c_str(), bucket.gen_index_str().c_str() );
        } else {
            sprintf(ds_name, "%s/constructdbg_%s", debug_path.c_str(), bucket.gen_index_str().c_str() );
        }

        if ( stage == 1 ) {
            // Matched edge data is generated from a previous scenery build - do this before we
            // start the current tile - it can mark edges as immutable
            LoadMatchedEdgeFiles();
        }
        
        if ( stage > 1 ) {
            LoadFromIntermediateFiles( stage-1 );
            LoadSharedEdgeData( stage-1 );
        }

        switch( stage ) {
            case 1:
                // STEP 1)
                // Load grid of elevation data (Array), and add the nodes
                LoadElevationArray( true );

                // STEP 2)
                // Clip 2D polygons against one another
                SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Loading landclass polys" );
                if ( LoadLandclassPolys() == 0 ) {
                    // don't build the tile if there is no 2d data ... it *must*
                    // be ocean and the sim can build the tile on the fly.
                    isOcean = true;
                    break;
                }

#if 0
                // STEP 3)
                // Load the land use polygons if the --cover option was specified
                if ( get_cover().size() > 0 ) {
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Loading landclass raster" );
                    load_landcover();
                }
#endif

                // STEP 4)
                // Clip the Landclass polygons
                SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Clipping landclass polys" );
                ClipLandclassPolys();

                // STEP 5)
                // Clean the polys - after this, we shouldn't change their shape (other than slightly for
                // fix T-Junctions - as This is the end of the first pass for multicore design
                SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Cleaning landclass polys" );
                nodes.init_spacial_query();
                CleanClippedPolys();
                break;

            case 2:
                if ( !IsOceanTile() ) {
                    // STEP 6)
                    // Need the array of elevation data for stage 2, but don't add the nodes - we already have them
                    LoadElevationArray( false );

                    // STEP 7)
                    // Fix T-Junctions by finding nodes that lie close to polygon edges, and
                    // inserting them into the edge
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Fix T-Junctions" );
                    nodes.init_spacial_query();
                    FixTJunctions();

                    // STEP 8)
                    // Generate triangles - we can't generate the node-face lookup table
                    // until all polys are tesselated, as extra nodes can still be generated
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Tesselate" );
                    TesselatePolys();

                    // STEP 9)
                    // Generate triangle vertex coordinates to node index lists
                    // NOTE: After this point, no new nodes can be added
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Lookup Nodes Per Vertex");
                    LookupNodesPerVertex();

                    // STEP 10)
                    // Interpolate elevations, and flatten stuff
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Calculate Elevation Per Node");
                    CalcElevations();

                    // ONLY do this when saving edge nodes...
                    // STEP 11)
                    // Generate face-connected list - needed for saving the edge data
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Lookup Faces Per Node");
                    LookupFacesPerNode();
                }
                break;

            case 3:
                if ( !IsOceanTile() ) {
                    // STEP 12
                    // Generate face-connectd list (again) - it was needed to save faces of the
                    // edge nodes, but saving the entire tile is i/o intensive - it's faster
                    // too just recompute the list
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Lookup Faces Per Node (again)");
                    LookupFacesPerNode();

                    // STEP 13)
                    // Average out the elevation for nodes on tile boundaries
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Average Edge Node Elevations");
                    AverageEdgeElevations();

                    // STEP 14)
                    // Calculate Face Normals
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Calculate Face Normals");
                    CalcFaceNormals();

                    // STEP 15)
                    // Calculate Point Normals
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Calculate Point Normals");
                    CalcPointNormals();

#if 0
                    // STEP 16)
                    if ( c.get_cover().size() > 0 ) {
                        // Now for all the remaining "default" land cover polygons, assign
                        // each one it's proper type from the land use/land cover
                        // database.
                        fix_land_cover_assignments( c );
                    }
#endif

                    // STEP 17)
                    // Calculate Texture Coordinates
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Calculate Texture Coordinates");
                    CalcTextureCoordinates();

                    // STEP 18)
                    // Generate the btg file
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Generate BTG File");
                    WriteBtgFile();

                    // STEP 19)
                    // Write Custom objects to .stg file
                    SG_LOG(SG_GENERAL, SG_ALERT, bucket.gen_index_str() << " - Generate Custom Objects");
                    AddCustomObjects();
                }
                break;
        }

        if ( ( stage < 3 ) && ( !IsOceanTile() ) ) {
            // Save data for next stage
            if ( stage == 2 ) {
                nodes.init_spacial_query(); // for stage 2 only...
            }
            SaveSharedEdgeData( stage );
            SaveToIntermediateFiles( stage );
        }

        // Clean up for next work queue item
        array.unload();
        polys_in.clear();
        polys_clipped.clear();
        nodes.clear();
        neighbor_faces.clear();
        debug_shapes.clear();
        debug_areas.clear();
    }
}
