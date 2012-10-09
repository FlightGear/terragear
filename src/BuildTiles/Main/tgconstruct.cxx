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

//#include <boost/foreach.hpp>

//#include <simgear/compiler.h>
//#include <simgear/constants.h>
//#include <simgear/math/sg_geodesy.hxx>
//#include <simgear/math/SGGeometry.hxx>
//#include <simgear/misc/sg_dir.hxx>
//#include <simgear/misc/texcoord.hxx>
//#include <simgear/io/sg_binobj.hxx>
//#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
//#include <CGAL/Plane_3.h>

//#include <Geometry/poly_support.hxx>
//#include <Geometry/poly_extra.hxx>

#include "tgconstruct.hxx"
//#include "usgs.hxx"

//using std::string;

// Constructor
TGConstruct::TGConstruct():
        ignoreLandmass(false),
        debug_all(false),
        ds_id((void*)-1)
{ }


// Destructor
TGConstruct::~TGConstruct() { 
    array.close();

    // land class polygons
    polys_in.clear();
    polys_clipped.clear();

    // All Nodes
    nodes.clear();
}

// TGConstruct: Setup
void TGConstruct::set_paths( const std::string work, const std::string share, const std::string output, const std::vector<std::string> load ) {
    work_base   = work;
    share_base  = share;
    output_base = output;
    load_dirs   = load;
}

void TGConstruct::set_options( bool ignore_lm, double n ) {
    ignoreLandmass = ignore_lm;
    nudge          = n;
}

// master construction routine
// TODO : Split each step into its own function, and move 
//        into seperate files by major functionality
//        loading, clipping, tesselating, normals, and output
//        Also, we are still calculating some thing more than one 
//        (like face area - need to move this into superpoly )
void TGConstruct::ConstructBucketStage1() {
    // First, set the precision of floating point logging:
    SG_LOG(SG_GENERAL, SG_ALERT, std::setprecision(12) << std::fixed);
    SG_LOG(SG_GENERAL, SG_ALERT, "\nConstructing tile ID " << bucket.gen_index_str() << " in " << bucket.gen_base_path() );

    /* If we have some debug IDs, create a datasource */
    if ( debug_shapes.size() || debug_all ) {
        sprintf(ds_name, "%s/constructdbg_%s", debug_path.c_str(), bucket.gen_index_str().c_str() );
        SG_LOG(SG_GENERAL, SG_ALERT, "Debug_string: " << ds_name );
    } else {
        strcpy( ds_name, "" );
    }

    // STEP 1) 
    // Load grid of elevation data (Array)
    LoadElevationArray();

    // STEP 2) 
    // Clip 2D polygons against one another
    if ( LoadLandclassPolys() == 0 ) {
        // don't build the tile if there is no 2d data ... it *must*
        // be ocean and the sim can build the tile on the fly.
        return;
    }

    // STEP 3)
    // Load the land use polygons if the --cover option was specified
    if ( get_cover().size() > 0 ) {
        load_landcover();
    }

    // STEP 4)
    // Clip the Landclass polygons    
    ClipLandclassPolys(); 

    // STEP 5)
    // Clean the polys - after this, we shouldn't change their shape (other than slightly for
    // fix T-Junctions - as This is the end of the first pass for multicore design
    CleanClippedPolys();

    // STEP 6)
    // Save the tile boundary info for stage 2 (just x,y coords of points on the boundary)
    SaveSharedEdgeData( 1 );
}

void TGConstruct::ConstructBucketStage2() {
    // First, set the precision of floating point logging:
    SG_LOG(SG_GENERAL, SG_ALERT, std::setprecision(12) << std::fixed);
    SG_LOG(SG_GENERAL, SG_ALERT, "\nConstructing tile ID " << bucket.gen_index_str() << " in " << bucket.gen_base_path() );

    /* If we have some debug IDs, create a datasource */
    if ( debug_shapes.size() || debug_all ) {
        sprintf(ds_name, "%s/constructdbg_%s", debug_path.c_str(), bucket.gen_index_str().c_str() );
        SG_LOG(SG_GENERAL, SG_ALERT, "Debug_string: " << ds_name );
    } else {
        strcpy( ds_name, "" );
    }

    // STEP 7) 
    // Need the array of elevation data for stage 2
    LoadElevationArray();

    // STEP 6)
    // Merge in Shared data - should just be x,y nodes on the borders from stage1
    LoadSharedEdgeData( 1 );

    // STEP 7) 
    // Fix T-Junctions by finding nodes that lie close to polygon edges, and
    // inserting them into the edge
    FixTJunctions();

    // STEP 8)
    // Generate triangles - we can't generate the node-face lookup table
    // until all polys are tesselated, as extra nodes can still be generated
    TesselatePolys();

    // STEP 9)
    // Generate triangle vertex coordinates to node index lists
    // NOTE: After this point, no new nodes can be added
    LookupNodesPerVertex();

    // STEP 10)
    // Interpolate elevations, and flatten stuff
    CalcElevations();

    // STEP 11)
    // Generate face_connected list
    LookupFacesPerNode();

    // STEP 12)
    // Save the tile boundary info for stage 3
    // includes elevation info, and a list of connected triangles
    SaveSharedEdgeData( 2 );
}

void TGConstruct::ConstructBucketStage3() {
    // First, set the precision of floating point logging:
    SG_LOG(SG_GENERAL, SG_ALERT, std::setprecision(12) << std::fixed);
    SG_LOG(SG_GENERAL, SG_ALERT, "\nConstructing tile ID " << bucket.gen_index_str() << " in " << bucket.gen_base_path() );

    /* If we have some debug IDs, create a datasource */
    if ( debug_shapes.size() || debug_all ) {
        sprintf(ds_name, "%s/constructdbg_%s", debug_path.c_str(), bucket.gen_index_str().c_str() );
        SG_LOG(SG_GENERAL, SG_ALERT, "Debug_string: " << ds_name );
    } else {
        strcpy( ds_name, "" );
    }

    // Load in the neighbor faces and elevation data
    LoadSharedEdgeDataStage2();

    // STEP 12)
    // Average out the elevation for nodes on tile boundaries
    AverageEdgeElevations();

    // STEP 12)
    // Calculate Face Normals
    CalcFaceNormals();

    // STEP 13)
    // Calculate Point Normals
    CalcPointNormals();

#if 0
    if ( c.get_cover().size() > 0 ) {
        // Now for all the remaining "default" land cover polygons, assign
        // each one it's proper type from the land use/land cover
        // database.
        fix_land_cover_assignments( c );
    }
#endif

    // STEP 14)
    // Calculate Texture Coordinates
    CalcTextureCoordinates();

    // STEP 16)
    // Generate the btg file
    WriteBtgFile();

    // STEP 17) 
    // Write Custom objects to .stg file 
    AddCustomObjects();
}