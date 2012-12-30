#include <list>
#include <ctime>

#include <stdio.h>

#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/misc/texcoord.hxx>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_chopper.hxx>
#include <terragear/tg_unique_geod.hxx>
#include <terragear/tg_unique_vec3f.hxx>
#include <terragear/tg_unique_vec2f.hxx>

#include "airport.hxx"
#include "beznode.hxx"
#include "debug.hxx"
#include "elevations.hxx"
#include "global.hxx"
#include "helipad.hxx"
#include "runway.hxx"
#include "output.hxx"


Airport::Airport( int c, char* def)
{
    int   numParams;
    char* tok;
    int   ct = 0;

    code = c;

    // we need to tokenize airports, since we can't scanf two strings next to each other...
    numParams = 0;
    bool done = false;

    while (!done)
    {
        // trim leading whitespace
        while(isspace(*def)) def++;

        tok = strtok(def, " \t\r\n");

        if (tok)
        {
            def += strlen(tok)+1;

            switch( numParams )
            {
                case 0:
                    altitude = atoi(tok);
                    break;

                case 1:
                    ct = atoi(tok);
                    break;

                case 2:
                    // deprecated - ignore
                    break;

                case 3:
                    icao = tok;
                    description = def;
                    done = true;
                    break;
            }
        }
        numParams++;
    }

    altitude *= SG_FEET_TO_METER;

    GENAPT_LOG( SG_GENERAL, SG_DEBUG, "Read airport with icao " << icao << ", control tower " << ct << ", and description " << description );
}

Airport::~Airport()
{
    for (unsigned int i=0; i<features.size(); i++)
    {
        delete features[i];
    }

    for (unsigned int i=0; i<helipads.size(); i++)
    {
        delete helipads[i];
    }

    for (unsigned int i=0; i<runways.size(); i++)
    {
        delete runways[i];
    }

    for (unsigned int i=0; i<waterrunways.size(); i++)
    {
        delete waterrunways[i];
    }

    for (unsigned int i=0; i<pavements.size(); i++)
    {
        delete pavements[i];
    }

    for (unsigned int i=0; i<taxiways.size(); i++)
    {
        delete taxiways[i];
    }

    for (unsigned int i=0; i<lightobjects.size(); i++)
    {
        delete lightobjects[i];
    }

    for (unsigned int i=0; i<windsocks.size(); i++)
    {
        delete windsocks[i];
    }

    for (unsigned int i=0; i<beacons.size(); i++)
    {
        delete beacons[i];
    }

    for (unsigned int i=0; i<signs.size(); i++)
    {
        delete signs[i];
    }

    for (unsigned int i=0; i<boundary.size(); i++)
    {
        delete boundary[i];
    }
}

bool Airport::isDebugRunway( int rwy )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_runways.find(icao);
    if ( it != debug_runways.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == rwy+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}

bool Airport::isDebugPavement( int pvmt )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_pavements.find(icao);
    if ( it != debug_pavements.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == pvmt+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}

bool Airport::isDebugTaxiway( int taxi )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_taxiways.find(icao);
    if ( it != debug_taxiways.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == taxi+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}

bool Airport::isDebugFeature( int feat )
{
    bool dbg = false;

    debug_map_const_iterator it = debug_features.find(icao);
    if ( it != debug_features.end() ) {
        for ( unsigned int i=0; i<it->second.size() && !dbg; i++ ) {
            if( it->second[i] == std::numeric_limits<int>::max() ) {
                dbg = true;
            } else if ( it->second[i] == feat+1 ) {
                dbg = true;
            }
        }
    }

    return dbg;
}


// TODO : Add somewhere
// Determine node elevations of a point_list based on the provided
// TGAptSurface.  Offset is added to the final elevation
static std::vector<SGGeod> calc_elevations( const tgSurface& surf, const std::vector<SGGeod>& geod_nodes, double offset )
{
    std::vector<SGGeod> result = geod_nodes;
    for ( unsigned int i = 0; i < result.size(); ++i ) {
        double elev = surf.query( result[i] );
        result[i].setElevationM( elev + offset );
    }

    return result;
}


static tgContour calc_elevations( const tgSurface& surf, const tgContour& geod_nodes, double offset )
{
    tgContour result = geod_nodes;
    for ( unsigned int i = 0; i < result.GetSize(); ++i ) {
        SGGeod node = result.GetNode(i);
        double elev = surf.query( node );
        node.setElevationM( elev + offset );
        result.SetNode( i, node );
    }

    return result;
}

static double calc_elevation( const tgSurface& surf, const SGGeod& node, double offset )
{
    double elev = surf.query( node );
    elev += offset;

    return elev;
}


// Determine node elevations of each node of a TGPolygon based on the
// provided TGAptSurface.  Offset is added to the final elevation
static tgPolygon calc_elevations( const tgSurface& surf, const tgPolygon& poly, double offset )
{
    tgPolygon result;

    for ( unsigned int i = 0; i < poly.Contours(); ++i ) {
        tgContour contour = poly.GetContour( i );
        tgContour elevated = calc_elevations( surf, contour, offset );

        result.AddContour( elevated );
    }

    return result;
}

void Airport::BuildBtg(const std::string& root, const string_list& elev_src )
{
    tgcontour_list slivers;
    tgcontour_list line_slivers;

    tgpolygon_list apt_base_polys;
    tgpolygon_list apt_clearing_polys;

    // runways
    tgpolygon_list rwy_polys;

    // pavements
    tgpolygon_list pvmt_polys;

    // linear features
    tgpolygon_list line_polys;

    // runway lights
    tglightcontour_list rwy_lights;

    bool make_shapefiles = false;

    // parse main airport information
    double apt_lon = 0.0, apt_lat = 0.0;

    SGTimeStamp build_start;
    SGTimeStamp build_end;
    SGTimeStamp cleanup_start;
    SGTimeStamp cleanup_end;
    SGTimeStamp triangulation_start;
    SGTimeStamp triangulation_end;
    time_t      log_time;

    char shapefile_name[64];
    std::string shapefile;
    
    // Find the average of all the runway and heliport long / lats
    int num_samples = 0;
    for (unsigned int i=0; i<runways.size(); i++)
    {
        apt_lon += runways[i]->GetMidpoint().getLongitudeDeg();
        apt_lat += runways[i]->GetMidpoint().getLatitudeDeg();
        num_samples++;
    }
    for (unsigned int i=0; i<helipads.size(); i++)
    {
        apt_lon += helipads[i]->GetLoc().getLongitudeDeg();
        apt_lat += helipads[i]->GetLoc().getLatitudeDeg();
        num_samples++;
    }
    apt_lon = apt_lon / (double)num_samples;
    apt_lat = apt_lat / (double)num_samples;

    SGBucket b( apt_lon, apt_lat );
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, b.gen_base_path() << "/" << b.gen_index_str());

    // If we are cutting in the linear features, add them first
    if (pavements.size())
    {
        for ( unsigned int i=0; i<pavements.size(); i++ )
        {
            AddFeatures( pavements[i]->GetFeatures() );
        }
    }

    GENAPT_LOG(SG_GENERAL, SG_INFO, "Parse Complete - Runways: " << runways.size() << " Pavements: " << pavements.size() << " Features: " << features.size() << " Taxiways: " << taxiways.size() );

    // Starting to clip the polys (for now - only UNIX builds)
    build_start.stamp();

    // Add the linear features
    if (features.size())
    {
        tgAccumulator lf_accum;

        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << features.size() << " Linear Feature Polys");
        for ( unsigned int i=0; i<features.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );

            features[i]->BuildBtg( line_polys, rwy_lights, lf_accum, make_shapefiles );
        }

//        lf_accum.ToShapefiles( "./lf_accum", "test", false );

        log_time = time(0);
        GENAPT_LOG( SG_GENERAL, SG_ALERT, "Finished building Linear Features for " << icao << " at " << DebugTimeToString(log_time) );
    }

    /* Initialize a new accumulator for the other objects */
    tgAccumulator pvmt_accum;

    // Build runways next
    if (runways.size())
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << runways.size() << " Runway Polys");
        for ( unsigned int i=0; i<runways.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build Runway " << i + 1 << " of " << runways.size());
            slivers.clear();

            if ( isDebugRunway(i) ) {
                sprintf( shapefile_name, "runway_%d", i );
            } else {
                strcpy( shapefile_name, "" );
            }
            shapefile = shapefile_name;

            if (boundary.size())
            {
                runways[i]->BuildBtg( rwy_polys, rwy_lights, slivers, pvmt_accum, shapefile );
            }
            else
            {
                runways[i]->BuildBtg( rwy_polys, rwy_lights, slivers, apt_base_polys, apt_clearing_polys, pvmt_accum, shapefile );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
        }

        log_time = time(0);
        GENAPT_LOG( SG_GENERAL, SG_ALERT, "Finished building runways for " << icao << " at " << DebugTimeToString(log_time) );
    }

    if (lightobjects.size())
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << lightobjects.size() << " Light Objects ");
        for ( unsigned int i=0; i<lightobjects.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build runway light " << i + 1 << " of " << lightobjects.size());
            lightobjects[i]->BuildBtg( rwy_lights );
        }
    }

    // Build helipads (use runway poly- and texture list for this)
    if (helipads.size())
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << helipads.size() << " Helipad Polys ");
        for ( unsigned int i=0; i<helipads.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build helipad " << i + 1 << " of " << helipads.size());
            slivers.clear();

            if (boundary.size())
            {
                helipads[i]->BuildBtg( rwy_polys, rwy_lights, slivers, pvmt_accum );
            }
            else
            {
                helipads[i]->BuildBtg( rwy_polys, rwy_lights, slivers, apt_base_polys, apt_clearing_polys, pvmt_accum );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
        }
    }

    // Build the pavements
    if (pavements.size())
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << pavements.size() << " Pavement Polys ");
        for ( unsigned int i=0; i<pavements.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement " << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
            slivers.clear();

            if ( isDebugPavement(i) ) {
                sprintf( shapefile_name, "pvmnt_%d", i );
            } else {
                strcpy( shapefile_name, "" );
            }
            shapefile = shapefile_name;

            if (boundary.size())
            {
                pavements[i]->BuildBtg( pvmt_polys, slivers, pvmt_accum, shapefile );
            }
            else
            {
                pavements[i]->BuildBtg( pvmt_polys, slivers, apt_base_polys, apt_clearing_polys, pvmt_accum, shapefile );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
            tgPolygon::MergeSlivers( pvmt_polys, slivers );
        }

        log_time = time(0);
        GENAPT_LOG( SG_GENERAL, SG_ALERT, "Finished building Pavements for " << icao << " at " << DebugTimeToString(log_time) );
    }

    // Build the legacy taxiways
    if (taxiways.size())
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << taxiways.size() << " Taxiway Polys ");
        for ( unsigned int i=0; i<taxiways.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build Taxiway " << i + 1 << " of " << taxiways.size());
            slivers.clear();

            if ( isDebugTaxiway(i) ) {
                sprintf( shapefile_name, "taxiway_%d", i );
            } else {
                strcpy( shapefile_name, "" );
            }
            shapefile = shapefile_name;

            if (boundary.size())
            {
                taxiways[i]->BuildBtg( pvmt_polys, rwy_lights, slivers, pvmt_accum, shapefile );
            }
            else
            {
                taxiways[i]->BuildBtg( pvmt_polys, rwy_lights, slivers, apt_base_polys, apt_clearing_polys, pvmt_accum, shapefile );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
            tgPolygon::MergeSlivers( pvmt_polys, slivers );
        }
    }

    // Build runway shoulders here
    if ( runways.size() )
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << runways.size() << " Runway Shoulder Polys ");
        for ( unsigned int i=0; i<runways.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build Runway shoulder " << i + 1 << " of " << runways.size());

            if ( runways[i]->GetsShoulder() )
            {
                slivers.clear();
                runways[i]->BuildShoulder( rwy_polys, slivers, pvmt_accum );

                // Now try to merge any slivers we found
                tgPolygon::MergeSlivers( rwy_polys, slivers );
                tgPolygon::MergeSlivers( pvmt_polys, slivers );
            }
        }
    }

    // Build helipad shoulders here
    if ( helipads.size() )
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << runways.size() << " Helipad Shoulder Polys ");
        for ( unsigned int i=0; i<helipads.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build Helipad shoulder " << i + 1 << " of " << helipads.size());

            if ( helipads[i]->GetsShoulder() )
            {
                slivers.clear();
                helipads[i]->BuildShoulder( rwy_polys, slivers, pvmt_accum );

                // Now try to merge any slivers we found
                tgPolygon::MergeSlivers( rwy_polys, slivers );
                tgPolygon::MergeSlivers( pvmt_polys, slivers );
            }
        }
    }

    // build the base and clearing if there's a boundary
    tgPolygon apt_base, apt_clearing;
    if (boundary.size())
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Build " << boundary.size() << " Boundary Polys ");
        shapefile = "";

        for ( unsigned int i=0; i<boundary.size(); i++ )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Build Userdefined boundary " << i + 1 << " of " << boundary.size());
            boundary[i]->BuildBtg( apt_base, apt_clearing, shapefile );
        }
    } else {
        apt_base     = tgPolygon::Union( apt_base_polys );
        apt_clearing = tgPolygon::Union( apt_clearing_polys );
    }

    if ( apt_base.TotalNodes() == 0 )
    {
        GENAPT_LOG(SG_GENERAL, SG_ALERT, "no airport points generated");
    	return;
    }

    tgPolygon filled_base  = tgPolygon::StripHoles( apt_base );
    tgPolygon divided_base = tgPolygon::SplitLongEdges( filled_base, 200.0 );
    tgPolygon base_poly    = pvmt_accum.Diff( divided_base );

    build_end.stamp();
    build_time = build_end - build_start;

    cleanup_start.stamp();

    // add segments to polygons to remove any possible "T"
    // intersections
    UniqueSGGeodSet tmp_pvmt_nodes;
    UniqueSGGeodSet tmp_feat_nodes;

    // build temporary node list from runways...
    GENAPT_LOG(SG_GENERAL, SG_INFO, "Build Node List " );
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k ) 
    {
    	tgPolygon poly = rwy_polys[k];
    	for ( unsigned int i = 0; i < poly.Contours(); ++i )
        {
    	    for ( unsigned int j = 0; j < poly.ContourSize( i ); ++j )
            {
                tmp_pvmt_nodes.add( poly.GetNode(i, j) );
    	    }
    	}
    }

    // and pavements
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k ) 
    {
    	tgPolygon poly = pvmt_polys[k];
    	for ( unsigned int i = 0; i < poly.Contours(); ++i )
        {
    	    for ( unsigned int j = 0; j < poly.ContourSize( i ); ++j )
            {
        		tmp_pvmt_nodes.add( poly.GetNode(i, j) );
    	    }
    	}
    }

    // and linear features ( keep Linear feature nodes seperate)
    for ( unsigned int k = 0; k < line_polys.size(); ++k ) 
    {
    	tgPolygon poly = line_polys[k];
    	for ( unsigned int i = 0; i < poly.Contours(); ++i )
        {
    	    for ( unsigned int j = 0; j < poly.ContourSize( i ); ++j )
            {
        		tmp_feat_nodes.add( poly.GetNode(i, j) );
    	    }
    	}
    }

    // and the base
    for ( unsigned int i = 0; i < base_poly.Contours(); ++i )
    {
    	for ( unsigned int j = 0; j < base_poly.ContourSize( i ); ++j )
        {
    	    tmp_pvmt_nodes.add( base_poly.GetNode(i, j) );
    	}
    }

    // the divided base could contain points not found in base_poly,
    // so we should add them because the skirt needs them.
    for ( unsigned int i = 0; i < divided_base.Contours(); ++i )
    {
        for ( unsigned int j = 0; j < divided_base.ContourSize( i ); ++j )
        {
            tmp_pvmt_nodes.add( divided_base.GetNode(i, j) );
        }
    }

    log_time = time(0);
    GENAPT_LOG( SG_GENERAL, SG_ALERT, "Finished collecting nodes for " << icao << " at " << DebugTimeToString(log_time) );

    // second pass : runways
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k ) 
    {
        tgPolygon poly = rwy_polys[k];
        poly = tgPolygon::AddColinearNodes( poly, tmp_pvmt_nodes );
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.TotalNodes());
        rwy_polys[k] = poly;
    }

    // second pass : and pavements
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k ) 
    {
        tgPolygon poly = pvmt_polys[k];
        poly = tgPolygon::AddColinearNodes( poly, tmp_pvmt_nodes );
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.TotalNodes());
        pvmt_polys[k] = poly;
    }

    // second pass : and lines
    for ( unsigned int k = 0; k < line_polys.size(); ++k ) 
    {
        tgPolygon poly = line_polys[k];
        poly = tgPolygon::AddColinearNodes( poly, tmp_feat_nodes );
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.TotalNodes());
        line_polys[k] = poly;
    }

    log_time = time(0);
    GENAPT_LOG( SG_GENERAL, SG_ALERT, "Finished adding intermediate nodes for " << icao << " at " << DebugTimeToString(log_time) );

    for ( unsigned int k = 0; k < line_polys.size(); ++k )
    {
        tgPolygon poly = line_polys[k];

#if 1
        poly = tgPolygon::RemoveCycles( poly );
        poly = tgPolygon::RemoveDups( poly );
        poly = tgPolygon::RemoveBadContours( poly );

//        poly = tgPolygon::Simplify( poly );
//        poly = tgPolygon::RemoveTinyContours( poly );
//        poly = tgPolygon::RemoveSpikes( poly );
//        poly = tgPolygon::RemoveDups( poly );
//        poly = tgPolygon::RemoveBadContours( poly );
//        poly = tgPolygon::RemoveTinyContours( poly );
#endif

        line_polys[k] = poly;
    }

    log_time = time(0);
    GENAPT_LOG( SG_GENERAL, SG_ALERT, "Finished cleaning polys for " << icao << " at " << DebugTimeToString(log_time) );

    base_poly = tgPolygon::AddColinearNodes( base_poly, tmp_pvmt_nodes );
    base_poly = tgPolygon::Snap( base_poly, gSnap );

    // Finally find slivers in base
    slivers.clear();
    tgPolygon::RemoveSlivers( base_poly, slivers );
    tgPolygon::MergeSlivers( rwy_polys, slivers );
    tgPolygon::MergeSlivers( pvmt_polys, slivers );

    // Then snap rwy and pavement to grid (was done right after adding intermediate nodes...)
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k ) 
    {
    	tgPolygon poly = rwy_polys[k];
        poly = tgPolygon::Snap(poly, gSnap);
        poly = tgPolygon::RemoveDups( poly );
        poly = tgPolygon::RemoveBadContours( poly );
    	rwy_polys[k] = poly;
    }
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k ) 
    {
    	tgPolygon poly = pvmt_polys[k];
        poly = tgPolygon::Snap(poly, gSnap);
        poly = tgPolygon::RemoveDups( poly );
        poly = tgPolygon::RemoveBadContours( poly );
    	pvmt_polys[k] = poly;
    }

    cleanup_end.stamp();
    cleanup_time = cleanup_end - cleanup_start;

    triangulation_start.stamp();

    // tesselate the polygons and prepair them for final output
    if ( rwy_polys.size() )
    {
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Tesselating " << rwy_polys.size() << " Runway Polys " );
        for ( unsigned int i = 0; i < rwy_polys.size(); ++i )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Tesselating runway poly = " << i + 1 << " of " << rwy_polys.size() );

            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "contours before " << rwy_polys[i].Contours() << " total points before = " << rwy_polys[i].TotalNodes());
            rwy_polys[i].Tesselate();
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << rwy_polys[i].Triangles());
            rwy_polys[i].Texture();
        }
    }

    if ( pvmt_polys.size() )
    {
        // tesselate the polygons and prepair them for final output
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Tesselating " << pvmt_polys.size() << " Pavement Polys " );
        for ( unsigned int i = 0; i < pvmt_polys.size(); ++i )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Tesselating pavement poly = " << i + 1 << " of " << pvmt_polys.size() );

            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "contours before " << pvmt_polys[i].Contours() << " total points before = " << pvmt_polys[i].TotalNodes());
            pvmt_polys[i].Tesselate();
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << pvmt_polys[i].Triangles());
            pvmt_polys[i].Texture();
        }
    }

    if ( line_polys.size() )
    {
        // tesselate the polygons and prepair them for final output
        GENAPT_LOG(SG_GENERAL, SG_INFO, "Tesselating " << line_polys.size() << " Linear Feature Polys " );
        for ( unsigned int i = 0; i < line_polys.size(); ++i )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Tesselating line poly = " << i + 1 << " of " << line_polys.size() );

            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "contours before " << line_polys[i].Contours() << " total points before = " << line_polys[i].TotalNodes());
            line_polys[i].Tesselate();
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << line_polys[i].Triangles());
            line_polys[i].Texture();
        }
    }

    /* before tessellating the base, make sure there are no
       intersecting contours */
    base_poly = tgPolygon::Simplify( base_poly );

    GENAPT_LOG(SG_GENERAL, SG_INFO, "Tesselating base poly : " << base_poly.Contours() << " contours " );
    base_poly.Tesselate();
    GENAPT_LOG(SG_GENERAL, SG_INFO, "Tesselating base poly - done : Triangles = " << base_poly.Triangles());
    // should we texture base here?

    triangulation_end.stamp();

    // a few airports fail here
    if ( base_poly.Triangles() == 0 )
    {
        GENAPT_LOG(SG_GENERAL, SG_ALERT, "no base poly triangles");
        return;
    }

    triangulation_time = triangulation_end - triangulation_start;

    //
    // We should now have the runway polygons all generated with their
    // corresponding triangles and texture coordinates, and the
    // surrounding base area.
    //
    // Next we need to create the output lists ... vertices, normals,
    // texture coordinates, and tri-strips
    //

    // traverse the tri list and create ordered node and texture
    // coordinate lists
    // start with just nodes
    UniqueSGGeodSet  nodes;
    UniqueSGVec3fSet normals;
    UniqueSGVec2fSet texcoords;

    group_list pts_v; pts_v.clear();
    group_list pts_n; pts_n.clear();
    string_list pt_materials; pt_materials.clear();

    group_list tris_v; tris_v.clear();
    group_list tris_n; tris_n.clear();
    group_list tris_tc; tris_tc.clear();
    string_list tri_materials; tri_materials.clear();

    group_list strips_v; strips_v.clear();
    group_list strips_n; strips_n.clear();
    group_list strips_tc; strips_tc.clear();
    string_list strip_materials; strip_materials.clear();

    int index;
    int_list pt_v, tri_v, strip_v;
    int_list pt_n, tri_n, strip_n;
    int_list tri_tc, strip_tc;

    // calculate "the" normal for this airport
    SGVec3f vnt = SGVec3f::fromGeod( base_poly.GetNode(0, 0) );
    vnt = normalize(vnt);

    GENAPT_LOG(SG_GENERAL, SG_INFO, "Adding runway nodes and normals");
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k ) 
    {
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        std::string material = rwy_polys[k].GetMaterial();
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << rwy_polys[k].Triangles());
        for ( unsigned int i = 0; i < rwy_polys[k].Triangles(); ++i )
        {
            tri_v.clear();
            tri_n.clear();
            tri_tc.clear();
            // let's go out on a limb, and assume triangles have 3 points..
            for ( int j = 0; j < 3; ++j )
            {
                index = nodes.add( rwy_polys[k].GetTriNode( i, j ) );
                tri_v.push_back( index );

        		// use 'the' normal
        		index = normals.add( vnt );
        		tri_n.push_back( index );

        		index = texcoords.add( rwy_polys[k].GetTriTexCoord(i,j) );
        		tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    GENAPT_LOG(SG_GENERAL, SG_INFO, "Adding pavement nodes and normals");
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k ) 
    {
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        std::string material = pvmt_polys[k].GetMaterial();
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << pvmt_polys[k].Triangles());
    	for ( unsigned int i = 0; i < pvmt_polys[k].Triangles(); ++i ) 
        {
    	    tri_v.clear();
    	    tri_n.clear();
    	    tri_tc.clear();
    	    for ( int j = 0; j < 3; ++j )
            {
                index = nodes.add( pvmt_polys[k].GetTriNode( i, j ) );
                tri_v.push_back( index );

                // use 'the' normal
                index = normals.add( vnt );
                tri_n.push_back( index );

                index = texcoords.add( pvmt_polys[k].GetTriTexCoord(i,j) );
                tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    GENAPT_LOG(SG_GENERAL, SG_INFO, "Adding line nodes and normals");
    for ( unsigned int k = 0; k < line_polys.size(); ++k ) 
    {
    	GENAPT_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        std::string material = line_polys[k].GetMaterial();
    	GENAPT_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
    	GENAPT_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << line_polys[k].Triangles());
    	for ( unsigned int i = 0; i < line_polys[k].Triangles(); ++i ) 
        {
    	    tri_v.clear();
    	    tri_n.clear();
    	    tri_tc.clear();
    	    for ( int j = 0; j < 3; ++j )
            {
                index = nodes.add( line_polys[k].GetTriNode( i, j ) );
                tri_v.push_back( index );

                // use 'the' normal
                index = normals.add( vnt );
                tri_n.push_back( index );

                index = texcoords.add( line_polys[k].GetTriTexCoord(i,j) );
                tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    // add base points
    std::vector< SGVec2f > base_txs; 
    int_list base_tc;

    GENAPT_LOG(SG_GENERAL, SG_INFO, "Adding base nodes and normals");
    for ( unsigned int i = 0; i < base_poly.Triangles(); ++i )
    {
    	tri_v.clear();
    	tri_n.clear();
    	tri_tc.clear();
    	for ( int j = 0; j < 3; ++j )
        {
    	    index = nodes.add( base_poly.GetTriNode( i, j ) );
    	    tri_v.push_back( index );

    	    index = normals.add( vnt );
    	    tri_n.push_back( index);
    	}
    	tris_v.push_back( tri_v );
    	tris_n.push_back( tri_n );
    	tri_materials.push_back( "Grass" );

    	std::vector < SGGeod > geodNodes = nodes.get_list();

        base_txs.clear();
    	base_txs = sgCalcTexCoords( b, geodNodes, tri_v );

    	base_tc.clear();
    	for ( unsigned int j = 0; j < base_txs.size(); ++j ) 
        {
    	    index = texcoords.add(  base_txs[j] );
    	    base_tc.push_back( index );
    	}
    	tris_tc.push_back( base_tc );
    }

    // on rare occasion, one or more of the divided base points can be
    // missed.  Make sure they are all in the node list so we can
    // build a proper skirt.
    for ( unsigned int i = 0; i < divided_base.Contours(); ++i )
    {
    	for ( unsigned int j = 0; j < divided_base.ContourSize( i ); ++j )
        {
    	    index = nodes.add( divided_base.GetNode(i, j) );
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "added base point " << divided_base.GetNode(i, j) << " at " << index );
    	}
    }

    // Now that we have assembled all the airport geometry nodes into
    // a list, calculate an "average" airport elevation based on all
    // the actual airport node points.  This is more useful than
    // calculating an average over the entire airport surface because
    // it avoids biases introduced from the surrounding area if the
    // airport is located in a bowl or on a hill.

    GENAPT_LOG(SG_GENERAL, SG_DEBUG, " calc average elevation");
    {
        std::vector < SGGeod > dbg = nodes.get_list();

        // dump the node list
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, " node list size is " << dbg.size() );
        for (unsigned int w = 0; w<dbg.size(); w++)
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, " node " << w << " is " << dbg[w] );
        }
    }

    double average = tgAverageElevation( root, elev_src, nodes.get_list() );

    // Now build the fitted airport surface ...

    // calculation min/max coordinates of airport area
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, " calculation min/max coordinates of airport area");

    SGGeod min_deg = SGGeod::fromDeg(9999.0, 9999.0);
    SGGeod max_deg = SGGeod::fromDeg(-9999.0, -9999.0);
    for ( unsigned int j = 0; j < nodes.get_list().size(); ++j )
    {
        SGGeod p = nodes.get_list()[j];
        if ( p.getLongitudeDeg() < min_deg.getLongitudeDeg() )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "new min lon from node " << j << " is " << p.getLongitudeDeg() );
            min_deg.setLongitudeDeg( p.getLongitudeDeg() );
        }
        if ( p.getLongitudeDeg() > max_deg.getLongitudeDeg() )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "new max lon from node " << j << " is " << p.getLongitudeDeg() );
            max_deg.setLongitudeDeg( p.getLongitudeDeg() );
        }
        if ( p.getLatitudeDeg() < min_deg.getLatitudeDeg() )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "new min lat from node " << j << " is " << p.getLatitudeDeg() );
            min_deg.setLatitudeDeg( p.getLatitudeDeg() );
        }
        if ( p.getLatitudeDeg() > max_deg.getLatitudeDeg() )
        {
            GENAPT_LOG(SG_GENERAL, SG_DEBUG, "new max lat from node " << j << " is " << p.getLatitudeDeg() );
            max_deg.setLatitudeDeg( p.getLatitudeDeg() );
        }
    }

    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Before extending for lights: min = " << min_deg << " max = " << max_deg );

    // extend the min/max coordinates of airport area to cover all
    // lights as well

    GENAPT_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : num rwy lights is " << rwy_lights.size() );
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i ) 
    {
        GENAPT_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : rwy light " << i << "has " << rwy_lights[i].ContourSize() << " lights " );

        for ( unsigned int j = 0; j < rwy_lights[i].ContourSize(); ++j )
        {
            SGGeod p = rwy_lights[i].GetNode(j);
            if ( p.getLongitudeDeg() < min_deg.getLongitudeDeg() )
            {
                min_deg.setLongitudeDeg( p.getLongitudeDeg() );
            }
            if ( p.getLongitudeDeg() > max_deg.getLongitudeDeg() )
            {
                max_deg.setLongitudeDeg( p.getLongitudeDeg() );
            }
            if ( p.getLatitudeDeg() < min_deg.getLatitudeDeg() )
            {
                min_deg.setLatitudeDeg( p.getLatitudeDeg() );
            }
            if ( p.getLatitudeDeg() > max_deg.getLatitudeDeg() )
            {
                max_deg.setLatitudeDeg( p.getLatitudeDeg() );
            }
        }
    }

    // Extend the area a bit so we don't have wierd things on the edges
    double dlon = max_deg.getLongitudeDeg() - min_deg.getLongitudeDeg();
    double dlat = max_deg.getLatitudeDeg() - min_deg.getLatitudeDeg();
    min_deg.setLongitudeDeg( min_deg.getLongitudeDeg() - 0.01 * dlon );
    max_deg.setLongitudeDeg( max_deg.getLongitudeDeg() + 0.01 * dlon );
    min_deg.setLatitudeDeg( min_deg.getLatitudeDeg() - 0.01 * dlat );
    max_deg.setLatitudeDeg( max_deg.getLatitudeDeg() + 0.01 * dlat );
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "min = " << min_deg << " max = " << max_deg );

    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Create Apt surface:" );
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, " root: " << root );
    //GENAPT_LOG(SG_GENERAL, SG_DEBUG, " elev: " << elev_src );
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, " min: " << min_deg );
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, " max: " << max_deg );
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, " average: " << average );

    // TODO elevation queries should be performed as member functions of surface
    tgRectangle aptBounds(min_deg, max_deg);
    tgSurface apt_surf( root, elev_src, aptBounds, average, slope_max, slope_eps );
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Airport surface created");

    // add light points
    // pass one, calculate raw elevations from Array
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i ) {
        for ( unsigned int j = 0; j < rwy_lights[i].ContourSize(); j++ ) {
            double light_elevation = calc_elevation( apt_surf, rwy_lights[i].GetNode(j), 0.0 );
            rwy_lights[i].SetElevation(j, light_elevation);
        }
    }

    GENAPT_LOG(SG_GENERAL, SG_INFO, "Done with lighting calc_elevations() num light polys is " << rwy_lights.size() );

    // pass two, for each light group check if we need to lift (based
    // on flag) and do so, then output next structures.
    // for ( unsigned int i = 0; i < rwy_lights.size(); ++i )
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i )
    {
        pt_v.clear();
        pt_n.clear();
        for ( unsigned int j = 0; j < rwy_lights[i].ContourSize(); ++j )
        {
            index = nodes.add( rwy_lights[i].GetPosition(j) );
            pt_v.push_back( index );

            index = normals.add( rwy_lights[i].GetNormal(j) );
            pt_n.push_back( index );
        }
        pts_v.push_back( pt_v );
        pts_n.push_back( pt_n );
        pt_materials.push_back( rwy_lights[i].GetType() );
    }

    // calculate node elevations
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Computing airport node elevations");

    std::vector<SGGeod> geod_nodes = calc_elevations( apt_surf, nodes.get_list(), 0.0 );
    divided_base = calc_elevations( apt_surf, divided_base, 0.0 );

    // calculate wgs84 mapping of nodes
    std::vector<SGVec3d> wgs84_nodes;
    for ( unsigned int i = 0; i < geod_nodes.size(); ++i ) 
    {
        wgs84_nodes.push_back( SGVec3d::fromGeod(geod_nodes[i]) );
    }

    SGSphered d;
    for ( unsigned int i = 0; i < wgs84_nodes.size(); ++i ) 
    {
        d.expandBy(wgs84_nodes[ i ]);
    }

    SGVec3d gbs_center = d.getCenter();
    double gbs_radius = d.getRadius();
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_n; fans_n.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    std::string objpath = root + "/AirportObj";
    std::string name = icao + ".btg";

    SGBinObject obj;
    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals.get_list() );
    obj.set_texcoords( texcoords.get_list() );
    obj.set_pts_v( pts_v );
    obj.set_pts_n( pts_n );
    obj.set_pt_materials( pt_materials );
    obj.set_tris_v( tris_v );
    obj.set_tris_n( tris_n );
    obj.set_tris_tc( tris_tc ); 
    obj.set_tri_materials( tri_materials );
    obj.set_strips_v( strips_v );
    obj.set_strips_n( strips_n );
    obj.set_strips_tc( strips_tc ); 
    obj.set_strip_materials( strip_materials );
    obj.set_fans_v( fans_v );
    obj.set_fans_n( fans_n );
    obj.set_fans_tc( fans_tc );
    obj.set_fan_materials( fan_materials );

    bool result;
    result = obj.write_bin( objpath, name, b );
    if ( !result ) 
    {
        throw sg_exception("error writing file. :-(");
    }

    // write out airport object reference
    write_index( objpath, b, name );

#if 0 // TODO : along with taxiway signs
    // write out tower references
    for ( i = 0; i < (int)tower_nodes.size(); ++i ) 
    {
        write_index_shared( objpath, b, tower_nodes[i],
                            "Models/Airport/tower.xml",
                            0.0 );
    }
#endif

    SGGeod ref_geod;
    // calc elevations and write out windsock references
    GENAPT_LOG(SG_GENERAL, SG_DEBUG, "Computing windsock node elevations");

    for ( unsigned int i = 0; i < windsocks.size(); ++i )
    {
        SGGeod ref_geod = windsocks[i]->GetLoc();
        ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );

        if ( windsocks[i]->IsLit() )
        {
            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/windsock_lit.xml", 0.0 );
        }
        else
        {
            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/windsock.xml", 0.0 );
        }
    }

    // write out beacon references
    for ( unsigned int i = 0; i < beacons.size(); ++i )
    {
        ref_geod = beacons[i]->GetLoc();
        ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );

        write_index_shared( objpath, b, ref_geod,
                            "Models/Airport/beacon.xml",
                            0.0 );
    }

    // write out taxiway signs references
    for ( unsigned int i = 0; i < signs.size(); ++i )
    {
        ref_geod = signs[i]->GetLoc();
        ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );
        write_object_sign( objpath, b, ref_geod,
                            signs[i]->GetDefinition(),
                            signs[i]->GetHeading(),
                            signs[i]->GetSize() );
    }

    // write out water buoys
    for ( unsigned int i = 0; i < waterrunways.size(); ++i )
    {
        tgContour buoys = waterrunways[i]->GetBuoys();

        for ( unsigned int j = 0; j < buoys.GetSize(); ++j )
        {
            ref_geod = buoys.GetNode(j);
            ref_geod.setElevationM( calc_elevation( apt_surf, ref_geod, 0.0 ) );
            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/water_rw_buoy.xml",
                                0.0 );
        }
    }

    std::string holepath = root + "/AirportArea";
    tgChopper chopper( holepath );

    divided_base.SetPreserve3D( true );
    apt_clearing.SetPreserve3D( false );
    apt_clearing.SetTexMethod( TG_TEX_BY_GEODE );

    chopper.Add( divided_base, "Hole" );
    chopper.Add( apt_clearing, "Airport" );
    chopper.Save();
}