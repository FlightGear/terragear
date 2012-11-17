#include "beznode.hxx"
#include "runway.hxx"
#include "helipad.hxx"
#include "airport.hxx"

#include <list>
#include <map>
#include <string>
#include <ctime>

#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/io/sg_binobj.hxx>
#include <simgear/misc/texcoord.hxx>

#include <Polygon/polygon.hxx>
#include <Polygon/texparams.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>
#include <Polygon/chop.hxx>

#include <Geometry/poly_support.hxx>
#include <Geometry/poly_extra.hxx>

#include <Output/output.hxx>

#include "global.hxx"
#include "elevations.hxx"


#include <stdio.h>

string SGLOG_GREEN  = "\033[0;32m";
string SGLOG_NORMAL = "\033[0m";

static std::string my_ctime(time_t& tt)
{
    char buf[256];

    strcpy(buf,ctime(&tt));
    buf[strlen(buf)-1]='\0';
    return std::string( buf );
}

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

    SG_LOG( SG_GENERAL, SG_DEBUG, "Read airport with icao " << icao << ", control tower " << ct << ", and description " << description );
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
static point_list calc_elevations( TGAptSurface &surf,
                                   const point_list& geod_nodes,
                                   double offset )
{
    point_list result = geod_nodes;
    for ( unsigned int i = 0; i < result.size(); ++i ) {
        double elev = surf.query( SGGeod::fromDeg( result[i].lon(), result[i].lat()) );
        result[i].setelev( elev + offset );
    }

    return result;
}

static tgContour calc_elevations( TGAptSurface &surf,
                                  const tgContour& geod_nodes,
                                  double offset )
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

static double calc_elevation( TGAptSurface &surf,
                               const SGGeod& node,
                               double offset )
{
    double elev = surf.query( node );
    elev += offset;

    return elev;
}


// Determine node elevations of each node of a TGPolygon based on the
// provided TGAptSurface.  Offset is added to the final elevation
static tgPolygon calc_elevations( TGAptSurface &surf,
                                  const tgPolygon& poly,
                                  double offset )
{
    tgPolygon result;
    for ( unsigned int i = 0; i < poly.Contours(); ++i ) {
        tgContour contour = poly.GetContour( i );
        tgContour elevated = calc_elevations( surf, contour, offset );

        result.AddContour( elevated );
    }

    return result;
}

void Airport::BuildBtg(const string& root, const string_list& elev_src )
{
    tgcontour_list slivers;
    tgcontour_list line_slivers;

    tgPolygon      apt_base;
    tgPolygon      apt_clearing;

    // runways
    tgpolygon_list rwy_polys;

    // pavements
    tgpolygon_list pvmt_polys;

    // linear features
    tgpolygon_list line_polys;

    // runway lights
    tglightcontour_list rwy_lights;

    Point3D p;

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
    SG_LOG(SG_GENERAL, SG_DEBUG, b.gen_base_path() << "/" << b.gen_index_str());

    // If we are cutting in the linear features, add them first
    if (pavements.size())
    {
        for ( unsigned int i=0; i<pavements.size(); i++ )
        {
            AddFeatures( pavements[i]->GetFeatures() );
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Parse Complete - Runways: " << runways.size() << " Pavements: " << pavements.size() << " Features: " << features.size() << " Taxiways: " << taxiways.size() );

    // Starting to clip the polys (for now - only UNIX builds)
    build_start.stamp();

    // Add the linear features
    tgPolygonInitClipperAccumulator();

    if (features.size())
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << features.size() << " Linear Feature Polys");
        for ( unsigned int i=0; i<features.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build Feature Poly " << i + 1 << " of " << features.size() << " : " << features[i]->GetDescription() );

            features[i]->BuildBtg( line_polys, rwy_lights, make_shapefiles );
        }

        log_time = time(0);
        SG_LOG( SG_GENERAL, SG_ALERT, "Finished building Linear Features for " << icao << " at " << my_ctime(log_time) );
    }

    /* Done with the linear features accumulator */
    tgPolygonFreeClipperAccumulator();

    /* Initialize a new accumulator for the other objects */
    tgPolygonInitClipperAccumulator();

    // Build runways next
    if (runways.size())
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << runways.size() << " Runway Polys");
        for ( unsigned int i=0; i<runways.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build Runway " << i + 1 << " of " << runways.size());
            slivers.clear();

            if ( isDebugRunway(i) ) {
                sprintf( shapefile_name, "runway_%d", i );
            } else {
                strcpy( shapefile_name, "" );
            }
            shapefile = shapefile_name;

            if (boundary.size())
            {
                runways[i]->BuildBtg( rwy_polys, rwy_lights, slivers, shapefile );
            }
            else
            {
                runways[i]->BuildBtg( rwy_polys, rwy_lights, slivers, apt_base, apt_clearing, shapefile );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
        }

        log_time = time(0);
        SG_LOG( SG_GENERAL, SG_ALERT, "Finished building runways for " << icao << " at " << my_ctime(log_time) );
    }

    if (lightobjects.size())
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << lightobjects.size() << " Light Objects ");
        for ( unsigned int i=0; i<lightobjects.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build runway light " << i + 1 << " of " << lightobjects.size());
            lightobjects[i]->BuildBtg( rwy_lights );
        }
    }

    // Build helipads (use runway poly- and texture list for this)
    if (helipads.size())
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << helipads.size() << " Helipad Polys ");
        for ( unsigned int i=0; i<helipads.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build helipad " << i + 1 << " of " << helipads.size());
            slivers.clear();

            if (boundary.size())
            {
                helipads[i]->BuildBtg( rwy_polys, rwy_lights, slivers );
            }
            else
            {
                helipads[i]->BuildBtg( rwy_polys, rwy_lights, slivers, apt_base, apt_clearing );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
        }
    }

    // Build the pavements
    if (pavements.size())
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << pavements.size() << " Pavement Polys ");
        for ( unsigned int i=0; i<pavements.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build Pavement " << i + 1 << " of " << pavements.size() << " : " << pavements[i]->GetDescription());
            slivers.clear();

            if ( isDebugPavement(i) ) {
                sprintf( shapefile_name, "pvmnt_%d", i );
            } else {
                strcpy( shapefile_name, "" );
            }
            shapefile = shapefile_name;

            if (boundary.size())
            {
                pavements[i]->BuildBtg( pvmt_polys, slivers, shapefile );
            }
            else
            {
                pavements[i]->BuildBtg( pvmt_polys, slivers, apt_base, apt_clearing, shapefile );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
            tgPolygon::MergeSlivers( pvmt_polys, slivers );
        }

        log_time = time(0);
        SG_LOG( SG_GENERAL, SG_ALERT, "Finished building Pavements for " << icao << " at " << my_ctime(log_time) );
    }

    // Build the legacy taxiways
    if (taxiways.size())
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << taxiways.size() << " Taxiway Polys ");
        for ( unsigned int i=0; i<taxiways.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build Taxiway " << i + 1 << " of " << taxiways.size());
            slivers.clear();

            if ( isDebugTaxiway(i) ) {
                sprintf( shapefile_name, "taxiway_%d", i );
            } else {
                strcpy( shapefile_name, "" );
            }
            shapefile = shapefile_name;

            if (boundary.size())
            {
                taxiways[i]->BuildBtg( pvmt_polys, rwy_lights, slivers, shapefile );
            }
            else
            {
                taxiways[i]->BuildBtg( pvmt_polys, rwy_lights, slivers, apt_base, apt_clearing, shapefile );
            }

            // Now try to merge any slivers we found
            tgPolygon::MergeSlivers( rwy_polys, slivers );
            tgPolygon::MergeSlivers( pvmt_polys, slivers );
        }
    }

    // Build runway shoulders here
    if ( runways.size() )
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << runways.size() << " Runway Shoulder Polys ");
        for ( unsigned int i=0; i<runways.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build Runway shoulder " << i + 1 << " of " << runways.size());

            if ( runways[i]->GetsShoulder() )
            {
                slivers.clear();

                if (boundary.size())
                {
                    runways[i]->BuildShoulder( rwy_polys, slivers );
                }
                else
                {
                    runways[i]->BuildShoulder( rwy_polys, slivers, apt_base, apt_clearing );
                }

                // Now try to merge any slivers we found
                tgPolygon::MergeSlivers( rwy_polys, slivers );
                tgPolygon::MergeSlivers( pvmt_polys, slivers );
            }
        }
    }

    // Build helipad shoulders here
    if ( helipads.size() )
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << runways.size() << " Helipad Shoulder Polys ");
        for ( unsigned int i=0; i<helipads.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build Helipad shoulder " << i + 1 << " of " << helipads.size());

            if ( helipads[i]->GetsShoulder() )
            {
                slivers.clear();

                if (boundary.size())
                {
                    helipads[i]->BuildShoulder( rwy_polys, slivers );
                }
                else
                {
                    helipads[i]->BuildShoulder( rwy_polys, slivers, apt_base, apt_clearing );
                }

                // Now try to merge any slivers we found
                tgPolygon::MergeSlivers( rwy_polys, slivers );
                tgPolygon::MergeSlivers( pvmt_polys, slivers );
            }
        }
    }

    // build the base and clearing if there's a boundary
    if (boundary.size())
    {
        SG_LOG(SG_GENERAL, SG_INFO, "Build " << boundary.size() << " Boundary Polys ");
        shapefile = "";

        for ( unsigned int i=0; i<boundary.size(); i++ )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Build Userdefined boundary " << i + 1 << " of " << boundary.size());
            boundary[i]->BuildBtg( apt_base, apt_clearing, shapefile );
        }
    }

    if ( apt_base.TotalNodes() == 0 )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "no airport points generated");
    	return;
    }

    tgPolygon filled_base  = tgPolygon::StripHoles( apt_base );
    tgPolygon divided_base = tgPolygon::SplitLongEdges( filled_base, 200.0 );
    tgPolygon base_poly    = tgPolygon::DiffWithAccumulator( divided_base );

    build_end.stamp();
    build_time = build_end - build_start;

    cleanup_start.stamp();

    // add segments to polygons to remove any possible "T"
    // intersections
    TGTriNodes tmp_pvmt_nodes;
    TGTriNodes tmp_feat_nodes;

    // build temporary node list from runways...
    SG_LOG(SG_GENERAL, SG_INFO, "Build Node List " );
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k ) 
    {
    	tgPolygon poly = rwy_polys[k];
    	for ( unsigned int i = 0; i < poly.Contours(); ++i )
        {
    	    for ( unsigned int j = 0; j < poly.ContourSize( i ); ++j )
            {
                tmp_pvmt_nodes.unique_add( Point3D::fromSGGeod( poly.GetNode(i, j) ) );
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
        		tmp_pvmt_nodes.unique_add( Point3D::fromSGGeod( poly.GetNode(i, j) ) );
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
        		tmp_feat_nodes.unique_add( Point3D::fromSGGeod( poly.GetNode(i, j) ) );
    	    }
    	}
    }

    // and the base
    for ( unsigned int i = 0; i < base_poly.Contours(); ++i )
    {
    	for ( unsigned int j = 0; j < base_poly.ContourSize( i ); ++j )
        {
    	    tmp_pvmt_nodes.unique_add( Point3D::fromSGGeod( base_poly.GetNode(i, j) ) );
    	}
    }

    // the divided base could contain points not found in base_poly,
    // so we should add them because the skirt needs them.
    for ( unsigned int i = 0; i < divided_base.Contours(); ++i )
    {
        for ( unsigned int j = 0; j < divided_base.ContourSize( i ); ++j )
        {
            tmp_pvmt_nodes.unique_add( Point3D::fromSGGeod( divided_base.GetNode(i, j) ) );
        }
    }

    log_time = time(0);
    SG_LOG( SG_GENERAL, SG_ALERT, "Finished collecting nodes for " << icao << " at " << my_ctime(log_time) );

    // second pass : runways
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k ) 
    {
        tgPolygon poly = rwy_polys[k];
        poly = tgPolygon::AddColinearNodes( poly, tmp_pvmt_nodes );
        SG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.TotalNodes());
        rwy_polys[k] = poly;
    }

    // second pass : and pavements
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k ) 
    {
        tgPolygon poly = pvmt_polys[k];
        poly = tgPolygon::AddColinearNodes( poly, tmp_pvmt_nodes );
        SG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.TotalNodes());
        pvmt_polys[k] = poly;
    }

    // second pass : and lines
    for ( unsigned int k = 0; k < line_polys.size(); ++k ) 
    {
        tgPolygon poly = line_polys[k];
        poly = tgPolygon::AddColinearNodes( poly, tmp_feat_nodes );
        SG_LOG(SG_GENERAL, SG_DEBUG, "total size after add nodes = " << poly.TotalNodes());
        line_polys[k] = poly;
    }

    log_time = time(0);
    SG_LOG( SG_GENERAL, SG_ALERT, "Finished adding intermediate nodes for " << icao << " at " << my_ctime(log_time) );

    for ( unsigned int k = 0; k < line_polys.size(); ++k )
    {
        tgPolygon poly = line_polys[k];

        poly = tgPolygon::RemoveCycles( poly );
        poly = tgPolygon::RemoveDups( poly );
        poly = tgPolygon::RemoveBadContours( poly );
        poly = tgPolygon::Simplify( poly );
        poly = tgPolygon::RemoveTinyContours( poly );
        poly = tgPolygon::RemoveSpikes( poly );
        poly = tgPolygon::RemoveDups( poly );
        poly = tgPolygon::RemoveBadContours( poly );
        poly = tgPolygon::RemoveTinyContours( poly );

        line_polys[k] = poly;
    }

    log_time = time(0);
    SG_LOG( SG_GENERAL, SG_ALERT, "Finished cleaning polys for " << icao << " at " << my_ctime(log_time) );

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
        SG_LOG(SG_GENERAL, SG_INFO, "Tesselating " << rwy_polys.size() << " Runway Polys " );
        for ( unsigned int i = 0; i < rwy_polys.size(); ++i )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating runway poly = " << i + 1 << " of " << rwy_polys.size() );

            SG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << rwy_polys[i].Contours() << " total points before = " << rwy_polys[i].TotalNodes());
            rwy_polys[i].Tesselate();
            SG_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << rwy_polys[i].Triangles());
            rwy_polys[i].Texture();
        }
    }

    if ( pvmt_polys.size() )
    {
        // tesselate the polygons and prepair them for final output
        SG_LOG(SG_GENERAL, SG_INFO, "Tesselating " << pvmt_polys.size() << " Pavement Polys " );
        for ( unsigned int i = 0; i < pvmt_polys.size(); ++i )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating pavement poly = " << i + 1 << " of " << pvmt_polys.size() );

            SG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << pvmt_polys[i].Contours() << " total points before = " << pvmt_polys[i].TotalNodes());
            pvmt_polys[i].Tesselate();
            SG_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << pvmt_polys[i].Triangles());
            pvmt_polys[i].Texture();
        }
    }

    if ( line_polys.size() )
    {
        // tesselate the polygons and prepair them for final output
        SG_LOG(SG_GENERAL, SG_INFO, "Tesselating " << line_polys.size() << " Linear Feature Polys " );
        for ( unsigned int i = 0; i < line_polys.size(); ++i )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Tesselating line poly = " << i + 1 << " of " << line_polys.size() );

            SG_LOG(SG_GENERAL, SG_DEBUG, "contours before " << line_polys[i].Contours() << " total points before = " << line_polys[i].TotalNodes());
            line_polys[i].Tesselate();
            SG_LOG(SG_GENERAL, SG_DEBUG, "triangles after = " << line_polys[i].Triangles());
            line_polys[i].Texture();
        }
    }

    /* before tessellating the base, make sure there are no
       intersecting contours */
    base_poly = tgPolygon::Simplify( base_poly );

    SG_LOG(SG_GENERAL, SG_INFO, "Tesselating base poly ");
    base_poly.Tesselate();
    SG_LOG(SG_GENERAL, SG_INFO, "Tesselating base poly - done : Triangles = " << base_poly.Triangles());
    // should we texture base here?

    triangulation_end.stamp();
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

    TGTriNodes nodes, normals, texcoords;
    nodes.clear();
    normals.clear();
    texcoords.clear();

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
    SGVec3d vnt = SGVec3d::fromGeod( base_poly.GetNode(0, 0) );
    vnt = normalize(vnt);
    Point3D vn = Point3D::fromSGVec3(vnt);

    SG_LOG(SG_GENERAL, SG_INFO, "Adding runway nodes and normals");
    for ( unsigned int k = 0; k < rwy_polys.size(); ++k ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        string material = rwy_polys[k].GetMaterial();
        SG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
        SG_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << rwy_polys[k].Triangles());
        for ( unsigned int i = 0; i < rwy_polys[k].Triangles(); ++i )
        {
            tri_v.clear();
            tri_n.clear();
            tri_tc.clear();
            // let's go out on a limb, and assume triangles have 3 points..
            for ( int j = 0; j < 3; ++j )
            {
                p = Point3D::fromSGGeod( rwy_polys[k].GetTriNode( i, j ) );
                index = nodes.unique_add( p );
                SG_LOG(SG_GENERAL, SG_DEBUG, "added rwy point " << p << " at " << index );
                tri_v.push_back( index );

        		// use 'the' normal
        		index = normals.unique_add( vn );
        		tri_n.push_back( index );

        		Point3D tc = Point3D::fromSGVec2( rwy_polys[k].GetTriTexCoord( i, j ) );
        		index = texcoords.unique_add( tc );
        		tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Adding pavement nodes and normals");
    for ( unsigned int k = 0; k < pvmt_polys.size(); ++k ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
        string material = pvmt_polys[k].GetMaterial();
        SG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
        SG_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << pvmt_polys[k].Triangles());
    	for ( unsigned int i = 0; i < pvmt_polys[k].Triangles(); ++i ) 
        {
    	    tri_v.clear();
    	    tri_n.clear();
    	    tri_tc.clear();
    	    for ( int j = 0; j < 3; ++j )
            {
                p = Point3D::fromSGGeod( pvmt_polys[k].GetTriNode( i, j ) );
                index = nodes.unique_add( p );
                SG_LOG(SG_GENERAL, SG_DEBUG, "added pvmt point " << p << " at " << index );
                tri_v.push_back( index );

                // use 'the' normal
                index = normals.unique_add( vn );
                tri_n.push_back( index );

                Point3D tc = Point3D::fromSGVec2( pvmt_polys[k].GetTriTexCoord( i, j ) );
                index = texcoords.unique_add( tc );
                tri_tc.push_back( index );
    	    }
    	    tris_v.push_back( tri_v );
    	    tris_n.push_back( tri_n );
    	    tris_tc.push_back( tri_tc );
    	    tri_materials.push_back( material );
    	}
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Adding line nodes and normals");
    for ( unsigned int k = 0; k < line_polys.size(); ++k ) 
    {
    	SG_LOG(SG_GENERAL, SG_DEBUG, "tri " << k);
    	string material = line_polys[k].GetMaterial();
    	SG_LOG(SG_GENERAL, SG_DEBUG, "material = " << material);
    	SG_LOG(SG_GENERAL, SG_DEBUG, "triangles = " << line_polys[k].Triangles());
    	for ( unsigned int i = 0; i < line_polys[k].Triangles(); ++i ) 
        {
    	    tri_v.clear();
    	    tri_n.clear();
    	    tri_tc.clear();
    	    for ( int j = 0; j < 3; ++j )
            {
                p = Point3D::fromSGGeod( line_polys[k].GetTriNode( i, j ) );
                index = nodes.unique_add( p );
                SG_LOG(SG_GENERAL, SG_DEBUG, "added line point " << p << " at " << index );
                tri_v.push_back( index );

                // use 'the' normal
                index = normals.unique_add( vn );
                tri_n.push_back( index );

                Point3D tc = Point3D::fromSGVec2( line_polys[k].GetTriTexCoord( i, j ) );
                index = texcoords.unique_add( tc );
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

    SG_LOG(SG_GENERAL, SG_INFO, "Adding base nodes and normals");
    for ( unsigned int i = 0; i < base_poly.Triangles(); ++i )
    {
    	tri_v.clear();
    	tri_n.clear();
    	tri_tc.clear();
    	for ( int j = 0; j < 3; ++j )
        {
    	    p = Point3D::fromSGGeod( base_poly.GetTriNode( i, j ) );
    	    index = nodes.unique_add( p );
            SG_LOG(SG_GENERAL, SG_DEBUG, "added base point " << p << " at " << index );
    	    tri_v.push_back( index );

    	    index = normals.unique_add( vn );
    	    tri_n.push_back( index);
    	}
    	tris_v.push_back( tri_v );
    	tris_n.push_back( tri_n );
    	tri_materials.push_back( "Grass" );

    	std::vector < SGGeod > geodNodes;
    	for ( unsigned int j = 0; j < nodes.get_node_list().size(); j++ ) 
        {
    	    Point3D node = nodes.get_node_list()[j];
    	    geodNodes.push_back( SGGeod::fromDegM( node.x(), node.y(), node.z() ) );
    	}
	    base_txs.clear();
    	base_txs = sgCalcTexCoords( b, geodNodes, tri_v );

    	base_tc.clear();
    	for ( unsigned int j = 0; j < base_txs.size(); ++j ) 
        {
    	    SGVec2f tc = base_txs[j];
    	    // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
    	    index = texcoords.simple_add( Point3D( tc.x(), tc.y(), 0 ) );
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
    	    index = nodes.unique_add( Point3D::fromSGGeod( divided_base.GetNode(i, j) ) );
            SG_LOG(SG_GENERAL, SG_DEBUG, "added base point " << divided_base.GetNode(i, j) << " at " << index );
    	}
    }

    // Now that we have assembled all the airport geometry nodes into
    // a list, calculate an "average" airport elevation based on all
    // the actual airport node points.  This is more useful than
    // calculating an average over the entire airport surface because
    // it avoids biases introduced from the surrounding area if the
    // airport is located in a bowl or on a hill.

    SG_LOG(SG_GENERAL, SG_DEBUG, " calc average elevation");
    {
        point_list dbg = nodes.get_node_list();

        // dump the node list
        SG_LOG(SG_GENERAL, SG_DEBUG, " node list size is " << dbg.size() );
        for (unsigned int w = 0; w<dbg.size(); w++)
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, " node " << w << " is " << dbg[w] );            
        }
    }

    double average = tgAverageElevation( root, elev_src, nodes.get_node_list() );

    // Now build the fitted airport surface ...

    // calculation min/max coordinates of airport area
    SG_LOG(SG_GENERAL, SG_DEBUG, " calculation min/max coordinates of airport area");

    SGGeod min_deg = SGGeod::fromDeg(9999.0, 9999.0);
    SGGeod max_deg = SGGeod::fromDeg(-9999.0, -9999.0);
    for ( unsigned int j = 0; j < nodes.get_node_list().size(); ++j ) 
    {
        Point3D p = nodes.get_node_list()[j];
        if ( p.lon() < min_deg.getLongitudeDeg() )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "new min lon from node " << j << " is " << p.lon() );
            min_deg.setLongitudeDeg( p.lon() );
        }
        if ( p.lon() > max_deg.getLongitudeDeg() )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "new max lon from node " << j << " is " << p.lon() );
            max_deg.setLongitudeDeg( p.lon() );
        }
        if ( p.lat() < min_deg.getLatitudeDeg() )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "new min lat from node " << j << " is " << p.lat() );
            min_deg.setLatitudeDeg( p.lat() );
        }
        if ( p.lat() > max_deg.getLatitudeDeg() )
        {
            SG_LOG(SG_GENERAL, SG_DEBUG, "new max lat from node " << j << " is " << p.lat() );
            max_deg.setLatitudeDeg( p.lat() );
        }
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Before extending for lights: min = " << min_deg << " max = " << max_deg );

    // extend the min/max coordinates of airport area to cover all
    // lights as well

    SG_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : num rwy lights is " << rwy_lights.size() );
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i ) 
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, " extend the min/max coordinates of airport area to cover all lights as well : rwy light " << i << "has " << rwy_lights[i].ContourSize() << " lights " );

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
    SG_LOG(SG_GENERAL, SG_DEBUG, "min = " << min_deg << " max = " << max_deg );

    SG_LOG(SG_GENERAL, SG_DEBUG, "Create Apt surface:" );
    SG_LOG(SG_GENERAL, SG_DEBUG, " root: " << root );
    //SG_LOG(SG_GENERAL, SG_DEBUG, " elev: " << elev_src );
    SG_LOG(SG_GENERAL, SG_DEBUG, " min: " << min_deg );
    SG_LOG(SG_GENERAL, SG_DEBUG, " max: " << max_deg );
    SG_LOG(SG_GENERAL, SG_DEBUG, " average: " << average );
    
    tg::Rectangle aptBounds(min_deg, max_deg);

    TGAptSurface apt_surf( root, elev_src, aptBounds, average );
    SG_LOG(SG_GENERAL, SG_DEBUG, "Airport surface created");

    // calculate node elevations
    SG_LOG(SG_GENERAL, SG_DEBUG, "Computing airport node elevations");
    point_list geod_nodes = calc_elevations( apt_surf,
                                             nodes.get_node_list(),
                                             0.0 );
    divided_base = calc_elevations( apt_surf, divided_base, 0.0 );

    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with base calc_elevations()");


    // add base skirt (to hide potential cracks)
    // this has to happen after we've calculated the node elevations
    // but before we convert to wgs84 coordinates

    int uindex, lindex;

    for ( unsigned int i = 0; i < divided_base.Contours(); ++i )
    {
        strip_v.clear();
        strip_n.clear();
        strip_tc.clear();

        // prime the pump ...
        p = Point3D::fromSGGeod( divided_base.GetNode( i, 0 ) );
        uindex = nodes.find( p );
        if ( uindex >= 0 )
        {
            Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
            SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
            lindex = nodes.simple_add( lower );
            geod_nodes.push_back( lower );
            strip_v.push_back( uindex );
            strip_v.push_back( lindex );

            // use 'the' normal.  We are pushing on two nodes so we
            // need to push on two normals.
            index = normals.unique_add( vn );
            strip_n.push_back( index );
            strip_n.push_back( index );
        }
        else
        {
            string message = "Ooops missing node when building skirt (in init)";
            SG_LOG( SG_GENERAL, SG_ALERT, message << " " << p );
            throw sg_exception( message );
        }

        // loop through the list
        for ( unsigned int j = 1; j < divided_base.ContourSize(i); ++j )
        {
            p = Point3D::fromSGGeod( divided_base.GetNode( i, j ) );
            uindex = nodes.find( p );
            if ( uindex >= 0 )
            {
                Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
                SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
                lindex = nodes.simple_add( lower );
                geod_nodes.push_back( lower );
                strip_v.push_back( uindex );
                strip_v.push_back( lindex );

                index = normals.unique_add( vn );
                strip_n.push_back( index );
                strip_n.push_back( index );
            }
            else
            {
                string message = "Ooops missing node when building skirt (in loop)";
                SG_LOG( SG_GENERAL, SG_ALERT, message << " " << p );
                throw sg_exception( message );
            }
        }

        // close off the loop
        p = Point3D::fromSGGeod( divided_base.GetNode( i, 0 ) );
        uindex = nodes.find( p );
        if ( uindex >= 0 )
        {
            Point3D lower = geod_nodes[uindex] - Point3D(0, 0, 20);
            SG_LOG(SG_GENERAL, SG_DEBUG, geod_nodes[uindex] << " <-> " << lower);
            lindex = nodes.simple_add( lower );
            geod_nodes.push_back( lower );
            strip_v.push_back( uindex );
            strip_v.push_back( lindex );

            index = normals.unique_add( vn );
            strip_n.push_back( index );
            strip_n.push_back( index );
        }
        else
        {
            string message = "Ooops missing node when building skirt (at end)";
            SG_LOG( SG_GENERAL, SG_ALERT, message << " " << p );
            throw sg_exception( message );
        }

        strips_v.push_back( strip_v );
        strips_n.push_back( strip_n );
        strip_materials.push_back( "Grass" );

        std::vector < SGGeod > geodNodes;
        for ( unsigned int j = 0; j < nodes.get_node_list().size(); j++ )
        {
            Point3D node = nodes.get_node_list()[j];
            geodNodes.push_back( SGGeod::fromDegM( node.x(), node.y(), node.z() ) );
        }
        base_txs.clear();
        base_txs = sgCalcTexCoords( b, geodNodes, strip_v );

        base_tc.clear();
        for ( unsigned int j = 0; j < base_txs.size(); ++j )
        {
            SGVec2f tc = base_txs[j];
            // SG_LOG(SG_GENERAL, SG_DEBUG, "base_tc = " << tc);
            index = texcoords.simple_add( Point3D( tc.x(), tc.y(), 0 ) );
            base_tc.push_back( index );
        }
        strips_tc.push_back( base_tc );
    }

    // add light points
    superpoly_list tmp_light_list; 
    tmp_light_list.clear();
    typedef std::map < string, double, std::less<string> > elev_map_type;
    typedef elev_map_type::const_iterator const_elev_map_iterator;
    elev_map_type elevation_map;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Computing runway/approach lighting elevations");

    // pass one, calculate raw elevations from Array
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i ) 
    {
        TGTriNodes light_nodes;
        light_nodes.clear();
        point_list lights_v = rwy_lights[i].TempGetPosListAsPoint3D();
        for ( unsigned int j = 0; j < lights_v.size(); ++j ) 
        {
            p = lights_v[j];
            index = light_nodes.simple_add( p );
        }

        // calculate light node elevations
        point_list geod_light_nodes = calc_elevations( apt_surf, light_nodes.get_node_list(), 0.0 );
        TGPolygon p;
        p.add_contour( geod_light_nodes, 0 );
        TGSuperPoly s;
        s.set_poly( p );
        tmp_light_list.push_back( s );

        string flag = rwy_lights[i].GetFlag();
        if ( flag != (string)"" ) 
        {
            double max = -9999;
            const_elev_map_iterator it = elevation_map.find( flag );
            if ( it != elevation_map.end() ) 
            {
                max = elevation_map[flag];
            }
            for ( unsigned int j = 0; j < geod_light_nodes.size(); ++j ) 
            {
                if ( geod_light_nodes[j].z() > max ) 
                {
                    max = geod_light_nodes[j].z();
                }
            }
            elevation_map[flag] = max;
            SG_LOG( SG_GENERAL, SG_DEBUG, flag << " max = " << max );
        }
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with lighting calc_elevations() num light polys is " << rwy_lights.size() );

    // pass two, for each light group check if we need to lift (based
    // on flag) and do so, then output next structures.
    for ( unsigned int i = 0; i < rwy_lights.size(); ++i ) 
    {
        // tmp_light_list is a parallel structure to rwy_lights
        point_list geod_light_nodes = tmp_light_list[i].get_poly().get_contour(0);
        
        // this is a little round about, but what we want to calculate the
        // light node elevations as ground + an offset so we do them
        // seperately, then we add them back into nodes to get the index
        // out, but also add them to geod_nodes to maintain consistancy
        // between these two lists.
        point_list light_normals = rwy_lights[i].TempGetNormalListAsPoint3D();
        pt_v.clear();
        pt_n.clear();
        for ( unsigned int j = 0; j < geod_light_nodes.size(); ++j ) 
        {
            p = geod_light_nodes[j];
            index = nodes.simple_add( p );
            pt_v.push_back( index );
            geod_nodes.push_back( p );

            index = normals.unique_add( light_normals[j] );
            pt_n.push_back( index );
        }
        pts_v.push_back( pt_v );
        pts_n.push_back( pt_n );
        pt_materials.push_back( rwy_lights[i].GetType() );
    }

    // calculate wgs84 mapping of nodes
    std::vector< SGVec3d > wgs84_nodes;
    for ( unsigned int i = 0; i < geod_nodes.size(); ++i ) 
    {
        SGGeod geod = SGGeod::fromDegM( geod_nodes[i].x(), geod_nodes[i].y(), geod_nodes[i].z() );
    	SG_LOG(SG_GENERAL, SG_DEBUG, "geod pt = " << geod_nodes[i] );
        SGVec3d cart = SGVec3d::fromGeod(geod);
        SG_LOG(SG_GENERAL, SG_DEBUG, "  cart pt = " << cart );
    	wgs84_nodes.push_back( cart );
    }
    SGSphered d;
    for ( unsigned int i = 0; i < wgs84_nodes.size(); ++i ) 
    {
        d.expandBy(wgs84_nodes[ i ]);
    }
    
    SGVec3d gbs_center = d.getCenter();
    double gbs_radius = d.getRadius();
    SG_LOG(SG_GENERAL, SG_DEBUG, "gbs center = " << gbs_center);
    SG_LOG(SG_GENERAL, SG_DEBUG, "Done with wgs84 node mapping");
    SG_LOG(SG_GENERAL, SG_DEBUG, "  center = " << gbs_center << " radius = " << gbs_radius );

    // null structures
    group_list fans_v; fans_v.clear();
    group_list fans_n; fans_n.clear();
    group_list fans_tc; fans_tc.clear();
    string_list fan_materials; fan_materials.clear();

    string objpath = root + "/AirportObj";
    string name = icao + ".btg";

    std::vector< SGVec3f > normals_3f;
    for ( unsigned int i = 0; i < normals.get_node_list().size(); i++ ) 
    {
        Point3D node = normals.get_node_list()[i];
        normals_3f.push_back( node.toSGVec3f() );
    }

    std::vector< SGVec2f > texcoords_2f;
    for ( unsigned int i = 0; i < texcoords.get_node_list().size(); i++ ) 
    {
        Point3D node = texcoords.get_node_list()[i];
        texcoords_2f.push_back( node.toSGVec2f() );
    }

    SGBinObject obj;

    obj.set_gbs_center( gbs_center );
    obj.set_gbs_radius( gbs_radius );
    obj.set_wgs84_nodes( wgs84_nodes );
    obj.set_normals( normals_3f );
    obj.set_texcoords( texcoords_2f );
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
    SG_LOG(SG_GENERAL, SG_DEBUG, "Computing windsock node elevations");

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

    string holepath = root + "/AirportArea";

    tgPolygon::Chop( divided_base, holepath, "Hole", false, true );
    tgPolygon::Chop( apt_clearing, holepath, "Airport", false, false );

    //SG_LOG( SG_GENERAL, SG_ALERT, SGLOG_GREEN << "\nSUCCESS generating " << icao << SGLOG_NORMAL << "\n" );
}
