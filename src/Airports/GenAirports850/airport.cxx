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
#include <terragear/tg_surface.hxx>
#include <terragear/tg_chopper.hxx>
#include <terragear/tg_rectangle.hxx>
#include <terragear/tg_unique_geod.hxx>
#include <terragear/tg_unique_vec3f.hxx>
#include <terragear/tg_unique_vec2f.hxx>
#include <terragear/tg_shapefile.hxx>

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

    TG_LOG( SG_GENERAL, SG_DEBUG, "Read airport with icao " << icao << ", control tower " << ct << ", and description " << description );
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

void Airport::BuildBtg(const std::string& root, const string_list& elev_src )
{
    TG_LOG(SG_GENERAL, SG_ALERT, "BUILDBTG");

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

    // parse main airport information
    double apt_lon = 0.0, apt_lat = 0.0;

    // Find the average of all the runway and heliport long / lats
    int num_samples = 0;
    TG_LOG(SG_GENERAL, SG_ALERT, "num runways is " << runways.size() << " num heipads is " << helipads.size() );

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

    if ( num_samples )
    {
        apt_lon = apt_lon / (double)num_samples;
        apt_lat = apt_lat / (double)num_samples;
    }
    else
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "AIRPORT HAS NO RUNWAYS/HELIPADS");
        return;
    }

    TG_LOG(SG_GENERAL, SG_ALERT, " long, lat is " << apt_lon << ", " << apt_lat );
    SGBucket b( apt_lon, apt_lat );

    TG_LOG(SG_GENERAL, SG_INFO, "Parse Complete - Runways: " << runways.size() << " Pavements: " << pavements.size() << " Features: " << features.size() << " Taxiways: " << taxiways.size() );

    // Airport building Steps
    // 1: Build the base polygons
    BuildBase();

    TG_LOG(SG_GENERAL, SG_INFO, "ClipBase" );

    // 2: Clip the polys in priority order
    ClipBase();

    TG_LOG(SG_GENERAL, SG_INFO, "spacialquery" );

    // 3: Clean the polys
    base_nodes.init_spacial_query();

    TG_LOG(SG_GENERAL, SG_INFO, "CleanBase" );

    CleanBase();

    TG_LOG(SG_GENERAL, SG_INFO, "TesselateBase" );

    // 4: Teseelate Base polys
    TesselateBase();

    TG_LOG(SG_GENERAL, SG_INFO, "LookupIndexes" );

    LookupBaseIndexes();

    TG_LOG(SG_GENERAL, SG_INFO, "TextureBase" );

    // 5: Texture Base polys
    TextureBase();

    TG_LOG(SG_GENERAL, SG_INFO, "CalcElevations" );

    // 6: calculate height
    CalcBaseElevations(root, elev_src);

    // save Base
    TG_LOG(SG_GENERAL, SG_INFO, "Write Base" );
    WriteBaseOutput( root, b );
    
    // 9: Build the linear feature polygons
    BuildFeatures();

    ClipFeatures();
    
    // 3: Clean the polys
    feat_nodes.init_spacial_query();
    
    TG_LOG(SG_GENERAL, SG_INFO, "CleanBase" );
    
    CleanFeatures();
    
    // we need to add nodes that intersect with the 
    // base we will drape with
    IntersectFeaturesWithBase();
    
    // 4: Teseelate Base polys
    TesselateFeatures();
    
    TG_LOG(SG_GENERAL, SG_INFO, "LookupIndexes" );
    
    LookupFeatureIndexes();
    
    TG_LOG(SG_GENERAL, SG_INFO, "TextureBase" );
    
    // 5: Texture Base polys
    TextureFeatures();
        
    TG_LOG(SG_GENERAL, SG_INFO, "CalcElevations" );
    // 6: calculate height
    CalcFeatureElevations();
    
    // save Base
    TG_LOG(SG_GENERAL, SG_INFO, "Write Base" );
    WriteFeatureOutput( root, b );
    
    // Build Lights
    BuildLights();
    
    WriteLightsOutput( root, b );
}