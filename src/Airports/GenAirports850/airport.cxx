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

    for ( unsigned int i=0; i<9; i++ ) {
        normal_lf_ig[i] = NULL;
    }
    for ( unsigned int i=0; i<3; i++ ) {
        white_lf_ig[i] = NULL;
    }
    for ( unsigned int i=0; i<9; i++ ) {
        black_lf_ig[i] = NULL;
    }
    rm_ig = NULL;
    
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
    
    if ( done ) {
        char ig_ds[64];
        for ( unsigned int i=0; i<9; i++ ) {
            sprintf(ig_ds, "%s_normal_%d", icao.c_str(), i ); 
            normal_lf_ig[i] = new tgIntersectionGenerator(ig_ds, LinearFeature::GetTextureInfo );
        }
        for ( unsigned int i=0; i<3; i++ ) {
            sprintf(ig_ds, "%s_white_%d", icao.c_str(), i ); 
            white_lf_ig[i] = new tgIntersectionGenerator(ig_ds, LinearFeature::GetTextureInfo );;
        }
        for ( unsigned int i=0; i<9; i++ ) {
            sprintf(ig_ds, "%s_black_%d", icao.c_str(), i ); 
            black_lf_ig[i] = new tgIntersectionGenerator(ig_ds, LinearFeature::GetTextureInfo );;
        }        
        sprintf(ig_ds, "%s_runways", icao.c_str() ); 
        rm_ig = new tgIntersectionGenerator(ig_ds, LinearFeature::GetTextureInfo );
    }

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
    
    for ( unsigned int i=0; i<9; i++ ) {
        if ( normal_lf_ig[i] ) {
            delete normal_lf_ig[i];
        }
    }
    for ( unsigned int i=0; i<3; i++ ) {
        if ( white_lf_ig[i] ) {
            delete white_lf_ig[i];
        }
    }
    for ( unsigned int i=0; i<9; i++ ) {
        if ( black_lf_ig[i] ) {
            delete black_lf_ig[i];
        }
    }        
    
    if ( rm_ig ) {
        delete rm_ig;
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
    TG_LOG(SG_GENERAL, SG_INFO, "Build Features" );
    BuildFeatures();

    TG_LOG(SG_GENERAL, SG_INFO, "Clip Features" );
    ClipFeatures();
    
    // 3: Clean the polys
    //feat_nodes.init_spacial_query();
    
    //TG_LOG(SG_GENERAL, SG_INFO, "CleanFeatures" );
    
    //CleanFeatures();
    
    TG_LOG(SG_GENERAL, SG_INFO, "IntersectFeaturesWithBase" );
    // we need to add nodes that intersect with the 
    // base we will drape with
    IntersectFeaturesWithBase();
    
    // 4: Teseelate Base polys
    TG_LOG(SG_GENERAL, SG_INFO, "TesselateFeatures" );
    TesselateFeatures();
    
    TG_LOG(SG_GENERAL, SG_INFO, "LookupIndexes" );
    
    LookupFeatureIndexes();
    
    TG_LOG(SG_GENERAL, SG_INFO, "TextureFeatures" );
    
    // 5: Texture Base polys
    TextureFeatures();
        
    TG_LOG(SG_GENERAL, SG_INFO, "CalcElevations" );
    // 6: calculate height
    CalcFeatureElevations();
    
    // save Base
    TG_LOG(SG_GENERAL, SG_INFO, "Write Features" );
    WriteFeatureOutput( root, b );
    
    // Build Lights
    TG_LOG(SG_GENERAL, SG_INFO, "Build Lights" );
    BuildLights();
    
    TG_LOG(SG_GENERAL, SG_INFO, "Write Lights" );
    WriteLightsOutput( root, b );
    
    // Generate Objects
    TG_LOG(SG_GENERAL, SG_INFO, "Write Objects" );    
    WriteObjects( root, b );
}

void Airport::WriteObjects( const std::string& root, const SGBucket& b )
{
    SGGeod ref_geod;
    std::string objpath = root + "/AirportObj";
    

    // TODO : tower nodes?
#if 0 
    // write out tower references
    for ( i = 0; i < (int)tower_nodes.size(); ++i )
    {
        write_index_shared( objpath, b, tower_nodes[i],
                            "Models/Airport/tower.xml",
                            0.0 );
    }
#endif

    // calc elevations and write out windsock references
    TG_LOG(SG_GENERAL, SG_INFO, "Computing elevations for " << windsocks.size() << " windsocks"); 
    for ( unsigned int i = 0; i < windsocks.size(); ++i )
    {
        ref_geod = windsocks[i]->GetLoc();
        ref_geod.setElevationM( base_surf.query( ref_geod ) );
        
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
        ref_geod.setElevationM( base_surf.query( ref_geod ) );
        
        write_index_shared( objpath, b, ref_geod,
                            "Models/Airport/beacon.xml",
                            0.0 );
    }
    
    // write out taxiway signs references
    TG_LOG(SG_GENERAL, SG_INFO, "Computing elevations for " << signs.size() << " signs"); 
    for ( unsigned int i = 0; i < signs.size(); ++i )
    {
        ref_geod = signs[i]->GetLoc();
        ref_geod.setElevationM( base_surf.query( ref_geod ) );
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
            ref_geod.setElevationM( base_surf.query( ref_geod ) );
            write_index_shared( objpath, b, ref_geod,
                                "Models/Airport/water_rw_buoy.xml",
                                0.0 );
        }
    }
}