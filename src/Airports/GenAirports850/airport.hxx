#ifndef _AIRPORT_H_
#define _AIRPORT_H_

#include <simgear/timing/timestamp.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/threads/SGThread.hxx>

#include <terragear/tg_array.hxx>
#include <terragear/tg_areas.hxx>
#include <terragear/tg_nodes.hxx>

#include "runway.hxx"
#include "object.hxx"
#include "helipad.hxx"
#include "taxiway.hxx"
#include "closedpoly.hxx"
#include "linearfeature.hxx"
#include "linked_objects.hxx"
#include "debug.hxx"

// Airport areas are hardcoded - no priority config to deal with
#define AIRPORT_AREA_RUNWAY             (0)
#define AIRPORT_AREA_HELIPAD            (1)
#define AIRPORT_AREA_PAVEMENT           (2)
#define AIRPORT_AREA_TAXIWAY            (3)
#define AIRPORT_AREA_RUNWAY_SHOULDER    (4)
#define AIRPORT_AREA_HELIPAD_SHOULDER   (5)
#define AIRPORT_AREA_INNER_BASE         (6)
#define AIRPORT_AREA_OUTER_BASE         (7)

#define AIRPORT_MAX_BASE                (6)
#define AIRPORT_NUM_AREAS               (8)

#define AIRPORT_LINE                    (0)

class Airport
{
public:
    Airport( int c, char* def);
    ~Airport();

    void AddRunway( Runway* runway )
    {
        runways.push_back( runway );
    }

    void AddWaterRunway( WaterRunway* waterrunway )
    {
        waterrunways.push_back( waterrunway );
    }

    void AddObj( LightingObj* lightobj )
    {
        lightobjects.push_back( lightobj );
    }

    void AddHelipad( Helipad* helipad )
    {
        helipads.push_back( helipad );
    }

    void AddTaxiway( Taxiway* taxiway )
    {
        taxiways.push_back( taxiway );
    }

    void AddPavement( ClosedPoly* pavement )
    {
        pavements.push_back( pavement );
    }

    void AddFeature( LinearFeature* feature )
    {
        features.push_back( feature );
    }

    void AddFeatures( FeatureList* feature_list )
    {
        for (unsigned int i=0; i<feature_list->size(); i++)
        {
            features.push_back( feature_list->at(i) );
        }
    }

    int NumFeatures( void )
    {
        return features.size();
    }

    void AddBoundary( ClosedPoly* bndry )
    {
        boundary.push_back( bndry );
    }

    void AddWindsock( Windsock* windsock )
    {
        windsocks.push_back( windsock );
    }

    void AddBeacon( Beacon* beacon )
    {
        beacons.push_back( beacon );
    }

    void AddSign( Sign* sign )
    {
        signs.push_back( sign );
    }

    std::string GetIcao( )
    {
        return icao;
    }

    void GetBuildTime( SGTimeStamp& tm )
    {
        tm = build_time;
    }

    void GetTriangulationTime( SGTimeStamp& tm )
    {
        tm = triangulation_time;
    }

    void GetCleanupTime( SGTimeStamp& tm )
    {
        tm = cleanup_time;
    }

    void merge_slivers( tgpolygon_list& polys, tgcontour_list& slivers );


    // break into stages
    void BuildBtg( const std::string& root, const string_list& elev_src );

    void DumpStats( void );

    void set_debug( std::string& path,
                    debug_map& dbg_runways, 
                    debug_map& dbg_pavements,
                    debug_map& dbg_taxiways,
                    debug_map& dbg_features ) {
        debug_path      = path;
        debug_runways   = dbg_runways;
        debug_pavements = dbg_pavements;
        debug_taxiways  = dbg_taxiways;
        debug_features  = dbg_features;
    };

    bool isDebugRunway  ( int i );
    bool isDebugPavement( int i );
    bool isDebugTaxiway ( int i );
    bool isDebugFeature ( int i );

private:
    // The airport building stages....
    // Step 1 - build the base polygons - clip against higher priorities
    void BuildBase();
    // Step 2 - clip the base polygons
    void ClipBase();
    // Step 2 - clean the base polygons - fix t-junctions
    void CleanBase();
    // Step 3 - tesselate the base polygons - generate triangle list
    void TesselateBase();

    void TexturePolys();
    
    // Step 4 - build the linear feature polygons
    void BuildFeatures();
    // Step 5 - add nodes from pavement intersections
    void IntersectFeaturesWithBase();
    // Step 6 - clean the features
    void CleanFeatures();
    // Step 7 - tesselate the feature polygons
    void TesselateFeatures();

    // Step 8 - texture all triangles
    void TextureTriangles();
    // Step 9 - calculate  elevations
    void CalcElevations(const std::string& root, const string_list& elev_src);

    void LookupIndexes(void);
    
    // Step 10 - output
    void WriteOutput( const std::string& root, const SGBucket& b );

    int          code;               // airport, heliport or sea port
    int          altitude;           // in meters
    std::string  icao;               // airport code
    std::string  description;        // description

    PavementList    pavements;
    FeatureList     features;
    RunwayList      runways;
    WaterRunwayList waterrunways;
    TaxiwayList     taxiways;
    LightingObjList lightobjects;
    WindsockList    windsocks;
    BeaconList      beacons;
    SignList        signs;
    HelipadList     helipads;
    PavementList    boundary;

    // runway lights
    tglightcontour_list rwy_lights;

    // Elevation data
    tgArray array;

    // area polygons
    tgAreas polys_built;
    tgAreas polys_clipped;

    // All Nodes
    TGNodes nodes;

    // stats
    SGTimeStamp build_time;
    SGTimeStamp cleanup_time;
    SGTimeStamp triangulation_time;

    // debug
    std::string     debug_path;
    debug_map       debug_runways;
    debug_map       debug_pavements;
    debug_map       debug_taxiways;
    debug_map       debug_features;
};

typedef std::vector <Airport *> AirportList;

#endif
