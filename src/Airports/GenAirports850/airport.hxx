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
#define AIRPORT_AREA_RWY_FEATURES       (8)
#define AIRPORT_AREA_TAXI_FEATURES      (9)

#define AIRPORT_MAX_BASE                (6)
#define AIRPORT_NUM_AREAS               (10)

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


    tgIntersectionGenerator* GetLFG( unsigned int type ) { 
        if ( type <= 9 ) {
            return normal_lf_ig[type-1]; 
        } else if ( type <= 22 ) {
            return white_lf_ig[type-20];
        } else if ( type <= 59 ) {
            return black_lf_ig[type-51];             
        } else {
            return rm_ig;
        }
    }
    tgIntersectionGenerator* GetRMG( void ) { return rm_ig; }
    
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
    
    // Build the base (base_construct)
    
    // Step 1 - build the base polygons - clip against higher priorities
    void BuildBase();
    // Step 2 - clip the base polygons
    void ClipBase();
    // Step 2 - clean the base polygons - fix t-junctions
    void CleanBase();
    // Step 3 - tesselate the base polygons - generate triangle list
    void TesselateBase();
    void TextureBase();
    // Step 9 - calculate  elevations
    void CalcBaseElevations(const std::string& root, const string_list& elev_src);    
    void LookupBaseIndexes(void);
    // Step 10 - output
    void WriteBaseOutput( const std::string& root, const SGBucket& b );

    // Build the features (feat_construct)
    
    // Step 4 - build the linear feature polygons
    void BuildFeatures();
    void ClipFeatures();
    void CleanFeatures();
    void IntersectFeaturesWithBase(void);
    // Step 3 - tesselate the base polygons - generate triangle list
    void TesselateFeatures();
    void TextureFeatures();
    void CalcFeatureElevations(void);    
    void LookupFeatureIndexes(void);
    // Step 10 - output
    void WriteFeatureOutput( const std::string& root, const SGBucket& b );
    
    // Build the lights (light_construct)
    void BuildLights( void );
    void WriteLightsOutput( const std::string& root, const SGBucket& b );
    
    // Airport Objects
    void WriteObjects( const std::string& root, const SGBucket& b );
    
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

    // feature poly intersections ( one for each type )
    tgIntersectionGenerator* normal_lf_ig[9];
    tgIntersectionGenerator* white_lf_ig[3];
    tgIntersectionGenerator* black_lf_ig[9];
    tgIntersectionGenerator* rm_ig;
    
    // runway lights
    tglightcontour_list lights;

    // Elevation data
    tgArray array;

    // the smoothing surface for generating the base
    tgSurface       base_surf;
    
    // the triangles making up the base mesh
    tgtriangle_list base_mesh;
    
    // area polygons
    tgAreas polys_built;
    tgAreas polys_clipped;

    // Base Nodes
    TGNodes base_nodes;
    tgPolygon inner_base, outer_base;

    // Feature Nodes
    TGNodes feat_nodes;
    
    // Light Nodes
    TGNodes light_nodes;
    
    
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
