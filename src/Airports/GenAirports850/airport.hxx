#ifndef _AIRPORT_H_
#define _AIRPORT_H_

#include <memory>

#include <simgear/timing/timestamp.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/threads/SGThread.hxx>

#include "runway.hxx"
#include "object.hxx"
#include "helipad.hxx"
#include "taxiway.hxx"
#include "closedpoly.hxx"
#include "linearfeature.hxx"
#include "linked_objects.hxx"
#include "debug.hxx"

class Airport
{
public:
    Airport( int c, char* def);
    ~Airport();

    void AddRunway( std::shared_ptr<Runway> runway )
    {
        runways.push_back( runway );
    }

    void AddWaterRunway( std::shared_ptr<WaterRunway> waterrunway )
    {
        waterrunways.push_back( waterrunway );
    }

    void AddObj( std::shared_ptr<LightingObj> lightobj )
    {
        lightobjects.push_back( lightobj );
    }

    void AddHelipad( std::shared_ptr<Helipad> helipad )
    {
        helipads.push_back( helipad );
    }

    void AddTaxiway( std::shared_ptr<Taxiway> taxiway )
    {
        taxiways.push_back( taxiway );
    }

    void AddPavement( std::shared_ptr<ClosedPoly> pavement )
    {
        pavements.push_back( pavement );
    }

    void AddFeature( std::shared_ptr<LinearFeature> feature )
    {
        features.push_back( feature );
    }

    void AddFeatures( FeatureList feature_list )
    {
        for (auto feature : feature_list)
        {
            features.push_back( feature );
        }
    }

    int NumFeatures( void )
    {
        return features.size();
    }

    void AddBoundary( std::shared_ptr<ClosedPoly> bndry )
    {
        boundary.push_back( bndry );
    }

    void AddWindsock( std::shared_ptr<Windsock> windsock )
    {
        windsocks.push_back( windsock );
    }

    void AddBeacon( std::shared_ptr<Beacon> beacon )
    {
        beacons.push_back( beacon );
    }

    void AddSign( std::shared_ptr<Sign> sign )
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
    bool CheckZFightingTriangles( const char* prefix, const char* debug_root, const tgPolygon& base_poly, const tgpolygon_list& rwy_polys, const tgpolygon_list& pvmt_polys );
    
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

    // stats
    SGTimeStamp build_time;
    SGTimeStamp cleanup_time;
    SGTimeStamp triangulation_time;

    // debug
    std::string          debug_path;
    debug_map       debug_runways;
    debug_map       debug_pavements;
    debug_map       debug_taxiways;
    debug_map       debug_features;
};

typedef std::vector<std::shared_ptr<Airport>> AirportList;

#endif
