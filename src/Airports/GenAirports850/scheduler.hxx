#ifndef __SCHEDULER_HXX__
#define __SCHEDULER_HXX__

#include <string>
#include <iostream>
#include <fstream>

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>
#include <simgear/timing/timestamp.hxx>
#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGQueue.hxx>
#include <terragear/tg_rectangle.hxx>
#include "airport.hxx"

#define P_STATE_INIT        (0)
#define P_STATE_PARSE       (1)
#define P_STATE_BUILD       (2)
#define P_STATE_TRIANGULATE (3)
#define P_STATE_OUTPUT      (4)
#define P_STATE_DONE        (8)
#define P_STATE_KILLED      (9)

#define P_STATE_INIT_TIME           ( 1*60)
#define P_STATE_PARSE_TIME          ( 1*60)
#define P_STATE_BUILD_TIME          (30*60)
#define P_STATE_TRIANGULATE_TIME    ( 1*60)
#define P_STATE_OUTPUT_TIME         (10*60)

#define GENAPT_PORT                 (12397)
#define PL_STATE_INIT               (0)
#define PL_STATE_WAIT_FOR_LAUNCH    (1)
#define PL_STATE_LIST_FULL          (2)
#define PL_STATE_ALL_LAUNCHED       (3)
#define PL_STATE_DONE               (10)

// Forward declaration
class Scheduler;

class AirportInfo
{
public:
    AirportInfo()
    {
    }

    AirportInfo( const std::string& id, long p, double s ) :
        icao(id)
    {
        pos  = p;
        snap = s;

        numRunways = -1;
        numPavements = -1;
        numFeats = -1;
        numTaxiways = -1;
    }

    std::string GetIcao( void )                     { return icao; }
    long    GetPos( void )                          { return pos; }
    double  GetSnap( void )                         { return snap; }

    void    SetRunways( int r )                     { numRunways = r; }
    void    SetPavements( int p )                   { numPavements = p; }
    void    SetFeats( int f )                       { numFeats = f; }
    void    SetTaxiways( int t )                    { numTaxiways = t; }
    void    SetParseTime( SGTimeStamp t )           { parseTime = t; }
    void    SetBuildTime( SGTimeStamp t )           { buildTime = t; }
    void    SetCleanTime( SGTimeStamp t )           { cleanTime = t; }
    void    SetTessTime( SGTimeStamp t )            { tessTime = t; }
    void    SetErrorString( char* e )               { errString = e; }

    void    IncreaseSnap( void )                    { snap *= 2.0f; }

    friend std::ostream& operator<<(std::ostream& output, const AirportInfo& ai);

private:
    std::string icao;
    long        pos;

    int         numRunways;
    int         numPavements;
    int         numFeats;
    int         numTaxiways;

    SGTimeStamp parseTime;
    SGTimeStamp buildTime;
    SGTimeStamp cleanTime;
    SGTimeStamp tessTime;

    double      snap;
    std::string errString;
};

extern SGLockedQueue<AirportInfo> global_workQueue;

class Scheduler
{
public:
    Scheduler(std::string& datafile, const std::string& root, const string_list& elev_src);

    long            FindAirport( const std::string& icao );
    void            AddAirport(  std::string icao );
    bool            AddAirports( long start_pos, tgRectangle* boundingBox );
    void            RetryAirport( AirportInfo* pInfo );

    void            Schedule( int num_threads, std::string& summaryfile );

    // Debug
    void            set_debug( const std::string& path, std::vector<std::string> runway_defs,
                                                 std::vector<std::string> pavement_defs,
                                                 std::vector<std::string> taxiway_defs,
                                                 std::vector<std::string> feature_defs );

private:
    bool            IsAirportDefinition( char* line, const std::string& icao );

    std::string     filename;
    string_list     elevation;
    std::string     work_dir;

    // debug
    std::string     debug_path;
    debug_map       debug_runways;
    debug_map       debug_pavements;
    debug_map       debug_taxiways;
    debug_map       debug_features;
};

#endif
