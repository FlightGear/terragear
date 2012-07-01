#include <string>
#include <iostream>
#include <fstream>

#include <Poco/Mutex.h>
#include <Poco/Pipe.h>
#include <Poco/PipeStream.h>
#include <Poco/Process.h>
#include <Poco/Runnable.h>
#include <Poco/Semaphore.h>
#include <Poco/Thread.h>
#include <Poco/Timespan.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/Socket.h>
#include <Poco/Net/StreamSocket.h>

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>
#include <simgear/timing/timestamp.hxx>

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

using namespace std;
using namespace Poco;

// Forward declaration
class Scheduler;

class AirportInfo
{
public:
    AirportInfo( string id, long p, double s )
    {
        icao = id;
        pos  = p;
        snap = s;
        
        numRunways = -1;
        numPavements = -1;
        numFeats = -1;
        numTaxiways = -1;
    }

    string  GetIcao( void )                         { return icao; }
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

    friend ostream& operator<<(ostream& output, const AirportInfo& ai);

private:
    string      icao;
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
    string      errString;
};
typedef std::vector <AirportInfo> parseList;

class ProcessInfo
{
public:
    ProcessInfo( AirportInfo* pai, const ProcessHandle ph, Net::StreamSocket s );

    void                SetTimeout( void );
    SGTimeStamp         GetTimeout( void )    { return timeout; }
    string              GetIcao( void )       { return pInfo->GetIcao(); }
    Net::StreamSocket   GetSocket( void )     { return sock; }
    int                 GetState( void )      { return state; }
    AirportInfo         GetInfo( void )       { return *pInfo; }
    AirportInfo*        GetInfoPtr( void )    { return pInfo; }

    void                SetErrorString( char *e )   { pInfo->SetErrorString( e ); }

    int                 HandleLine( void );
    void                Kill( void );
    void                CloseSock( void )     { sock.close(); }

private:
    AirportInfo*         pInfo;
    ProcessHandle        procHandle;
    Net::StreamSocket    sock;
    Net::SocketStreamBuf *pssb;
    istream              *pin;
    int                  state;
    SGTimeStamp          timeout;
};
typedef std::vector <ProcessInfo> ProcessInfoList;

class ProcessList
{
public:
    ProcessList( int n, string& summaryfile, Scheduler* pScheduler );

    // The main thread needs to wait until a slot is ready for creating a new
    // Parser child process
    inline void WaitForSlot(void);
    
    // When a slot is available, the main thread calls launch to instantiate a 
    // new pareser process 
    void Launch( string command, string work_dir, string file, AirportInfo* pai, bool last );
    Timespan GetNextTimeout();
    void HandleReceivedMessages( Net::Socket::SocketList& slr );
    void HandleTimeouts();
    void HandleFinished( void );
    void Monitor();
    
private:
    Semaphore           available;
    Semaphore           ready;
    Mutex               lock;
    ProcessInfoList     plist;
    Net::ServerSocket*  pss;
    int                 state;
    int                 threads;
    ofstream            csvfile;
    Scheduler*          scheduler;
};

class ProcessMonitor : public Runnable
{
public:
    ProcessMonitor(ProcessList* pl);
    virtual void run();

private:
    ProcessList*    plist;    
};

class Scheduler
{
public:
    Scheduler(string& cmd, string& datafile, const string& root, const string_list& elev_src);

    long            FindAirport( string icao );
    void            AddAirport(  string icao );
    void            AddAirports( long start_pos, float min_lat, float min_lon, float max_lat, float max_lon );
    void            RetryAirport( AirportInfo* pInfo );

    void            Schedule( int num_threads, string& summaryfile );

    Net::ServerSocket*  GetServerSocket( void ) { return &ss; }

private:
    bool            IsAirportDefinition( char* line, string icao );

    string          command;
    string          filename;
    string_list     elevation;
    string          work_dir;

    Net::ServerSocket  ss;

    // List of positions in database file to parse
    parseList       originalList;
    parseList       retryList;
};

