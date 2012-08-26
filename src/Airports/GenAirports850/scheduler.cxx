#include <cstring>

#include <Poco/Environment.h>

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>

#include "airport.hxx"
#include "parser.hxx"
#include "scheduler.hxx"

extern double gSnap;

/*** PROCESS INFO ***/
ProcessInfo::ProcessInfo( AirportInfo* pai, const ProcessHandle ph, Net::StreamSocket s ) : procHandle(ph)
{
    pInfo       = pai;
    sock        = s;
    pssb        = new Net::SocketStreamBuf( s );
    pin         = new istream( pssb );
    state       = P_STATE_INIT;
    SetTimeout();
}

void ProcessInfo::SetTimeout( void )
{
    SGTimeStamp now;
    SGTimeStamp to;

    now.stamp();
    switch( state ) {
        case P_STATE_INIT:
            to.setTime(P_STATE_INIT_TIME, 0);
            break;

        case P_STATE_PARSE:
            to.setTime(P_STATE_PARSE_TIME, 0);
            break;

        case P_STATE_BUILD:
            to.setTime(P_STATE_BUILD_TIME, 0);
            break;

        case P_STATE_TRIANGULATE:
            to.setTime(P_STATE_TRIANGULATE_TIME, 0);
            break;

        case P_STATE_OUTPUT:
            to.setTime(P_STATE_OUTPUT_TIME, 0);
            break;
    }

    timeout = (now + to);
}

void ProcessInfo::Kill( void )
{
    SGTimeStamp now;
    now.stamp();

    // kill the process
    Process::kill( procHandle.id() );

    // wait for the zombie
    procHandle.wait();

    // mark process info - so we can reclaim it
    state = P_STATE_KILLED;
}

int ProcessInfo::HandleLine( void )
{
    char line[256];
    
    pin->getline( line, 256 );
    if ( pin->rdstate() != ifstream::goodbit ) {
        SG_LOG( SG_GENERAL, SG_INFO, pInfo->GetIcao() << ": ProcessInfo::HandleLine read from socket error " << pin->rdstate() );

        state = P_STATE_KILLED;
    } else {
        pInfo->SetErrorString( line );

        // Print the line
        SG_LOG( SG_GENERAL, SG_INFO, pInfo->GetIcao() << ": " << line);

        // Update state
        if ( strstr( line, "Parse Complete " ) != NULL ) {
            // Grab the stats
            int rwys, pvmnts, feats, twys;

            sscanf(line, "Parse Complete - Runways: %d Pavements: %d Features: %d Taxiways: %d", &rwys, &pvmnts, &feats, &twys);

            pInfo->SetRunways( rwys );
            pInfo->SetPavements( pvmnts );
            pInfo->SetFeats( feats );
            pInfo->SetTaxiways( twys );

        } else if ( strstr( line, "Finished airport " ) != NULL ) {
            // Grab the stats
            int         parse_sec, parse_nsec;
            int         build_sec, build_nsec;
            int         clean_sec, clean_nsec;
            int         tess_sec,  tess_nsec;
            SGTimeStamp parse_ts;
            SGTimeStamp build_ts;
            SGTimeStamp clean_ts;
            SGTimeStamp tess_ts;

            state = P_STATE_DONE;

            sscanf(line, "Finished airport %*s : parse %d.%d : build %d.%d : clean %d.%d : tesselate %d.%d", 
                &parse_sec, &parse_nsec, &build_sec, &build_nsec, &clean_sec, &clean_nsec, &tess_sec, &tess_nsec );
         
            parse_ts.setTime( parse_sec, parse_nsec );
            build_ts.setTime( build_sec, build_nsec );
            clean_ts.setTime( clean_sec, clean_nsec );
            tess_ts.setTime( tess_sec, tess_nsec );

            pInfo->SetParseTime( parse_ts );
            pInfo->SetBuildTime( build_ts );
            pInfo->SetCleanTime( clean_ts );
            pInfo->SetTessTime( tess_ts );

            procHandle.wait();        
        } else if ( strstr( line, "Build Feature Poly " ) != NULL ) {
            state = P_STATE_BUILD;
        } else if ( strstr( line, "Build Pavement " ) != NULL ) {
            state = P_STATE_BUILD;
        } else if ( strstr( line, "Build Runway " ) != NULL ) {
            state = P_STATE_BUILD;
        } else if ( strstr( line, "Tesselating " ) != NULL ) {
            state = P_STATE_TRIANGULATE;
        } else if ( strstr( line, "Adding runway nodes and normals " ) != NULL ) {
            state = P_STATE_OUTPUT;
        }
 
    }

    SetTimeout();
    return state;
}


ostream& operator<< (ostream &out, const AirportInfo &ai)
{
    char snap_string[32];
    sprintf( snap_string, "%1.8lf", ai.snap );
    
    out << ai.icao;
    out << ",";
    out << ai.numRunways;
    out << ",";
    out << ai.numPavements;
    out << ",";
    out << ai.numFeats;
    out << ",";
    out << ai.numTaxiways;
    out << ",";
    out << ai.parseTime;
    out << ",";
    out << ai.buildTime;
    out << ",";
    out << ai.cleanTime;
    out << ",";
    out << ai.tessTime;
    out << ",";
    out << ai.parseTime+ai.buildTime+ai.cleanTime+ai.tessTime;
    out << ",";
    out << snap_string,
    out << ",";
    out << ai.errString;

    return out;  // MSVC
}


/*** PROCESS LIST CLASS ***/
ProcessList::ProcessList( int n, string& summaryfile, Scheduler* pScheduler ) : available(n), ready(1), state( PL_STATE_WAIT_FOR_LAUNCH )
{
    // The process List is responsible for creating new processes (Launch)
    // and monitoring the status of the launched parsers (Monitor)  These 
    // functions are called from different threads.  
    pss = pScheduler->GetServerSocket();

    // remember the output file 
    csvfile.open( summaryfile.c_str(), ios_base::out | ios_base::app );

    // remember the scheduler so we can add retries
    scheduler = pScheduler;

    // remember the number of available helper procs so we know when we're full
    threads = n;
}
    
// When a slot is available, the main thread calls launch to instantiate a 
// new pareser process 
void ProcessList::Launch( string command, string work_dir, string file, AirportInfo* pai, bool last ) 
{
    Process::Args args;
    char arg[512];
    Pipe outPipe;

    // generate correct command line arguments
    sprintf( arg, "--work=%s", work_dir.c_str() );
    args.push_back(arg);

    sprintf( arg, "--input=%s", file.c_str() );
    args.push_back(arg);

    sprintf( arg, "--airport-pos=%ld", pai->GetPos() );
    args.push_back(arg);

    sprintf( arg, "--snap=%1.8lf", pai->GetSnap() );
    args.push_back(arg);

    sprintf( arg, "--redirect-port=%d", GENAPT_PORT );
    args.push_back(arg);
        
    // Launch the child process
    ProcessHandle ph = Process::launch(command, args, 0, &outPipe, &outPipe);

    // Wait 10 seconds for connection
    Timespan timeout( 10, 0 );
    bool retVal = pss->poll( timeout, Net::Socket::SELECT_READ );

    // If we connected - create a new entry
    if ( retVal ) {
        Net::SocketAddress sockaddr;
        Net::StreamSocket sock = pss->acceptConnection( sockaddr );

        // Make sure the list can't be modified while adding a member
        lock.lock();
        ProcessInfo pi( pai, ph, sock );
        plist.push_back( pi );
        lock.unlock();        

        // If we have all of the airports in our list, we are done
        // when the list is empty - set the transition state
        if ( last ) {
            // The launch list is empty - we're ready to monitor
            state = PL_STATE_ALL_LAUNCHED;
            ready.set();
        } else if ( plist.size() == threads ) {
            // The resource list is full - we're ready to monitor
            state = PL_STATE_LIST_FULL;
            ready.set();
        } else {
            // resource list has space, and launch list is not empty - hold off monitoring
            state = PL_STATE_WAIT_FOR_LAUNCH;
        }
    }
}

Timespan ProcessList::GetNextTimeout() 
{
    SGTimeStamp now, min, timeout;

    min.setTime( UINT_MAX, 0 );
    timeout.setTime( 0, 0 );
        
    for ( unsigned int i=0; i< plist.size(); i++ ) {
        if ( plist[i].GetTimeout() < min ) {
            min = plist[i].GetTimeout();
        }
    }

    now.stamp();
    if ( min > now ) {
        timeout = min - now;
    }

    return Timespan( timeout.get_seconds(), timeout.get_usec() );
}

void ProcessList::HandleReceivedMessages( Net::Socket::SocketList& slr ) 
{
    // for each socket that has data - find the corresponding icao
    for (unsigned int i=0; i<slr.size(); i++) {
        Net::StreamSocket ss = (Net::StreamSocket)slr[i];

        // find the index handling this socket, and let it deal with the line
        for ( unsigned int j=0; j < plist.size(); j++ ) {
            if ( plist[j].GetSocket() == ss ) {
                plist[j].HandleLine( );
                break;
            }
        }
    }
}

void ProcessList::HandleFinished( void )
{
    AirportInfo* pInfo = NULL;
    int          num_deleted = 0;        
    bool         done = false;

    while (!done) {
        done = true;

        lock.lock();
        for ( unsigned int i=0; i< plist.size(); i++ ) {
            switch ( plist[i].GetState() ) {
                case P_STATE_DONE:
                    plist[i].SetErrorString( (char *)"success" );

                    // holding the list lock - only one thread can write to the csvfile at a time
                    csvfile << plist[i].GetInfo() << "\n";
                    csvfile.flush();

                    // remove this airport from the list - it's complete
                    plist[i].CloseSock();
                    plist.erase( plist.begin()+i );

                    // keep track of the number of deleted entries
                    num_deleted++;

                    // let's iterate again to look for more timeouts...
                    done = false;
                    break;

                case P_STATE_KILLED:
                    // holding the list lock - only one thread can write to the csvfile at a time
                    csvfile << plist[i].GetInfo() << "\n";
                    csvfile.flush();

                    // Schedule a retry
                    pInfo = plist[i].GetInfoPtr();
                    pInfo->IncreaseSnap();
                    scheduler->RetryAirport( pInfo );

                    // remove the airport from the monitor list - it's complete
                    plist[i].CloseSock();
                    plist.erase( plist.begin()+i );

                    // keep track of the number of deleted entries
                    num_deleted++;

                    // let's iterate again to look for more timeouts...
                    done = false;
                    break;

                default:
                    break;
            }
        }

        lock.unlock();
    }


    // Let launcher thread know we have opening(s)
    while ( num_deleted-- ) {
        // make sure we don't start waiting on output before a new apt is launched...
        if (state != PL_STATE_ALL_LAUNCHED) {
            state = PL_STATE_WAIT_FOR_LAUNCH;
        }

        // free each resource that is no longer used
        available.set();
    }
}

void ProcessList::WaitForSlot( void )
{ 
    available.wait(); 
}

// list lock is held
void ProcessList::HandleTimeouts() 
{
    SGTimeStamp now;

    now.stamp();
    for ( unsigned int i=0; i< plist.size(); i++ ) {
        if ( plist[i].GetTimeout() < now ) {
            plist[i].Kill();
        }
    }
}

void ProcessList::Monitor() 
{
    // Wait until process list has a connection, then continue until we are done
    while( state != PL_STATE_DONE ) {
        Net::Socket::SocketList     slr, slw, sle;
        Timespan                    timeout;
        int                         retVal;

        // if we aren't ready to start - wait on ready
        if ( state == PL_STATE_WAIT_FOR_LAUNCH ) {
            ready.wait();       
        }

        // then lock the list when calculating the timeouts 
        lock.lock();

        // calculate the shortest timeout
        timeout = GetNextTimeout();

        // Add currently connected sockets
        for ( unsigned int i=0; i< plist.size(); i++ ) {
            slr.push_back( plist[i].GetSocket() );
        }

        // unlock before waiting on i/o
        lock.unlock();

        // this needs to be interrupted when new airports are added to the list
        retVal =  Net::Socket::select( slr, slw, sle, timeout );

        if ( retVal > 0 ) {
            HandleReceivedMessages( slr );
        } else {
            HandleTimeouts();
        }

        // remove finished or dead processes, and notify launcher 
        //
        HandleFinished();

        slr.clear();
        slw.clear();
        sle.clear();

        // if we have launched all airports, we are done
        if ( ( state == PL_STATE_ALL_LAUNCHED ) && ( plist.size() == 0 ) ) {
            state = PL_STATE_DONE;
        }
    }

    csvfile.close();
}

/*** PROCESS MONITOR ***/
ProcessMonitor::ProcessMonitor(ProcessList* pl) : Runnable()
{
    plist = pl;
}

void ProcessMonitor::run()
{
    SG_LOG( SG_GENERAL, SG_INFO, "ProcessMonitor Started " );

    // Run the monitoring function in this thread
    plist->Monitor();

    SG_LOG( SG_GENERAL, SG_INFO, "ProcessMonitor Exited " );
}

/*** SCEDULER ***/
bool Scheduler::IsAirportDefinition( char* line, string icao )
{
    char*    tok;
    int      code;
    Airport* airport = NULL;
    bool     match = false;
    
    // Get the number code
    tok = strtok(line, " \t\r\n");

    if (tok)
    {
        line += strlen(tok)+1;
        code = atoi(tok);

        switch(code)
        {
            case LAND_AIRPORT_CODE: 
            case SEA_AIRPORT_CODE:
            case HELIPORT_CODE:
                airport = new Airport( code, line );
                if ( airport->GetIcao() == icao )
                {
                    match = true;
                }
                break;

            case LAND_RUNWAY_CODE:
            case WATER_RUNWAY_CODE:
            case HELIPAD_CODE:
            case PAVEMENT_CODE:
            case LINEAR_FEATURE_CODE:
            case BOUNDRY_CODE:
            case NODE_CODE:
            case BEZIER_NODE_CODE:
            case CLOSE_NODE_CODE:
            case CLOSE_BEZIER_NODE_CODE:
            case TERM_NODE_CODE:
            case TERM_BEZIER_NODE_CODE:
            case AIRPORT_VIEWPOINT_CODE:
            case AIRPLANE_STARTUP_LOCATION_CODE:
            case LIGHT_BEACON_CODE:
            case WINDSOCK_CODE:
            case TAXIWAY_SIGN:
            case LIGHTING_OBJECT:
            case COMM_FREQ1_CODE:
            case COMM_FREQ2_CODE:
            case COMM_FREQ3_CODE:
            case COMM_FREQ4_CODE:
            case COMM_FREQ5_CODE:
            case COMM_FREQ6_CODE:
            case COMM_FREQ7_CODE:
            case END_OF_FILE :
                break;
        }
    }

    return match;
}

void Scheduler::AddAirport( string icao )
{
    char            line[2048];
    long            cur_pos;
    bool            found = false;
    AirportInfo*    pInfo;

    ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

    SG_LOG( SG_GENERAL, SG_INFO, "Adding airport " << icao << " to parse list");
    while ( !in.eof() && !found ) 
    {
        // remember the position of this line
        cur_pos = in.tellg();

        // get a line
    	in.getline(line, 2048);

        // this is and airport definition - remember it
        if ( IsAirportDefinition( line, icao ) )
        {
            SG_LOG( SG_GENERAL, SG_DEBUG, "Found airport " << icao << " at " << cur_pos );
         
            pInfo = new AirportInfo( icao, cur_pos, gSnap );   
            originalList.push_back( *pInfo );
            delete pInfo;

            found = true;
        }
    }    
}

long Scheduler::FindAirport( string icao )
{
    char line[2048];
    long cur_pos = 0;
    bool found = false;

    ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

    SG_LOG( SG_GENERAL, SG_DEBUG, "Finding airport " << icao );
    while ( !in.eof() && !found ) 
    {
        // remember the position of this line
        cur_pos = in.tellg();

        // get a line
    	in.getline(line, 2048);

        // this is and airport definition - remember it
        if ( IsAirportDefinition( line, icao ) )
        {
            SG_LOG( SG_GENERAL, SG_DEBUG, "Found airport " << line << " at " << cur_pos );
            found = true;
        }
    }    

	if (found)
	{
		return cur_pos;
	}
	else
	{
		return 0;
	}
}

void Scheduler::RetryAirport( AirportInfo* pai )
{
    retryList.push_back( *pai );        
}

bool Scheduler::AddAirports( long start_pos, float min_lat, float min_lon, float max_lat, float max_lon )
{
    char 	 line[2048];
    char*	 def;
    long 	 cur_pos;
    long	 cur_apt_pos = 0;
    string   cur_apt_name;
    char*    tok;
    int      code;
    bool 	 match;
    bool 	 done;

    done  = false;
    match = false;

	// start from current position, and push all airports where a runway start or end 
	// lies within the given min/max coordinates

    ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

	if (start_pos)
	{
        in.seekg(start_pos, ios::beg);
	}

	while (!done)
	{
	  	// remember the position of this line
	   	cur_pos = in.tellg();

	   	// get a line
	   	in.getline(line, 2048);
		def = &line[0];

    	// Get the number code
    	tok = strtok(def, " \t\r\n");

	    if (tok)
    	{
    	    def += strlen(tok)+1;
    	    code = atoi(tok);

	        switch(code)
    	    {
    	        case LAND_AIRPORT_CODE: 
    	        case SEA_AIRPORT_CODE:
    	        case HELIPORT_CODE:
					{
						Airport* airport = new Airport( code, def );
						if (match)
						{
                            // Start off with given snap value
                            AirportInfo* pInfo = new AirportInfo( cur_apt_name, cur_apt_pos, gSnap );   
                            originalList.push_back( *pInfo );
                            delete pInfo;
						}
						// remember this new apt pos and name, and clear match
						cur_apt_pos  = cur_pos;
						cur_apt_name = airport->GetIcao();
						delete airport;

						match = false;
					}
    	            break;

				case END_OF_FILE:
					if (match)
					{
                        // Start off with given snap value
                        AirportInfo* pInfo = new AirportInfo( cur_apt_name, cur_apt_pos, gSnap );   
                        originalList.push_back( *pInfo );
                        delete pInfo;
					}
					done = true;
					break;

	            case LAND_RUNWAY_CODE:
					// if the the runway start / end  coords are within the rect, 
					// we have a winner
					{ 
						Runway* runway = new Runway(def);
						Point3D start = runway->GetStart();
						Point3D end   = runway->GetEnd();
						if ( (start.x() >= min_lon ) && 
						     (start.y() >= min_lat ) &&
							 (start.x() <= max_lon ) &&
							 (start.y() <= max_lat ) ) {
							match = true;
						}
						else if ( (end.x() >= min_lon ) && 
						     (end.y() >= min_lat ) &&
							 (end.x() <= max_lon ) &&
							 (end.y() <= max_lat ) ) {
							match = true;
						}
						delete runway;
					}
					break;

    	        case WATER_RUNWAY_CODE:
					// if the the runway start / end  coords are within the rect, 
					// we have a winner
					{ 
						WaterRunway* runway = new WaterRunway(def);
						Point3D start = runway->GetStart();
						Point3D end   = runway->GetEnd();
						if ( (start.x() >= min_lon ) && 
						     (start.y() >= min_lat ) &&
							 (start.x() <= max_lon ) &&
							 (start.y() <= max_lat ) ) {
							match = true;
						}
						else if ( (end.x() >= min_lon ) && 
						     (end.y() >= min_lat ) &&
							 (end.x() <= max_lon ) &&
							 (end.y() <= max_lat ) ) {
							match = true;
						}
						delete runway;
					}
					break;

    	        case HELIPAD_CODE:
					// if the heliport coords are within the rect, we have
					// a winner
					{ 
						Helipad* helipad = new Helipad(def);
						Point3D  loc = helipad->GetLoc();
						if ( (loc.x() >= min_lon ) && 
						     (loc.y() >= min_lat ) &&
							 (loc.x() <= max_lon ) &&
							 (loc.y() <= max_lat ) ) {
							match = true;
						}
						delete helipad;
					}
					break;

                case TAXIWAY_CODE:
    	        case PAVEMENT_CODE:
	            case LINEAR_FEATURE_CODE:
    	        case BOUNDRY_CODE:
    	        case NODE_CODE:
    	        case BEZIER_NODE_CODE:
    	        case CLOSE_NODE_CODE:
    	        case CLOSE_BEZIER_NODE_CODE:
    	        case TERM_NODE_CODE:
    	        case TERM_BEZIER_NODE_CODE:
    	        case AIRPORT_VIEWPOINT_CODE:
    	        case AIRPLANE_STARTUP_LOCATION_CODE:
    	        case LIGHT_BEACON_CODE:
    	        case WINDSOCK_CODE:
    	        case TAXIWAY_SIGN:
    	        case LIGHTING_OBJECT:
    	        case COMM_FREQ1_CODE:
    	        case COMM_FREQ2_CODE:
    	        case COMM_FREQ3_CODE:
    	        case COMM_FREQ4_CODE:
    	        case COMM_FREQ5_CODE:
    	        case COMM_FREQ6_CODE:
    	        case COMM_FREQ7_CODE:
    	            break;
    	    }
    	}
	}

	// did we add airports to the parse list?
        if ( originalList.size() )
        {
            return true;
        } else
        {
            return false;
        }
}

Scheduler::Scheduler(string& cmd, string& datafile, const string& root, const string_list& elev_src)
{
    command         = cmd;
    filename        = datafile;
    work_dir        = root;
    elevation       = elev_src;

    ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }
}

void Scheduler::Schedule( int num_threads, string& summaryfile )
{
    ProcessList     *procList = NULL;
    Thread          *monThread = NULL;
    ProcessMonitor  *procMon = NULL;
    bool            done = false;
    bool            last = false;
    ofstream        csvfile;

    // open and truncate the summary file : monitor only appends 
    csvfile.open( summaryfile.c_str(), ios_base::out | ios_base::trunc );
    csvfile.close();

    SG_LOG( SG_GENERAL, SG_INFO, "Scheduler: Bind to socket" );

    // Bind the parent listener socket for children to connect to
    ss.bind(GENAPT_PORT);
    ss.listen();

    SG_LOG( SG_GENERAL, SG_INFO, "Scheduler: Bound" );
    
    while (!done) {
        procList  = new ProcessList(num_threads, summaryfile, this);
        monThread = new Thread;
        procMon   = new ProcessMonitor(procList);

        // Launch monitor thread
        monThread->start(*procMon);

        // now try to launch child processes to parse individual airports
        for ( unsigned int i=0; i<originalList.size(); i++ ) {
            // Wait for an available process slot
            procList->WaitForSlot();

            SG_LOG( SG_GENERAL, SG_INFO, "Scheduler: originalList has " << originalList.size() << ", i is " << i );

            // let the process list know if more airports are coming
            if ( i == originalList.size()-1 ) {
                last = true;
            }

            // Launch a new parser
            procList->Launch( command, work_dir, filename, &originalList[i], last );
        }

        // Sync up before relaunching
        monThread->join();

        // Delete the old monitor
        delete procMon;
        delete monThread;
        delete procList;

        SG_LOG( SG_GENERAL, SG_INFO, "Scheduler: originalList has " << originalList.size() << ", retry list has " << retryList.size() << " entries" );

        // delete original, and copy retry to it
        if ( retryList.size() ) {
            SG_LOG( SG_GENERAL, SG_INFO, "Scheduler: clear original list " );            
            originalList.clear();

            SG_LOG( SG_GENERAL, SG_INFO, "Scheduler - cleared original: originalList has " << originalList.size() << ", retry list has " << retryList.size() << " entries" );

            for ( unsigned int i=0; i<retryList.size(); i++ ) {
                originalList.push_back( retryList[i] );
            }

            SG_LOG( SG_GENERAL, SG_INFO, "Scheduler - copied retryList: originalList has " << originalList.size() << ", retry list has " << retryList.size() << " entries" );

            retryList.clear();

            SG_LOG( SG_GENERAL, SG_INFO, "Scheduler - cleared retry: originalList has " << originalList.size() << ", retry list has " << retryList.size() << " entries" );
        } else {
            done = true;
        }
    }
}
