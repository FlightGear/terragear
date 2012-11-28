#include <cstring>

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>

#include "airport.hxx"
#include "parser.hxx"
#include "scheduler.hxx"

extern double gSnap;

SGLockedQueue<AirportInfo> global_workQueue;

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

void Scheduler::set_debug( std::string path, std::vector<std::string> runway_defs,
                                             std::vector<std::string> pavement_defs,
                                             std::vector<std::string> taxiway_defs,
                                             std::vector<std::string> feature_defs )
{
    GENAPT_LOG(SG_GENERAL, SG_ALERT, "Set debug Path " << path);

    debug_path = path;

    /* Find any ids for our tile */
    for (unsigned int i=0; i< runway_defs.size(); i++) {
        std::string dsd     = runway_defs[i];
        size_t d_pos   = dsd.find(":");

        std::string icao    = dsd.substr(0, d_pos);
        std::vector<int> shapes;
        shapes.clear();

        dsd.erase(0, d_pos+1);

        if ( dsd == "all" ) {
            shapes.push_back( std::numeric_limits<int>::max() );
        } else {
            std::stringstream ss(dsd);
            int i;

            while (ss >> i)
            {
                GENAPT_LOG(SG_GENERAL, SG_ALERT, "Adding debug runway " << i << " for " << icao );

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_runways[icao] = shapes;
    }

    for (unsigned int i=0; i< pavement_defs.size(); i++) {
        std::string dsd     = pavement_defs[i];
        size_t d_pos   = dsd.find(":");

        std::string icao    = dsd.substr(0, d_pos);
        std::vector<int> shapes;
        shapes.clear();

        dsd.erase(0, d_pos+1);

        if ( dsd == "all" ) {
            shapes.push_back( std::numeric_limits<int>::max() );
        } else {
            std::stringstream ss(dsd);
            int i;

            while (ss >> i)
            {
                GENAPT_LOG(SG_GENERAL, SG_ALERT, "Adding debug pavement " << i << " for " << icao );

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_pavements[icao] = shapes;
    }

    for (unsigned int i=0; i< taxiway_defs.size(); i++) {
        std::string dsd     = taxiway_defs[i];
        size_t d_pos   = dsd.find(":");

        std::string icao    = dsd.substr(0, d_pos);
        std::vector<int> shapes;
        shapes.clear();

        dsd.erase(0, d_pos+1);

        if ( dsd == "all" ) {
            shapes.push_back( std::numeric_limits<int>::max() );
        } else {
            std::stringstream ss(dsd);
            int i;

            while (ss >> i)
            {
                GENAPT_LOG(SG_GENERAL, SG_ALERT, "Adding debug taxiway " << i << " for " << icao );

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_taxiways[icao] = shapes;
    }

    for (unsigned int i=0; i< feature_defs.size(); i++) {
        std::string dsd     = feature_defs[i];
        size_t d_pos   = dsd.find(":");

        std::string icao    = dsd.substr(0, d_pos);
        std::vector<int> shapes;
        shapes.clear();

        dsd.erase(0, d_pos+1);

        if ( dsd == "all" ) {
            shapes.push_back( std::numeric_limits<int>::max() );
        } else {
            std::stringstream ss(dsd);
            int i;

            while (ss >> i)
            {
                GENAPT_LOG(SG_GENERAL, SG_ALERT, "Adding debug feature " << i << " for " << icao );

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_features[icao] = shapes;
    }
}

bool Scheduler::IsAirportDefinition( char* line, std::string icao )
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

void Scheduler::AddAirport( std::string icao )
{
    char            line[2048];
    long            cur_pos;
    bool            found = false;
    AirportInfo     ai;

    std::ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        GENAPT_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

    GENAPT_LOG( SG_GENERAL, SG_INFO, "Adding airport " << icao << " to parse list");
    while ( !in.eof() && !found ) 
    {
        // remember the position of this line
        cur_pos = in.tellg();

        // get a line
    	in.getline(line, 2048);

        // this is and airport definition - remember it
        if ( IsAirportDefinition( line, icao ) )
        {
            GENAPT_LOG( SG_GENERAL, SG_DEBUG, "Found airport " << icao << " at " << cur_pos );
         
            ai = AirportInfo( icao, cur_pos, gSnap );
            global_workQueue.push( ai );

            found = true;
        }
    }    
}

long Scheduler::FindAirport( std::string icao )
{
    char line[2048];
    long cur_pos = 0;
    bool found = false;

    std::ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        GENAPT_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

    GENAPT_LOG( SG_GENERAL, SG_DEBUG, "Finding airport " << icao );
    while ( !in.eof() && !found ) 
    {
        // remember the position of this line
        cur_pos = in.tellg();

        // get a line
    	in.getline(line, 2048);

        // this is and airport definition - remember it
        if ( IsAirportDefinition( line, icao ) )
        {
            GENAPT_LOG( SG_GENERAL, SG_DEBUG, "Found airport " << line << " at " << cur_pos );
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
    // retryList.push_back( *pai );
}

bool Scheduler::AddAirports( long start_pos, tg::Rectangle* boundingBox )
{
    char 	 line[2048];
    char*	 def;
    long 	 cur_pos;
    long	 cur_apt_pos = 0;
    std::string  cur_apt_name;
    char*    tok;
    int      code;
    bool 	 match;
    bool 	 done;

    done  = false;
    match = false;

    // start from current position, and push all airports where a runway start or end
    // lies within the given min/max coordinates

    std::ifstream in( filename.c_str() );
    if ( !in.is_open() )
    {
        GENAPT_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

    if (start_pos)
    {
        in.seekg(start_pos, std::ios::beg);
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
                        AirportInfo ai = AirportInfo( cur_apt_name, cur_apt_pos, gSnap );
                        global_workQueue.push( ai );
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
                        AirportInfo ai = AirportInfo( cur_apt_name, cur_apt_pos, gSnap );
                        global_workQueue.push( ai );
                    }
                    done = true;
                    break;

                case LAND_RUNWAY_CODE:
                    // if the the runway start / end  coords are within the rect,
                    // we have a winner
                    {
                        Runway* runway = new Runway(def);
                        if ( boundingBox->isInside(runway->GetStart()) ) {
                            match = true;
                        }
                        else if ( boundingBox->isInside(runway->GetEnd()) ) {
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
                        if ( boundingBox->isInside(runway->GetStart()) ) {
                            match = true;
                        }
                        else if ( boundingBox->isInside(runway->GetEnd()) ) {
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
                        if ( boundingBox->isInside(helipad->GetLoc()) ) {
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
    if ( global_workQueue.size() ) {
        return true;
    } else {
        return false;
    }
}

Scheduler::Scheduler(std::string& datafile, const std::string& root, const string_list& elev_src)
{
    filename        = datafile;
    work_dir        = root;
    elevation       = elev_src;

    std::ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        GENAPT_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }
}

void Scheduler::Schedule( int num_threads, std::string& summaryfile )
{
//    std::ofstream   csvfile;

    // open and truncate the summary file : monitor only appends 
//    csvfile.open( summaryfile.c_str(), std::ios_base::out | std::ios_base::trunc );
//    csvfile.close();

    std::vector<Parser *> parsers;
    for (int i=0; i<num_threads; i++) {
        Parser* parser = new Parser( filename, work_dir, elevation );
        // parser->set_debug();
        parser->start();
        parsers.push_back( parser );
    }

    while (!global_workQueue.empty()) {
        sleep(1);
    }

    // Then wait until they are finished
    for (unsigned int i=0; i<parsers.size(); i++) {
        parsers[i]->join();
    }
}
