#include <ctime>

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

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>
#include <simgear/timing/timestamp.hxx>

#include "parser.hxx"

bool Parser::GetAirportDefinition( char* line, string& icao )
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
                icao = airport->GetIcao();
                match = true;
                break;

            default:
                break;
        }
    }

    return match;
}

void Parser::set_debug( std::string path, std::vector<string> runway_defs,
                                          std::vector<string> pavement_defs,
                                          std::vector<string> taxiway_defs,
                                          std::vector<string> feature_defs )
{
    SG_LOG(SG_GENERAL, SG_ALERT, "Set debug Path " << path);

    debug_path = path;

    /* Find any ids for our tile */
    for (unsigned int i=0; i< runway_defs.size(); i++) {
        string dsd     = runway_defs[i];
        size_t d_pos   = dsd.find(":");

        string icao    = dsd.substr(0, d_pos);
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
                SG_LOG(SG_GENERAL, SG_ALERT, "Adding debug runway " << i);

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_runways[icao] = shapes;
    }

    for (unsigned int i=0; i< pavement_defs.size(); i++) {
        string dsd     = pavement_defs[i];
        size_t d_pos   = dsd.find(":");

        string icao    = dsd.substr(0, d_pos);
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
                SG_LOG(SG_GENERAL, SG_ALERT, "Adding debug pavement " << i);

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_pavements[icao] = shapes;
    }

    for (unsigned int i=0; i< taxiway_defs.size(); i++) {
        string dsd     = taxiway_defs[i];
        size_t d_pos   = dsd.find(":");

        string icao    = dsd.substr(0, d_pos);
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
                SG_LOG(SG_GENERAL, SG_ALERT, "Adding debug taxiway " << i);

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_taxiways[icao] = shapes;
    }

    for (unsigned int i=0; i< feature_defs.size(); i++) {
        string dsd     = feature_defs[i];
        size_t d_pos   = dsd.find(":");

        string icao    = dsd.substr(0, d_pos);
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
                SG_LOG(SG_GENERAL, SG_ALERT, "Adding debug feature " << i);

                shapes.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_features[icao] = shapes;
    }
}

void Parser::Parse( long pos )
{
    char line[2048];
    string icao;

    SGTimeStamp parse_start;
    SGTimeStamp parse_end;
    SGTimeStamp parse_time;
    SGTimeStamp build_time;
    SGTimeStamp clean_time;
    SGTimeStamp triangulation_time;
    time_t      log_time;

    ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }
    in.seekg(pos, ios::beg);

    // get a line
 	in.getline(line, 2048);

    // Verify this is and airport definition and get the icao
    if( GetAirportDefinition( line, icao ) ) {
        SG_LOG( SG_GENERAL, SG_INFO, "Found airport " << icao << " at " << pos );

        // Start parse at pos
        SetState(STATE_NONE);
        in.clear();

        parse_start.stamp();
        log_time = time(0);
        SG_LOG( SG_GENERAL, SG_ALERT, "\n*******************************************************************" );
        SG_LOG( SG_GENERAL, SG_ALERT, "Start airport " << icao << " at " << pos << ": start time " << ctime(&log_time) );

        in.seekg(pos, ios::beg);
        while ( !in.eof() && (cur_state != STATE_DONE ) )
        {
        	in.getline(line, 2048);

            // Parse the line
            ParseLine(line);
        }

        parse_end.stamp();
        parse_time = parse_end - parse_start;

        // write the airport BTG
        if (cur_airport)
        {
            cur_airport->set_debug( debug_path, debug_runways, debug_pavements, debug_taxiways, debug_features );
            cur_airport->BuildBtg( work_dir, elevation );

            cur_airport->GetBuildTime( build_time );
            cur_airport->GetCleanupTime( clean_time );
            cur_airport->GetTriangulationTime( triangulation_time );

            delete cur_airport;
            cur_airport = NULL;
        }

        log_time = time(0);
        SG_LOG( SG_GENERAL, SG_ALERT, "Finished airport " << icao << 
            " : parse " << parse_time << " : build " << build_time << 
            " : clean " << clean_time << " : tesselate " << triangulation_time );
    } else {
        SG_LOG( SG_GENERAL, SG_INFO, "Not an airport at pos " << pos << " line is: " << line );  
    }
}

BezNode* Parser::ParseNode( int type, char* line, BezNode* prevNode )
{
    double lat, lon;
    double ctrl_lat, ctrl_lon;
    int feat_type1, feat_type2;
    BezNode *curNode = NULL;

    bool hasCtrl = false;
    bool close = false;
    bool term = false;
    bool hasFeat1 = false;
    bool hasFeat2 = false;
    int  numParams;

    switch(type)
    {
        case NODE_CODE:
            hasCtrl = false;
            close   = false;
            term    = false;
            break;

        case BEZIER_NODE_CODE:
            hasCtrl = true;
            close   = false;
            term    = false;
            break;

        case CLOSE_NODE_CODE:
            hasCtrl = false;
            close   = true;
            term    = false;
            break;

        case CLOSE_BEZIER_NODE_CODE:
            hasCtrl = true;
            close   = true;
            term    = false;
            break;

        case TERM_NODE_CODE:
            hasCtrl = false;
            close   = false;
            term    = true;
            break;

        case TERM_BEZIER_NODE_CODE:
            hasCtrl = true;
            close   = false;
            term    = true;
            break;
    }

    // parse the line
    if (hasCtrl)
    {
        numParams = sscanf(line, "%lf %lf %lf %lf %d %d", &lat, &lon, &ctrl_lat, &ctrl_lon, &feat_type1, &feat_type2);
        if (numParams > 4)
        {
            hasFeat1 = true;
        }
        if (numParams > 5)
        {
            hasFeat2 = true;
        }
    }
    else
    {
        numParams = sscanf(line, "%lf %lf %d %d", &lat, &lon, &feat_type1, &feat_type2);
        if (numParams > 2)
        {
            hasFeat1 = true;
        }
        if (numParams > 3)
        {
            hasFeat2 = true;
        }
    }

    if ( (prevNode) && (prevNode->IsAt( lat, lon )) )
    {
        curNode = prevNode;

        // editing already existent node
        if (hasCtrl)
        {        
            // we have ctrl info -> set it
            curNode->SetNextCp(ctrl_lat,ctrl_lon);
        }
        else
        {
            // no control info - make sure we clear anything in next
            curNode->ClearNextCp();
        }
    }

    // if this is a new node, add it - as first part never has prev cp
    if (curNode == NULL)
    {
        if (hasCtrl)
        {
            curNode = new BezNode(lat, lon, ctrl_lat, ctrl_lon);
        }
        else
        {
            curNode = new BezNode(lat, lon);
        }
    }

    if (hasFeat1)
    {
        if (feat_type1 < 100)
        {
            curNode->SetMarking( feat_type1 );
        }
        else
        {
            curNode->SetLighting( feat_type1 );
        }
    }

    if (hasFeat2)
    {
        if (feat_type2 < 100)
        {
            curNode->SetMarking( feat_type2 );
        }
        else
        {
            curNode->SetLighting( feat_type2 );
        }
    }

    curNode->SetTerm( term );
    curNode->SetClose( close );

    return curNode;
}

LinearFeature* Parser::ParseFeature( char* line )
{
    LinearFeature* feature;

    if (strlen( line ))
    {
        feature = new LinearFeature(line, 0.0f);
    }
    else
    {
        feature = new LinearFeature(NULL, 0.0f);
    }
        
    SG_LOG(SG_GENERAL, SG_DEBUG, "Creating Linear Feature with desription \"" << line << "\"");

    return feature;
}

ClosedPoly* Parser::ParsePavement( char* line )
{
    ClosedPoly* poly;
    int   st = 0;
    float s = 0.0f;
    float th = 0.0f;
    char  desc[256];
    char  *d = NULL;
    int   numParams;

    numParams = sscanf(line, "%d %f %f %s", &st, &s, &th, desc);

    if (numParams == 4)
    {
        d = strstr(line,desc);
        SG_LOG(SG_GENERAL, SG_DEBUG, "Creating Closed Poly with st " << st << " smoothness " << s << " thexture heading " << th << " and description " << d);
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Creating Closed Poly with st " << st << " smoothness " << s << " thexture heading " << th );
    }

    poly = new ClosedPoly(st, s, th, d);

    return poly;
}

ClosedPoly* Parser::ParseBoundary( char* line )
{
    ClosedPoly* poly;
    char  desc[256];
    char  *d = NULL;
    int   numParams;

    numParams = sscanf(line, "%s", desc);

    if (numParams == 1)
    {
        d = strstr(line,desc);
    }
    else
    {
        d = (char *)"none";
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Creating Closed Poly for airport boundary : " << d);
    poly = new ClosedPoly(d);

    return poly;
}

int Parser::SetState( int state )
{
    // if we are currently parsing pavement, the oly way we know we are done 
    // is when we get a non-node line to parse
    if ( cur_airport && cur_state == STATE_PARSE_PAVEMENT )
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Closing and Adding pavement");
        cur_pavement->Finish();
        cur_airport->AddPavement( cur_pavement );
        cur_pavement = NULL;
    } 

    if ( cur_airport && cur_state == STATE_PARSE_BOUNDARY )
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Closing and Adding boundary");
        cur_boundary->Finish();
        cur_airport->AddBoundary( cur_boundary );
        cur_boundary = NULL;
    } 

    cur_state = state;

    return cur_state;
}

// TODO: This should be a loop here, and main should just pass the file name and airport code...
int Parser::ParseLine(char* line)
{
    char*  tok;
    int    code;

    BezNode* cur_node = NULL;

    if (*line != '#')
    {
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
                    if (cur_state == STATE_NONE)
                    {
                        SetState( STATE_PARSE_SIMPLE );
                        SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing land airport: " << line);
                        cur_airport = new Airport( code, line );
                    }
                    else
                    {
                        SetState( STATE_DONE );
                    }
                    break;
                case HELIPORT_CODE:
                    if (cur_state == STATE_NONE)
                    {
                        SetState( STATE_PARSE_SIMPLE );
                        SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing heliport: " << line);
                        cur_airport = new Airport( code, line );
                    }
                    else
                    {
                        SetState( STATE_DONE );
                    }
                    break;
    
                case LAND_RUNWAY_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing runway: " << line);
                    cur_runway = new Runway(line);
                    if (cur_airport)
                    {
                        cur_airport->AddRunway( cur_runway );
                    }
                    break;
    
                case WATER_RUNWAY_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing water runway: " << line);
                    cur_waterrunway = new WaterRunway(line);
                    if (cur_airport)
                    {
                        cur_airport->AddWaterRunway( cur_waterrunway );
                    }
                    break;
                case HELIPAD_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing helipad: " << line);
                    cur_helipad = new Helipad(line);
                    if (cur_airport)
                    {
                        cur_airport->AddHelipad( cur_helipad );
                    }
                    break;
    
                case TAXIWAY_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing taxiway: " << line);
                    cur_taxiway = new Taxiway(line);
                    if (cur_airport)
                    {
                        cur_airport->AddTaxiway( cur_taxiway );
                    }
                    break;
    
                case PAVEMENT_CODE:
                    SetState( STATE_PARSE_PAVEMENT );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing pavement: " << line);
                    cur_pavement  = ParsePavement( line );
                    break;
    
                case LINEAR_FEATURE_CODE:
                    SetState( STATE_PARSE_FEATURE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Linear Feature: " << line);
                    cur_feat = ParseFeature( line );
                    break;
    
                case BOUNDRY_CODE:
                    SetState( STATE_PARSE_BOUNDARY );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing Boundary: " << line);
                    cur_boundary = ParseBoundary( line ); 
                    break;
    
                case NODE_CODE:
                case BEZIER_NODE_CODE:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing node: " << line);
                    cur_node = ParseNode( code, line, prev_node );
    
                    if ( prev_node && (cur_node != prev_node) )
                    {
                        // prev node is done - process it\n");
                        if ( cur_state == STATE_PARSE_PAVEMENT )
                        {
                            cur_pavement->AddNode( prev_node );
                        }
                        else if ( cur_state == STATE_PARSE_FEATURE )
                        {
                            cur_feat->AddNode( prev_node );
                        }
                        else if ( cur_state == STATE_PARSE_BOUNDARY )
                        {
                            cur_boundary->AddNode( prev_node );
                        }
                    }
    
                    prev_node = cur_node;
                    break;
    
                case CLOSE_NODE_CODE:
                case CLOSE_BEZIER_NODE_CODE:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing close loop node: " << line);
                    cur_node = ParseNode( code, line, prev_node );
    
                    if ( cur_state == STATE_PARSE_PAVEMENT )
                    {
                        if (cur_node != prev_node)
                        {
                            cur_pavement->AddNode( prev_node );
                            cur_pavement->AddNode( cur_node );
                        }
                        else
                        {
                            cur_pavement->AddNode( cur_node );
                        }
                        cur_pavement->CloseCurContour();
                    }
                    else if ( cur_state == STATE_PARSE_BOUNDARY )
                    {
                        if (cur_node != prev_node)
                        {
                            cur_boundary->AddNode( prev_node );
                            cur_boundary->AddNode( cur_node );
                        }
                        else
                        {
                            cur_boundary->AddNode( cur_node );
                        }
                        cur_boundary->CloseCurContour();
                    }
                    else if ( cur_state == STATE_PARSE_FEATURE )
                    {
                        if (cur_node != prev_node)
                        {
                            cur_feat->AddNode( prev_node );
                            cur_feat->AddNode( cur_node );
                        }
                        else
                        {
                            cur_feat->AddNode( cur_node );
                        }
                        if (cur_airport)
                        {
                            cur_feat->Finish( true, cur_airport->NumFeatures() );
                            cur_airport->AddFeature( cur_feat );
                        }
                        cur_feat = NULL;
                        SetState( STATE_PARSE_SIMPLE );
                    }
                    prev_node = NULL;
                    cur_node  = NULL;
                    break;
    
                case TERM_NODE_CODE:
                case TERM_BEZIER_NODE_CODE:
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing termination node: " << line);
    
                    if ( cur_state == STATE_PARSE_FEATURE )
                    {
                        // we have some bad data - termination nodes right after the
                        // linear feature declaration - can't do anything with a
                        // single point - detect and delete.
                        if ( prev_node )
                        {
                            cur_node = ParseNode( code, line, prev_node );
    
                            if (cur_node != prev_node)
                            {
                                cur_feat->AddNode( prev_node );
                                cur_feat->AddNode( cur_node );
                            }
                            else
                            {
                                cur_feat->AddNode( cur_node );
                            }
                            if (cur_airport)
                            {
                                cur_feat->Finish( false, cur_airport->NumFeatures()  );
                                cur_airport->AddFeature( cur_feat );
                            }
                        }
                        else
                        {
                            SG_LOG(SG_GENERAL, SG_ALERT, "Parsing termination node with no previous nodes!!!" );
    
                            // this feature is bogus...
                            delete cur_feat;
                        }
                        cur_feat = NULL;
                        SetState( STATE_PARSE_SIMPLE );
                    }
                    prev_node = NULL;
                    cur_node  = NULL;
                    break;
    
                case AIRPORT_VIEWPOINT_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing viewpoint: " << line);
                    break;
                case AIRPLANE_STARTUP_LOCATION_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing airplane startup location: " << line);
                    break;
                case LIGHT_BEACON_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing light beacon: " << line);
                    cur_beacon = new Beacon(line);
                    cur_airport->AddBeacon( cur_beacon );                                
                    break;
                case WINDSOCK_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing windsock: " << line);
                    cur_windsock = new Windsock(line);
                    cur_airport->AddWindsock( cur_windsock );                                
                    break;
                case TAXIWAY_SIGN:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing taxiway sign: " << line);
                    cur_sign = new Sign(line);
                    cur_airport->AddSign( cur_sign );                                
                    break;
                case LIGHTING_OBJECT:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing lighting object: " << line);
                    cur_object = new LightingObj(line);
                    cur_airport->AddObj( cur_object );
                    break;
                case COMM_FREQ1_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 1: " << line);
                    break;
                case COMM_FREQ2_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 2: " << line);
                    break;
                case COMM_FREQ3_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 3: " << line);
                    break;
                case COMM_FREQ4_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 4: " << line);
                    break;
                case COMM_FREQ5_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 5: " << line);
                    break;
                case COMM_FREQ6_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 6: " << line);
                    break;
                case COMM_FREQ7_CODE:
                    SetState( STATE_PARSE_SIMPLE );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 7: " << line);
                    break;
                case END_OF_FILE :
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Reached end of file");
                    SetState( STATE_DONE );
                    break;
            }
        }
    }
    
    return cur_state;
}
