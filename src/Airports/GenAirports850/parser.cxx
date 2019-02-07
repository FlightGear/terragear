#include <ctime>

#include <simgear/debug/logstream.hxx>
#include <simgear/io/iostreams/sgstream.hxx>
#include <simgear/timing/timestamp.hxx>

#include "parser.hxx"

bool Parser::GetAirportDefinition( char* line, std::string& icao )
{
    bool match = false;

    // Get the number code
    char* tok = strtok(line, " \t\r\n");

    if (tok)
    {
        line += strlen(tok)+1;
        int code = atoi(tok);

        switch(code)
        {
            case LAND_AIRPORT_CODE: 
            case SEA_AIRPORT_CODE:
            case HELIPORT_CODE:
            {
                Airport ap( code, line );
                icao = ap.GetIcao();
                match = true;
            }
                break;

            default:
                break;
        }
    }

    return match;
}

void Parser::set_debug( const std::string& path, std::vector<std::string> runway_defs,
                                          std::vector<std::string> pavement_defs,
                                          std::vector<std::string> taxiway_defs,
                                          std::vector<std::string> feature_defs )
{
    TG_LOG(SG_GENERAL, SG_ALERT, "Set debug Path " << path);

    debug_path = path;

    /* Find any ids for our tile */
    for (unsigned int i=0; i< runway_defs.size(); i++) {
        std::string dsd = runway_defs[i];
        size_t d_pos   = dsd.find(":");

        std::string icao = dsd.substr(0, d_pos);
        std::vector<int> shapes;
        shapes.clear();

        dsd.erase(0, d_pos+1);

        if ( dsd == "all" ) {
            shapes.push_back( std::numeric_limits<int>::max() );
        } else {
            std::stringstream ss(dsd);
            int idx;
            while (ss >> idx)
            {
                TG_LOG(SG_GENERAL, SG_ALERT, "Adding debug runway " << idx);

                shapes.push_back(idx);

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
            int idx;
            while (ss >> idx)
            {
                TG_LOG(SG_GENERAL, SG_ALERT, "Adding debug pavement " << idx);

                shapes.push_back(idx);

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
            int idx;
            while (ss >> idx)
            {
                TG_LOG(SG_GENERAL, SG_ALERT, "Adding debug taxiway " << idx);

                shapes.push_back(idx);

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
            int idx;
            while (ss >> idx)
            {
                TG_LOG(SG_GENERAL, SG_ALERT, "Adding debug feature " << idx);

                shapes.push_back(idx);

                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        debug_features[icao] = shapes;
    }
}

void Parser::run()
{
    char line[2048];
    std::string icao;

    SGTimeStamp parse_start;
    SGTimeStamp parse_end;
    SGTimeStamp parse_time;
    SGTimeStamp build_time;
    SGTimeStamp clean_time;
    SGTimeStamp triangulation_time;
    time_t      log_time;
    long        pos;

    std::ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        TG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

    // as long as we have airports to parse, do so
    while (!global_workQueue.empty()) {
        AirportInfo ai = global_workQueue.pop();

        if ( ai.GetIcao() == "NZSP" ) {
            continue;
        }

        DebugRegisterPrefix( ai.GetIcao() );
        pos = ai.GetPos();
        in.seekg(pos, std::ios::beg);

        // get a line
        in.getline(line, 2048);

        // Verify this is an airport definition and get the icao
        if( GetAirportDefinition( line, icao ) ) {
            TG_LOG( SG_GENERAL, SG_INFO, "Found airport " << icao << " at " << pos );

            // Start parse at pos
            SetState(STATE_NONE);
            in.clear();

            parse_start.stamp();
            log_time = time(0);
            TG_LOG( SG_GENERAL, SG_ALERT, "\n*******************************************************************" );
            TG_LOG( SG_GENERAL, SG_ALERT, "Start airport " << icao << " at " << pos << ": start time " << ctime(&log_time) );

            in.seekg(pos, std::ios::beg);
            while ( !in.eof() && (cur_state != STATE_DONE ) ) {
                in.getline(line, 2048);

                // Parse the line
                ParseLine(line);
            }

            parse_end.stamp();
            parse_time = parse_end - parse_start;

            // write the airport BTG
            if (cur_airport) {
                cur_airport->set_debug( debug_path, debug_runways, debug_pavements, debug_taxiways, debug_features );
                cur_airport->BuildBtg( work_dir, elevation );

                cur_airport->GetBuildTime( build_time );
                cur_airport->GetCleanupTime( clean_time );
                cur_airport->GetTriangulationTime( triangulation_time );

                cur_airport = nullptr;
            }

            log_time = time(0);
            TG_LOG( SG_GENERAL, SG_ALERT, "Finished airport " << icao << 
                " : parse " << parse_time << " : build " << build_time << 
                " : clean " << clean_time << " : tesselate " << triangulation_time );
        } else {
            TG_LOG( SG_GENERAL, SG_INFO, "Not an airport at pos " << pos << " line is: " << line );  
        }
    }
}

std::shared_ptr<BezNode> Parser::ParseNode( int type, char* line, std::shared_ptr<BezNode> prevNode )
{
    double lat, lon;
    double ctrl_lat, ctrl_lon;
    int feat_type1, feat_type2;
    std::shared_ptr<BezNode> curNode = nullptr;

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
    if (curNode == nullptr)
    {
        if (hasCtrl)
        {
            curNode = std::make_shared<BezNode>(lat, lon, ctrl_lat, ctrl_lon);
        }
        else
        {
            curNode = std::make_shared<BezNode>(lat, lon);
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

    return std::move(curNode);
}

std::shared_ptr<LinearFeature> Parser::ParseFeature( char* line )
{
    std::shared_ptr<LinearFeature> feature;

    if (strlen( line ))
    {
        feature = std::make_shared<LinearFeature>(line, 0.0f);
    }
    else
    {
        feature = std::make_shared<LinearFeature>(nullptr, 0.0f);
    }

    TG_LOG(SG_GENERAL, SG_DEBUG, "Creating Linear Feature with description \"" << line << "\"");

    return std::move(feature);
}

std::shared_ptr<ClosedPoly> Parser::ParsePavement( char* line )
{
    std::shared_ptr<ClosedPoly> poly;
    int   st = 0;       // surface type
    float s = 0.0f;     // smoothness
    float th = 0.0f;    // texture heading
    char  desc[256];    // description
    char  *d = nullptr;
    int   numParams;

    numParams = sscanf(line, "%d %f %f %255s", &st, &s, &th, desc);

    if (numParams == 4)
    {
        d = strstr(line,desc);
        TG_LOG(SG_GENERAL, SG_DEBUG, "Creating Closed Poly with st " << st << " smoothness " << s << " texture heading " << th << " and description " << d);
    }
    else
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Creating Closed Poly with st " << st << " smoothness " << s << " texture heading " << th);
    }

    poly = std::make_shared<ClosedPoly>(st, s, th, d);

    return std::move(poly);
}

std::shared_ptr<ClosedPoly> Parser::ParseBoundary( char* line )
{
    std::shared_ptr<ClosedPoly> poly;
    char  desc[256];
    char  *d = nullptr;
    int   numParams;

    numParams = sscanf(line, "%255s", desc);

    if (numParams == 1)
    {
        d = strstr(line,desc);
    }
    else
    {
        d = (char *)"none";
    }

    TG_LOG(SG_GENERAL, SG_DEBUG, "Creating Closed Poly for airport boundary : " << d);
    poly = std::make_shared<ClosedPoly>(d);

    return std::move(poly);
}

int Parser::SetState( int state )
{
    // if we are currently parsing pavement, the oly way we know we are done 
    // is when we get a non-node line to parse
    if ( cur_airport && cur_state == STATE_PARSE_PAVEMENT )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Closing and Adding pavement");
        cur_pavement->Finish();
        cur_airport->AddPavement( cur_pavement );
        cur_pavement = nullptr;
    } 

    if ( cur_airport && cur_state == STATE_PARSE_BOUNDARY )
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Closing and Adding boundary");
        cur_boundary->Finish();
        cur_airport->AddBoundary( cur_boundary );
        cur_boundary = nullptr;
    } 

    cur_state = state;

    return cur_state;
}

// TODO: This should be a loop here, and main should just pass the file name and airport code...
int Parser::ParseLine(char* line)
{
    std::shared_ptr<BezNode> cur_node = nullptr;

    if (*line == '#')
        return cur_state;
    
    // Get the number code
    char* tok = strtok(line, " \t\r\n");

    if (tok)
    {
        line += strlen(tok)+1;
        int code = atoi(tok);

        switch(code)
        {
            case LAND_AIRPORT_CODE: 
            case SEA_AIRPORT_CODE:
                if (cur_state == STATE_NONE)
                {
                    SetState( STATE_PARSE_SIMPLE );
                    TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing land airport: " << line);
                    cur_airport = std::make_shared<Airport>( code, line );
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
                    TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing heliport: " << line);
                    cur_airport = std::make_shared<Airport>( code, line );
                }
                else
                {
                    SetState( STATE_DONE );
                }
                break;

            case LAND_RUNWAY_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing runway: " << line);
                cur_runway = std::make_shared<Runway>(line);
                if (cur_airport)
                {
                    cur_airport->AddRunway( cur_runway );
                }
                break;

            case WATER_RUNWAY_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing water runway: " << line);
                cur_waterrunway = std::make_shared<WaterRunway>(line);
                if (cur_airport)
                {
                    cur_airport->AddWaterRunway( cur_waterrunway );
                }
                break;
            case HELIPAD_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing helipad: " << line);
                cur_helipad = std::make_shared<Helipad>(line);
                if (cur_airport)
                {
                    cur_airport->AddHelipad( cur_helipad );
                }
                break;

            case TAXIWAY_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing taxiway: " << line);
                cur_taxiway = std::make_shared<Taxiway>(line);
                if (cur_airport)
                {
                    cur_airport->AddTaxiway( cur_taxiway );
                }
                break;

            case PAVEMENT_CODE:
                SetState( STATE_PARSE_PAVEMENT );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing pavement: " << line);
                cur_pavement  = ParsePavement( line );
                break;

            case LINEAR_FEATURE_CODE:
                SetState( STATE_PARSE_FEATURE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Linear Feature: " << line);
                cur_feat = ParseFeature( line );
                break;

            case BOUNDARY_CODE:
                SetState( STATE_PARSE_BOUNDARY );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing Boundary: " << line);
                cur_boundary = ParseBoundary( line ); 
                break;

            case NODE_CODE:
            case BEZIER_NODE_CODE:
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing node: " << line);
                cur_node = ParseNode( code, line, prev_node );

                if ( prev_node && (cur_node != prev_node) )
                {
                    // prev node is done - process it
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
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing close loop node: " << line);
                cur_node = ParseNode( code, line, prev_node );

                if ( cur_state == STATE_PARSE_PAVEMENT && prev_node )
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
                    cur_feat = nullptr;
                    SetState( STATE_PARSE_SIMPLE );
                }
                prev_node = nullptr;
                cur_node  = nullptr;
                break;

            case TERM_NODE_CODE:
            case TERM_BEZIER_NODE_CODE:
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing termination node: " << line);

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
                        TG_LOG(SG_GENERAL, SG_ALERT, "Parsing termination node with no previous nodes!!!" );
                    }
                    
                    cur_feat = nullptr;
                    SetState( STATE_PARSE_SIMPLE );
                }
                prev_node = nullptr;
                cur_node  = nullptr;
                break;

            case AIRPORT_VIEWPOINT_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing viewpoint: " << line);
                break;
            case AIRPLANE_STARTUP_LOCATION_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing airplane startup location: " << line);
                break;
            case LIGHT_BEACON_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing light beacon: " << line);
                cur_beacon = std::make_shared<Beacon>(line);
                cur_airport->AddBeacon( cur_beacon );                                
                break;
            case WINDSOCK_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing windsock: " << line);
                cur_windsock = std::make_shared<Windsock>(line);
                cur_airport->AddWindsock( cur_windsock );                                
                break;
            case TAXIWAY_SIGN:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing taxiway sign: " << line);
                cur_sign = std::make_shared<Sign>(line);
                cur_airport->AddSign( cur_sign );                                
                break;
            case LIGHTING_OBJECT:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing lighting object: " << line);
                cur_object = std::make_shared<LightingObj>(line);
                cur_airport->AddObj( cur_object );
                break;
            case COMM_FREQ1_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 1: " << line);
                break;
            case COMM_FREQ2_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 2: " << line);
                break;
            case COMM_FREQ3_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 3: " << line);
                break;
            case COMM_FREQ4_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 4: " << line);
                break;
            case COMM_FREQ5_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 5: " << line);
                break;
            case COMM_FREQ6_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 6: " << line);
                break;
            case COMM_FREQ7_CODE:
                SetState( STATE_PARSE_SIMPLE );
                TG_LOG(SG_GENERAL, SG_DEBUG, "Parsing commfreq 7: " << line);
                break;
            case END_OF_FILE :
                TG_LOG(SG_GENERAL, SG_DEBUG, "Reached end of file");
                SetState( STATE_DONE );
                break;
        }
    }
    
    return cur_state;
}
