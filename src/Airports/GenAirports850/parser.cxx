#include <sys/time.h>

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>

#include "parser.hxx"


bool Parser::IsAirportDefinition( char* line, string icao )
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
                airport = new Airport( code, line );
                if ( airport->GetIcao() == icao )
                {
                    match = true;
                }
                break;

            case HELIPORT_CODE:
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

void Parser::AddAirport( string icao )
{
    char line[2048];
    long cur_pos;
    bool found = false;

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
            parse_positions.push_back( cur_pos );
			airport_icaos.push_back( icao );
            found = true;
        }
    }    
}

long Parser::FindAirport( string icao )
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

void Parser::AddAirports( long start_pos, float min_lat, float min_lon, float max_lat, float max_lon )
{
        char 	 line[2048];
        char*	 def;
        long 	 cur_pos;
        long	 cur_apt_pos;
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
							parse_positions.push_back( cur_apt_pos );
							airport_icaos.push_back( cur_apt_name );
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
						parse_positions.push_back( cur_apt_pos );
						airport_icaos.push_back( cur_apt_name );
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
}

void Parser::Parse()
{
    char tmp[2048];
    int  i;
    struct timeval parse_start;
    struct timeval parse_end;
    struct timeval parse_time;
    struct timeval build_time;
    struct timeval clean_time;
    struct timeval triangulation_time;

    ifstream in( filename.c_str() );
    if ( !in.is_open() ) 
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << filename );
        exit(-1);
    }

    // for each position in parse_positions, parse an airport
    for (i=0; i<parse_positions.size(); i++)
    {
        SetState(STATE_NONE);
        in.clear();

        gettimeofday(&parse_start, NULL);

        SG_LOG( SG_GENERAL, SG_ALERT, "\n*******************************************************************" );
        SG_LOG( SG_GENERAL, SG_ALERT, "Parsing airport " << airport_icaos[i] << " at " << parse_positions[i] << " start time " << ctime(&parse_start.tv_sec) );
        in.seekg(parse_positions[i], ios::beg);

        while ( !in.eof() && (cur_state != STATE_DONE ) )
        {
        	in.getline(tmp, 2048);

            // Parse the line
            ParseLine(tmp);
        }
        gettimeofday(&parse_end, NULL);
        timersub(&parse_end, &parse_start, &parse_time);

        // write the airport BTG
        if (cur_airport)
        {
            cur_airport->BuildBtg( work_dir, elevation );

            cur_airport->GetBuildTime( build_time );
            cur_airport->GetCleanupTime( clean_time );
            cur_airport->GetTriangulationTime( triangulation_time );

            delete cur_airport;
            cur_airport = NULL;
        }


        SG_LOG( SG_GENERAL, SG_DEBUG, "Time to parse       " << parse_time.tv_sec << ":" << parse_time.tv_usec );
        SG_LOG( SG_GENERAL, SG_DEBUG, "Time to build       " << build_time.tv_sec << ":" << build_time.tv_usec );
        SG_LOG( SG_GENERAL, SG_DEBUG, "Time to clean up    " << clean_time.tv_sec << ":" << clean_time.tv_usec );
        SG_LOG( SG_GENERAL, SG_DEBUG, "Time to triangulate " << triangulation_time.tv_sec << ":" << triangulation_time.tv_usec );
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

    numParams = sscanf(line, "%d %f %f %ls", &st, &s, &th, desc);

    if (numParams == 4)
    {
        d = strstr(line,desc);
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Creating Closed Poly with st " << st << " smoothness " << s << " thexture heading " << th << " and description " << d);
    poly = new ClosedPoly(st, s, th, d);

    return poly;
}

ClosedPoly* Parser::ParseBoundary( char* line )
{
    ClosedPoly* poly;
    char  desc[256];
    char  *d = NULL;
    int   numParams;

    numParams = sscanf(line, "%ls", desc);

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
        cur_airport->SetBoundary( cur_boundary );
        cur_boundary = NULL;
    } 

    cur_state = state;
}

// TODO: This should be a loop here, and main should just pass the file name and airport code...
int Parser::ParseLine(char* line)
{
    char*  tok;
    int    code;

    BezNode* cur_node = NULL;

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
                SetState( STATE_PARSE_SIMPLE );
                SG_LOG(SG_GENERAL, SG_DEBUG, "Parsing heliport: " << line);
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

            case PAVEMENT_CODE:
                SetState( STATE_PARSE_PAVEMENT );
                cur_pavement  = ParsePavement( line );
                break;

            case LINEAR_FEATURE_CODE:
                SetState( STATE_PARSE_FEATURE );
                cur_feat = ParseFeature( line );
                break;

            case BOUNDRY_CODE:
                SetState( STATE_PARSE_BOUNDARY );
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
                        cur_feat->Finish();
                        cur_airport->AddFeature( cur_feat );
                    }
                    cur_feat = NULL;
                    SetState( STATE_NONE );
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
                            cur_feat->Finish();
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
                    SetState( STATE_NONE );
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

    return cur_state;
}

#if 0
osg::Group* Parser::CreateOsgGroup( void )
{
    osg::Group* airportNode = new osg::Group();
    int i;

    for (i=0; i<airports.size(); i++)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Add airport " << i << " to node");
        airports[i]->BuildOsg( airportNode );
    }

    return airportNode;
}
#endif

