#ifndef _PARSER_H_
#define _PARSER_H_

#include <iostream>
#include <fstream>
#include <memory>

#include <simgear/threads/SGThread.hxx>

#include "scheduler.hxx"
#include "beznode.hxx"
#include "closedpoly.hxx"
#include "linearfeature.hxx"
#include "runway.hxx"
#include "airport.hxx"

#define STATE_INIT                  (0)
#define STATE_NONE                  (1)
#define STATE_PARSE_SIMPLE          (2)
#define STATE_PARSE_BOUNDARY        (3)
#define STATE_PARSE_PAVEMENT        (4)
#define STATE_PARSE_FEATURE         (5)
#define STATE_DONE                  (10)

#define MAXLINE (256)

#define LAND_AIRPORT_CODE               (1)
#define TAXIWAY_CODE                    (10)
#define SEA_AIRPORT_CODE                (16)
#define HELIPORT_CODE                   (17)

#define LAND_RUNWAY_CODE                (100)
#define WATER_RUNWAY_CODE               (101)
#define HELIPAD_CODE                    (102)

#define PAVEMENT_CODE                   (110)
#define LINEAR_FEATURE_CODE             (120)
#define BOUNDARY_CODE                   (130)

#define NODE_CODE                       (111)
#define BEZIER_NODE_CODE                (112)
#define CLOSE_NODE_CODE                 (113)
#define CLOSE_BEZIER_NODE_CODE          (114)
#define TERM_NODE_CODE                  (115)
#define TERM_BEZIER_NODE_CODE           (116)

#define AIRPORT_VIEWPOINT_CODE          (14)
#define AIRPLANE_STARTUP_LOCATION_CODE  (15)
#define LIGHT_BEACON_CODE               (18)
#define WINDSOCK_CODE                   (19)
#define TAXIWAY_SIGN                    (20)
#define LIGHTING_OBJECT                 (21)

#define COMM_FREQ1_CODE                 (50)               
#define COMM_FREQ2_CODE                 (51)               
#define COMM_FREQ3_CODE                 (52)               
#define COMM_FREQ4_CODE                 (53)               
#define COMM_FREQ5_CODE                 (54)               
#define COMM_FREQ6_CODE                 (55)               
#define COMM_FREQ7_CODE                 (56)               

#define END_OF_FILE                     (99)

#define AIRPORT_TRAFFIC_FLOW                    (1000)
#define TRAFFIC_FLOW_WIND_RULE                  (1001)
#define TRAFFIC_FLOW_MIN_CEILING_RULE           (1002)
#define TRAFFIC_FLOW_MIN_VISIBILITY_RULE        (1003)
#define TRAFFIC_FLOW_TIME_RULE                  (1004)

#define RWY_ARR_DEP_CONSTRAINTS                 (1100)
#define VFR_TRAFFIC_PATTERN                     (1101)

#define TAXI_ROUTE_NETWORK_HEADER               (1200)
#define TAXI_ROUTE_NETWORK_NODE                 (1201)
#define TAXI_ROUTE_NETWORK_EDGE                 (1202)
#define TAXI_ROUTE_NETWORK_OBSOLETE             (1203)
#define TAXI_ROUTE_EDGE_ACTIVE_ZONE             (1204)
#define TAXI_ROUTE_EDGE_CONTROL                 (1205)
#define TAXI_ROUTE_EDGE_GROUND_VEHICLES         (1206)

#define START_UP_LOCATION                       (1300)
#define START_UP_LOCATION_METADATA              (1301)
#define AIRPORT_IDENTIFICATION_METADATA         (1302)

#define TRUCK_PARKING_LOCATION                  (1400)
#define TRUCK_DESTINATION_LOCATION              (1401)


class Parser : public SGThread
{
public:
    Parser(const std::string& datafile, const std::string& root, const string_list& elev_src ) :
        prev_node(nullptr),
        filename(datafile),
        elevation(elev_src),
        work_dir(root),
        cur_airport(nullptr),
        cur_taxiway(nullptr),
        cur_runway(nullptr),
        cur_waterrunway(nullptr),
        cur_helipad(nullptr),
        cur_pavement(nullptr),
        cur_boundary(nullptr),
        cur_feat(nullptr),
        cur_object(nullptr),
        cur_windsock(nullptr),
        cur_beacon(nullptr),
        cur_sign(nullptr)
    {
        cur_state = STATE_NONE;
    }

    // Debug
    void            set_debug( const std::string& path, std::vector<std::string> runway_defs,
                                                 std::vector<std::string> pavement_defs,
                                                 std::vector<std::string> taxiway_defs,
                                                 std::vector<std::string> feature_defs );

private:
    virtual void    run();

    bool            IsAirportDefinition( char* line, std::string icao );
    bool            GetAirportDefinition( char* line, std::string& icao );

    int             SetState( int state );

    std::shared_ptr<BezNode>        ParseNode( int type, char* line, std::shared_ptr<BezNode> prevNode );
    std::shared_ptr<LinearFeature>  ParseFeature( char* line );
    std::shared_ptr<ClosedPoly>     ParsePavement( char* line );
    std::shared_ptr<ClosedPoly>     ParseBoundary( char* line );

    int             ParseLine( char* line );

    std::shared_ptr<BezNode>        prev_node;
    int             cur_state;
    std::string     filename;
    string_list     elevation;
    std::string     work_dir;

    // a polygon conists of an array of contours 
    // (first is outside boundry, remaining are holes)
    std::shared_ptr<Airport>        cur_airport;
    std::shared_ptr<Taxiway>        cur_taxiway;
    std::shared_ptr<Runway>         cur_runway;
    std::shared_ptr<WaterRunway>    cur_waterrunway;
    std::shared_ptr<Helipad>        cur_helipad;
    std::shared_ptr<ClosedPoly>     cur_pavement;
    std::shared_ptr<ClosedPoly>     cur_boundary;
    std::shared_ptr<LinearFeature>  cur_feat;
    std::shared_ptr<LightingObj>    cur_object;
    std::shared_ptr<Windsock>       cur_windsock;
    std::shared_ptr<Beacon>         cur_beacon;
    std::shared_ptr<Sign>           cur_sign;

    // debug
    std::string     debug_path;
    debug_map       debug_runways;
    debug_map       debug_pavements;
    debug_map       debug_taxiways;
    debug_map       debug_features;
};

#endif