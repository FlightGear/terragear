#ifndef _AIRPORT_H_
#define _AIRPORT_H_

#include <stdio.h>
#include <stdlib.h>

#include "runway.hxx"
#include "closedpoly.hxx"
#include "linearfeature.hxx"

using std::string;

class Airport
{
public:
    Airport( int c, char* def);

    void AddRunway( Runway* runway )
    {
        runways.push_back( runway );
    }

    void AddPavement( ClosedPoly* pavement )
    {
        pavements.push_back( pavement );
    }

    void AddFeature( LinearFeature* feature )
    {
        features.push_back( feature );
    }

    void BuildOsg( osg::Group* airport );
    void BuildBtg( const string& root, const string_list& elev_src );

private:
    int     code;               // airport, heliport or sea port
    int     altitude;           // in meters
    string  icao;               // airport code
    string  description;        // description

    PavementList    pavements;
    FeatureList     features;
    RunwayList      runways;
};

typedef std::vector <Airport *> AirportList;

#endif
