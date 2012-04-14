#ifndef _TAXIWAY_H_
#define _TAXIWAY_H_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>
#include <Geometry/point3d.hxx>

#include "apt_math.hxx"

using std::string;

class Taxiway
{
public:

    Taxiway(char* def);

    int BuildBtg( superpoly_list* taxi_polys, 
                  texparams_list* texparams, 
                  superpoly_list* taxi_lights, 
                  ClipPolyType* accum, 
                  poly_list& slivers, 
                  TGPolygon* apt_base, 
                  TGPolygon* apt_clearing,
                  bool make_shapefiles );
    
private:
    double  lat;
    double  lon;
    double  heading;
    double  length;
    double  width;
    int     surface;
    char    lighting[8];
};

typedef std::vector <Taxiway *> TaxiwayList;

#endif
