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

    int BuildBtg( tgpolygon_list& taxi_polys,
                  tglightcontour_list& taxi_lights,
                  tgcontour_list& slivers,
                  bool make_shapefiles );

    int BuildBtg( tgpolygon_list& taxi_polys,
                  tglightcontour_list& taxi_lights,
                  tgcontour_list& slivers,
                  tgPolygon& apt_base,
                  tgPolygon& apt_clearing,
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
