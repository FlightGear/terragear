#ifndef _TAXIWAY_H_
#define _TAXIWAY_H_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>

#include "apt_math.hxx"

using std::string;

class Taxiway
{
public:

    Taxiway(char* def);

    int BuildBtg( tgpolygon_list& taxi_polys,
                  tglightcontour_list& taxi_lights,
                  tgcontour_list& slivers,
                  std::string& shapefile_name );

    int BuildBtg( tgpolygon_list& taxi_polys,
                  tglightcontour_list& taxi_lights,
                  tgcontour_list& slivers,
                  tgPolygon& apt_base,
                  tgPolygon& apt_clearing,
                  std::string& shapefile_name );
    
private:
    SGGeod  origin;
    double  heading;
    double  length;
    double  width;
    int     surface;
    char    lighting[8];

    tgContour taxi_contour;
};

typedef std::vector <Taxiway *> TaxiwayList;

#endif
