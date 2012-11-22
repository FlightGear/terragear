#ifndef _TAXIWAY_H_
#define _TAXIWAY_H_

#include <Polygon/polygon.hxx>

#include "apt_math.hxx"

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
    char    lighting[6];

    tgContour taxi_contour;
    void GenLights(tglightcontour_list& rwy_lights);
};

typedef std::vector <Taxiway *> TaxiwayList;

#endif
