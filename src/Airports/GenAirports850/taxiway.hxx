#ifndef _TAXIWAY_H_
#define _TAXIWAY_H_

#include <memory>

#include <terragear/tg_light.hxx>
#include <terragear/tg_polygon.hxx>
#include <terragear/tg_accumulator.hxx>

#include "apt_math.hxx"

class Taxiway
{
public:

    explicit Taxiway(char* def);

    int BuildBtg( tgpolygon_list& taxi_polys,
                  tglightcontour_list& taxi_lights,
                  tgcontour_list& slivers,
                  tgAccumulator& accum,
                  std::string& shapefile_name );

    int BuildBtg( tgpolygon_list& taxi_polys,
                  tglightcontour_list& taxi_lights,
                  tgcontour_list& slivers,
                  tgpolygon_list& apt_base_polys,
                  tgpolygon_list& apt_clearing_polys,
                  tgAccumulator& accum,
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

typedef std::vector<std::shared_ptr<Taxiway>> TaxiwayList;

#endif
