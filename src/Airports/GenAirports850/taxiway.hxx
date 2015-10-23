#ifndef _TAXIWAY_H_
#define _TAXIWAY_H_

#include <terragear/polygon_set/tg_polygon_set.hxx>
#include <terragear/tg_light.hxx>

#include "apt_math.hxx"

class Taxiway
{
public:

    Taxiway(char* def);

#if 0
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
#endif
    void GetPolys( tgPolygonSetList& polys );
    void GetInnerBasePolys( tgPolygonSetList& polys );
    void GetOuterBasePolys( tgPolygonSetList& polys );
    void GetLights(tglightcontour_list& lights);
    
private:
    SGGeod  origin;
    double  heading;
    double  length;
    double  width;
    int     surface;
    char    lighting[6];

    cgalPoly_Polygon taxi_contour;
};

typedef std::vector <Taxiway *> TaxiwayList;

#endif
