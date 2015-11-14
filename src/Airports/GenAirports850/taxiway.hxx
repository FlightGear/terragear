#ifndef _TAXIWAY_H_
#define _TAXIWAY_H_

#include <terragear/polygon_set/tg_polygon_set.hxx>
#include <terragear/tg_light.hxx>

#include "apt_math.hxx"

class Taxiway
{
public:
    Taxiway(char* def);

    tgPolygonSet& GetPoly( void ) {
        return taxiwayPoly;
    }
    
    tgPolygonSet& GetInnerBasePoly( void ) {
        return innerBasePoly;
    }
    
    tgPolygonSet& GetOuterBasePoly( void ) {
        return outerBasePoly;
    }
    
    void GetLights( tglightcontour_list& l ) {
        l.insert( l.end(), lights.begin(), lights.end() );
    }
    
private:
    tgPolygonSet taxiwayPoly;
    tgPolygonSet innerBasePoly;
    tgPolygonSet outerBasePoly;    
    
    tglightcontour_list  lights;
};

typedef std::vector <Taxiway *> TaxiwayList;

#endif
