#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <Polygon/polygon.hxx>

class LightingObj
{
public:
    LightingObj(char* def);

    double lat;
    double lon;
    int type;
    double heading;
    double glideslope;
    char assoc_rw;


void BuildBtg( tglightcontour_list& lights );

};
typedef std::vector <LightingObj *> LightingObjList;

#endif
