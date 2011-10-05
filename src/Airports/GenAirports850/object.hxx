#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Geometry/point3d.hxx>

#include "texparams.hxx"

using std::string;

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


void BuildBtg( int alt_m, superpoly_list* lights );

};
typedef std::vector <LightingObj *> LightingObjList;
#endif