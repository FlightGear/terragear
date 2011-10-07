#ifndef _LINKED_OBJECTS_H_
#define _LINKED_OBJECTS_H_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Geometry/point3d.hxx>

#include "texparams.hxx"

using std::string;

class Windsock
{
public:
    Windsock(char* def);

    double lat;
    double lon;
    int lit;

    Point3D GetLoc()
    {
        return Point3D( lon, lat, 0.0f );
    }

    bool IsLit()
    {
        return (lit == 1) ? true : false;
    }
};

typedef std::vector <Windsock *> WindsockList;


class Beacon
{
public:
    Beacon(char* def);

    double lat;
    double lon;
    int code;

    Point3D GetLoc()
    {
        return Point3D( lon, lat, 0.0f );
    }

    int GetCode()
    {
        return code;
    }
};

typedef std::vector <Beacon *> BeaconList;

class Sign
{
public:
    Sign(char* def);

    double lat;
    double lon;
    double heading;
    int    reserved;
    int    size;
    string sgn_def;

    Point3D GetLoc()
    {
        return Point3D( lon, lat, 0.0f );
    }

    double GetHeading()
    {
        return heading;
    }

    string GetDefinition()
    {
        return sgn_def;
    }
};

typedef std::vector <Sign *> SignList;

#endif
