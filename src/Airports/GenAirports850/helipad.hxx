// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.


#ifndef _HELIPAD_HXX
#define _HELIPAD_HXX

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>

#include "texparams.hxx"

class Helipad
{
public:
    Helipad(char* def);
    void BuildBtg( float alt_m, superpoly_list* heli_polys, texparams_list* texparams, superpoly_list* heli_lights, TGPolygon* accum, TGPolygon* apt_base, TGPolygon* apt_clearing );

private:
    struct TGRunway {
    // data for whole runway
    int     surface;
    int     shoulder;
    int     edge_lights;

    double  width;
    double  length;
    double  heading;
    double  smoothness;

    // data for each end
    char    designator[16];
    double  lat;
    double  lon;

    int     marking;
    };

    TGRunway heli;

    // generate an area for a runway with expansion specified in meters
    // (return result points in degrees)
    TGPolygon gen_runway_area_w_extend( double alt_m, double length_extend, double displ1, double displ2, double width_extend )
    {
        return ( gen_wgs84_area(Point3D(heli.lon, heli.lat, 0.0f), heli.length + 2.0*length_extend, displ1, displ2, heli.width + 2.0*width_extend, heli.heading, alt_m, false) );
    }

    superpoly_list gen_helipad_lights(void);

};

typedef std::vector <Helipad *> HelipadList;

#endif
