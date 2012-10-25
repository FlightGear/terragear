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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.


#ifndef _HELIPAD_HXX
#define _HELIPAD_HXX

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>

class Helipad
{
public:
    Helipad(char* def);

    void BuildBtg( tgpolygon_list& heli_polys,
                   tglightcontour_list& heli_lights,
                   tgcontour_list& slivers );

    void BuildBtg( tgpolygon_list& heli_polys,
                   tglightcontour_list& heli_lights,
                   tgcontour_list& slivers,
                   tgPolygon& apt_base,
                   tgPolygon& apt_clearing );

    SGGeod GetLoc()
    {
        return SGGeod::fromDeg(heli.lon, heli.lat);
    }

    bool GetsShoulder()
    {
        return (heli.surface < 3) ? true : false;
    }

    void BuildShoulder( tgpolygon_list& rwy_polys,
                        tgcontour_list& slivers );

    void BuildShoulder( tgpolygon_list& rwy_polys,
                        tgcontour_list& slivers,
                        tgPolygon& apt_base,
                        tgPolygon& apt_clearing );

private:
    struct TGRunway {
    // data for helipad
    char    designator[16];
    double  lat;
    double  lon;
    double  heading;
    double  length;
    double  width;
    int     surface;
    int     marking;
    int     shoulder;
    double  smoothness;
    int     edge_lights;
    };

    TGRunway heli;


    // generate an area for a runway with expansion specified in meters
    // (return result points in degrees)
    tgContour gen_runway_area_w_extend( double alt_m, double length_extend, double displ1, double displ2, double width_extend )
    {
        return ( gen_wgs84_area( GetLoc(), heli.length + 2.0*length_extend, displ1, displ2, heli.width + 2.0*width_extend, heli.heading, false) );
    }

    tglightcontour_list gen_helipad_lights(double maxsize);

    // storage for Shoulders - The superpolys are generated during
    // helipad construction, but not clipped until shoulder construction.
    tgpolygon_list  shoulder_polys;

    tgPolygon WriteGeom( const tgContour& area,
                         string material,
                         tgpolygon_list& rwy_polys,
                         tgcontour_list& slivers );

};

typedef std::vector <Helipad *> HelipadList;

#endif
