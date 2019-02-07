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

#include <memory>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_accumulator.hxx>
#include <terragear/tg_light.hxx>

class Helipad
{
public:
    explicit Helipad(char* def);

    void BuildBtg( tgpolygon_list& heli_polys,
                   tglightcontour_list& heli_lights,
                   tgcontour_list& slivers,
                   tgAccumulator& accum );

    void BuildBtg( tgpolygon_list& heli_polys,
                   tglightcontour_list& heli_lights,
                   tgcontour_list& slivers,
                   tgpolygon_list& apt_base_polys,
                   tgpolygon_list& apt_clearing_polys,
                   tgAccumulator& accum );

    SGGeod GetLoc()
    {
        return SGGeod::fromDeg(heli.lon, heli.lat);
    }

    bool GetsShoulder()
    {
        return (heli.surface < 3) ? true : false;
    }

    void BuildShoulder( tgpolygon_list& rwy_polys,
                        tgcontour_list& slivers,
                        tgAccumulator& accum
                      );

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
    tgContour gen_helipad_area_w_extend( double length_extend, double width_extend )
    {
        return ( gen_wgs84_area( GetLoc(), heli.length + 2.0*length_extend, 0.0, 0.0, heli.width + 2.0*width_extend, heli.heading, false) );
    }

    tglightcontour_list gen_helipad_lights(double maxsize);
    void                build_helipad_shoulders( const tgContour& outer_area );

    // storage for Shoulders - The superpolys are generated during
    // helipad construction, but not clipped until shoulder construction.
    tgpolygon_list  shoulder_polys;

    tgPolygon WriteGeom( const tgContour& area,
                         std::string material,
                         tgpolygon_list& rwy_polys,
                         tgcontour_list& slivers );

};

typedef std::vector<std::shared_ptr<Helipad>> HelipadList;

#endif
