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

#include <terragear/polygon_set/tg_polygon_set.hxx>
#include <terragear/tg_light.hxx>

class Helipad
{
public:
    Helipad(char* def);

#if 0
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
#endif
    void GetMainPolys( tgPolygonSetList& polys );
    void GetShoulderPolys( tgPolygonSetList& polys );
    void GetInnerBasePolys( tgPolygonSetList& polys );
    void GetOuterBasePolys( tgPolygonSetList& polys );
    void GetLights( tglightcontour_list& lights );
    
    SGGeod GetLoc()
    {
        return SGGeod::fromDeg(lon, lat);
    }

    bool GetsShoulder()
    {
        return (surface < 3) ? true : false;
    }

//    void BuildShoulder( tgPolygonSetList& rwy_polys,
//                        tgcontour_list& slivers,
//                        tgAccumulator& accum
//                      );

private:
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

    // generate an area for a runway with expansion specified in meters
    // (return result points in degrees)
    cgalPoly_Polygon gen_helipad_area_w_extend( double length_extend, double width_extend )
    {
        return ( gen_wgs84_area( GetLoc(), length + 2.0*length_extend, 0.0, 0.0, width + 2.0*width_extend, heading, false) );
    }

    tglightcontour_list gen_helipad_lights(double maxsize);
    void                build_helipad_shoulders( const cgalPoly_Polygon& outer_area );

    // storage for Shoulders - The superpolys are generated during
    // helipad construction, but not clipped until shoulder construction.
    tgPolygonSetList  shoulderPolys;

    tgPolygonSet WriteGeom( const cgalPoly_Polygon& area,
                            std::string material,
                            tgPolygonSetList& rwy_polys );
};

typedef std::vector <Helipad *> HelipadList;

#endif
