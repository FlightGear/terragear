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
    void BuildBtg( superpoly_list* heli_polys, texparams_list* texparams, superpoly_list* heli_lights, ClipPolyType* accum, poly_list& slivers, TGPolygon* apt_base, TGPolygon* apt_clearing );

    SGGeod GetLoc()
    {
        return SGGeod::fromDeg(heli.lon, heli.lat);
    }

    bool GetsShoulder()
    {
        return (heli.surface < 3) ? true : false;
    }

    void BuildShoulder( superpoly_list *rwy_polys,
                        texparams_list *texparams,
                        ClipPolyType *accum,
                        poly_list& slivers,
                        TGPolygon* apt_base,
                        TGPolygon* apt_clearing );

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
    TGPolygon gen_runway_area_w_extend( double alt_m, double length_extend, double displ1, double displ2, double width_extend )
    {
        return ( gen_wgs84_area( Point3D::fromSGGeod(GetLoc()), heli.length + 2.0*length_extend, displ1, displ2, heli.width + 2.0*width_extend, heli.heading, false) );
    }

    superpoly_list gen_helipad_lights(double maxsize);

    // storage for Shoulders - The superpolys are generated during
    // helipad construction, but not clipped until shoulder construction.
    superpoly_list  shoulder_polys;
    texparams_list  shoulder_tps;

    void WriteGeom( TGPolygon polygon, string material,
                        superpoly_list *rwy_polys,
                        ClipPolyType *accum,
                        poly_list& slivers );

};

typedef std::vector <Helipad *> HelipadList;

#endif
