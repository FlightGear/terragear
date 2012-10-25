#ifndef _APT_MATH_HXX_
#define _APT_MATH_HXX_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>
#include <Geometry/point3d.hxx>


using std::string;

tgContour gen_wgs84_area( SGGeod origin,
                          double length_m,
                          double displ1,
                          double displ2,
                          double width_m,
                          double heading_deg,
                          bool   add_mid );

tgContour gen_wgs84_area( SGGeod end1, SGGeod end2,
                          double length_m,
                          double displ1, double displ2,
                          double width_m,
                          double heading_deg,
                          bool   add_mid );

tgContour gen_wgs84_rect( double lat, double lon, double heading, double length, double width );


#endif
