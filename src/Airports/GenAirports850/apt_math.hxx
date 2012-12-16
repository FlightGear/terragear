#ifndef _APT_MATH_HXX_
#define _APT_MATH_HXX_

#include <terragear/tg_polygon.hxx>

tgContour gen_wgs84_area( SGGeod origin,
                          double length_m,
                          double displ1,
                          double displ2,
                          double width_m,
                          double heading_deg,
                          bool   add_mid );

tgContour gen_wgs84_area( SGGeod end1, SGGeod end2,
                          double length_ext,
                          double displ1, double displ2,
                          double width_m,
                          double heading_deg,
                          bool   add_mid );

tgContour gen_wgs84_rect( SGGeod origin, double heading, double length, double width );


#endif
