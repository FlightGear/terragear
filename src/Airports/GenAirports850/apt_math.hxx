#ifndef _APT_MATH_HXX_
#define _APT_MATH_HXX_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Geometry/point3d.hxx>
#include "texparams.hxx"


using std::string;


TGPolygon gen_wgs84_area( Point3D origin, double length_m, double displ1, double displ2, double width_m, double heading_deg, double alt_m, bool add_mid );

void gen_tex_section( const TGPolygon& runway,
                         double startl_pct, double endl_pct,
                         double startw_pct, double endw_pct,
                         double minu, double maxu, double minv, double maxv,
                         double heading, double width, double length,
                         const string& prefix,
                         const string& material,
                         superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         ClipPolyType *accum  );

#endif
