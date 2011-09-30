#ifndef _RUNWAY_H_
#define _RUNWAY_H_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Geometry/point3d.hxx>

#include "texparams.hxx"

#include <osg/Group>

using std::string;

class Runway
{
public:
    struct TGRunway {
    // data for whole runway
    int     surface;
    int     shoulder;
    int     centerline_lights;
    int     edge_lights;
    int     dist_remain_signs;

    double  width;
    double  length;
    double  heading;
    double  smoothness;

    // data for each end
    char    rwnum[2][16];
    double  lat[2];
    double  lon[2];
    double  threshold[2];
    double  overrun[2];

    int     marking[2];
    int     approach_lights[2];
    int     tz_lights[2];
    int     reil[2];
    };
typedef std::vector < TGRunway > runway_list;

TGRunway rwy;

    Runway(char* def);

    bool IsPrecision()
    {
        return true;
    }

    Point3D GetStart(void)
    {
        return ( Point3D( rwy.lon[0], rwy.lat[0], 0.0f ));
    }

    Point3D GetEnd(void)
    {
        return ( Point3D( rwy.lon[1], rwy.lat[1], 0.0f ));
    }

    Point3D GetMidpoint(void)
    {
        return ( Point3D( (rwy.lon[0]+rwy.lon[1])/2.0f, (rwy.lat[0]+rwy.lat[1])/2.0f, 0.0f) );
    }

    int BuildOsg( osg::Group* airport );
    int BuildBtg( float alt_m, superpoly_list* rwy_polys, texparams_list* texparams, TGPolygon* accum, TGPolygon* apt_base, TGPolygon* apt_clearing );
    
private:

    // Build Helpers
    TGPolygon gen_wgs84_area( Point3D origin, double length_m, double displ1, double displ2, double width_m, double heading_deg, double alt_m, bool add_mid );
    TGPolygon gen_runway_w_mid( double alt_m, double length_extend_m, double width_extend_m );
    TGPolygon gen_runway_area_w_extend( double alt_m, double length_extend, double displ1, double displ2, double width_extend );

    void gen_number_block( const std::string& material,
                           TGPolygon poly, double heading, int num,
                           double start_pct, double end_pct,
                           superpoly_list *rwy_polys,
                           texparams_list *texparams,
                           TGPolygon *accum );

    // generate the runway overrun area
    void gen_runway_overrun( const TGPolygon& runway_half,
                             int rwhalf,
                             const std::string& prefix,
                             superpoly_list *rwy_polys,
                             texparams_list *texparams,
                             TGPolygon* accum );

    // generate a section of runway
    void gen_runway_section( const TGPolygon& runway,
                             double startl_pct, double endl_pct,
                             double startw_pct, double endw_pct,
                             double minu, double maxu, double minv, double maxv,
                             double heading,
                             const std::string& prefix,
                             const std::string& material,
                             superpoly_list *rwy_polys,
                             texparams_list *texparams,
                             TGPolygon *accum  );

    void gen_simple_rwy( double alt_m, const string& material, superpoly_list *rwy_polys, texparams_list *texparams, TGPolygon *accum );
    void gen_rwy( double alt_m,
                  const std::string& material,
                  superpoly_list *rwy_polys,
                  texparams_list *texparams,
                  TGPolygon *accum );

    void gen_rw_marking( const TGPolygon& runway,
                         double &start1_pct, double &end1_pct,
                         double heading,
                         const string& material,
                         superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         TGPolygon *accum, int marking);
};

typedef std::vector <Runway *> RunwayList;

#endif
