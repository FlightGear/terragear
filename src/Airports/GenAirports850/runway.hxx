#ifndef _RUNWAY_H_
#define _RUNWAY_H_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Geometry/point3d.hxx>

#include "apt_math.hxx"
#include "texparams.hxx"

using std::string;

class Runway
{
public:

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

    int BuildBtg( float alt_m, superpoly_list* rwy_polys, texparams_list* texparams, superpoly_list* rwy_lights, ClipPolyType* accum, TGPolygon* apt_base, TGPolygon* apt_clearing );
    
private:
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

    TGRunway rwy;

    // Build Helpers:
    // generate an area for a runway and include midpoints
    TGPolygon gen_runway_w_mid( double alt_m, double length_extend_m, double width_extend_m )
    {
         return ( gen_wgs84_area( Point3D(GetStart()), Point3D(GetEnd()), rwy.length + 2.0*length_extend_m, 0.0, 0.0, rwy.width + 2.0 * width_extend_m, rwy.heading, alt_m, true) );
    }

    // generate an area for a runway with expansion specified in meters
    // (return result points in degrees)
    TGPolygon gen_runway_area_w_extend( double alt_m, double length_extend, double displ1, double displ2, double width_extend )
    {
        return ( gen_wgs84_area( Point3D(GetStart()), Point3D(GetEnd()), rwy.length + 2.0*length_extend, displ1, displ2, rwy.width + 2.0*width_extend, rwy.heading, alt_m, false) );
    }


    void gen_rw_designation( const std::string& material,
                             TGPolygon poly, double heading, string rwname,
                             double &start_pct, double &end_pct,
                             superpoly_list* rwy_polys,
                             texparams_list* texparams,
                             ClipPolyType* accum );

    // generate the runway overrun area
    void gen_runway_overrun( const TGPolygon& runway_half,
                             int rwhalf,
                             const std::string& prefix,
                             superpoly_list* rwy_polys,
                             texparams_list* texparams,
                             ClipPolyType* accum );

    // generate a section of runway
    void gen_runway_section( const TGPolygon& runway,
                             double startl_pct, double endl_pct,
                             double startw_pct, double endw_pct,
                             double minu, double maxu, double minv, double maxv,
                             double heading,
                             const std::string& prefix,
                             const std::string& material,
                             superpoly_list* rwy_polys,
                             texparams_list* texparams,
                             ClipPolyType* accum  );

    void gen_simple_rwy( double alt_m, const string& material, superpoly_list *rwy_polys, texparams_list *texparams, ClipPolyType *accum );
    void gen_rwy( double alt_m,
                  const std::string& material,
                  superpoly_list* rwy_polys,
                  texparams_list* texparams,
                  ClipPolyType* accum );

    void gen_rw_marking( const TGPolygon& runway,
                         double &start1_pct, double &end1_pct,
                         double heading,
                         const string& material,
                         superpoly_list* rwy_polys,
                         texparams_list* texparams,
                         ClipPolyType* accum, int marking);

    void gen_runway_lights( float alt_m, superpoly_list* lights );

    Point3D gen_runway_light_vector( double angle, bool recip );
    superpoly_list gen_runway_edge_lights( bool recip );
    superpoly_list gen_taxiway_edge_lights( const int kind, bool recip );
    superpoly_list gen_runway_threshold_lights( const int kind, float alt_m, bool recip );
    superpoly_list gen_runway_center_line_lights( bool recip );
    TGSuperPoly gen_touchdown_zone_lights( float alt_m, bool recip );
    TGSuperPoly gen_reil( float alt_m, bool recip );
    superpoly_list gen_calvert( float alt_m, const string &kind, bool recip );
    superpoly_list gen_alsf( float alt_m, const string &kind, bool recip );
    TGSuperPoly gen_odals( float alt_m, bool recip );
    superpoly_list gen_ssalx( float alt_m, const string& kind, bool recip );
    superpoly_list gen_malsx( float alt_m, const string& kind, bool recip );
};
typedef std::vector <Runway *> RunwayList;


class WaterRunway
{
public:
    WaterRunway(char* def);

    Point3D GetStart(void)
    {
        return ( Point3D( lon[0], lat[0], 0.0f ));
    }

    Point3D GetEnd(void)
    {
        return ( Point3D( lon[1], lat[1], 0.0f ));
    }

    bool HasBuoys()
    {
        if (buoys == 1)
        return true;
        else
        return false;
    }

    double  width;
    int     buoys;
    char    rwnum[2][16];
    double  lat[2];
    double  lon[2];

    TGPolygon GetNodes();
};
typedef std::vector <WaterRunway *> WaterRunwayList;

#endif
