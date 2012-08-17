#ifndef _RUNWAY_H_
#define _RUNWAY_H_

#include <stdio.h>
#include <stdlib.h>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>
#include <Geometry/point3d.hxx>

#include "apt_math.hxx"

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

    bool GetsShoulder()
    {
        return (rwy.surface < 3) ? true : false;
    }

    int BuildBtg( superpoly_list* rwy_polys, 
                  texparams_list* texparams, 
                  superpoly_list* rwy_lights, 
                  ClipPolyType* accum, 
                  poly_list& slivers, 
                  TGPolygon* apt_base, 
                  TGPolygon* apt_clearing,
                  bool make_shapefiles );

    void BuildShoulder( superpoly_list *rwy_polys,
                        texparams_list *texparams,
                        ClipPolyType *accum,
                        poly_list& slivers, 
                        TGPolygon* apt_base, 
                        TGPolygon* apt_clearing );
    
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

    TGRunway      rwy;
    std::string   material_prefix;

    // storage for Shoulders - The superpolys are generated during rwy construction,
    // but not clipped until shoulder construction.
    superpoly_list  shoulder_polys;
    texparams_list  shoulder_tps;

    // Build Helpers:
    // generate an area for a runway and include midpoints
    TGPolygon gen_runway_w_mid( double length_extend_m, double width_extend_m )
    {
         return ( gen_wgs84_area( GetStart(), GetEnd(), 2.0*length_extend_m, 0.0, 0.0, rwy.width + 2.0 * width_extend_m, rwy.heading, true) );
    }

    // generate an area for a runway with expansion specified in meters
    // (return result points in degrees)
    TGPolygon gen_runway_area_w_extend( double length_extend, double displ1, double displ2, double width_extend )
    {
        return ( gen_wgs84_area( GetStart(), GetEnd(), 2.0*length_extend, displ1, displ2, rwy.width + 2.0*width_extend, rwy.heading, false) );
    }

    void gen_rw_designation( TGPolygon poly, double heading, string rwname,
                             double &start_pct, double &end_pct,
                             superpoly_list* rwy_polys,
                             texparams_list* texparams,
                             ClipPolyType* accum,
                             poly_list& slivers,
                             bool make_shapefiles );

    // generate a section of runway
    void gen_runway_section( const TGPolygon& runway,
                      double startl_pct, double endl_pct,
                      double startw_pct, double endw_pct,
                      double minu, double maxu, double minv, double maxv,
                      double heading,
                      const string& material,
                      superpoly_list *rwy_polys,
                      texparams_list *texparams,
                      superpoly_list *shoulder_polys,
                      texparams_list *shoulder_tps,
                      ClipPolyType *accum,
                      poly_list& slivers,
                      bool make_shapefiles );

    // generate a section of shoulder
    void gen_shoulder_section( Point3D p0, Point3D p1, 
                               Point3D t0, Point3D t1, 
                               int side,
                               double heading,
                               double width,
                               std::string surface, 
                               TGSuperPoly& sp, TGTexParams& tp );

    void gen_simple_rwy( superpoly_list *rwy_polys, texparams_list *texparams, ClipPolyType *accum, poly_list& slivers );
    
    void gen_rwy( superpoly_list* rwy_polys,
                  texparams_list* texparams,
                  ClipPolyType* accum,
                  poly_list& slivers,
                  bool make_shapefiles );

    void gen_runway_lights( superpoly_list* lights );

    int get_thresh0(bool recip)
    {
        return (recip) ? 1 : 0;
    }

    int get_thresh1(bool recip)
    {
        return (recip) ? 0 : 1;
    }

    point_list gen_corners( double l_ext, double disp1, double disp2, double w_ext );
    Point3D gen_runway_light_vector( double angle, bool recip );
    superpoly_list gen_runway_edge_lights( bool recip );
    superpoly_list gen_runway_threshold_lights( const int kind, bool recip );
    superpoly_list gen_runway_center_line_lights( bool recip );
    TGSuperPoly gen_touchdown_zone_lights( bool recip );
    TGSuperPoly gen_reil( bool recip );
    superpoly_list gen_calvert( const string &kind, bool recip );
    superpoly_list gen_alsf( const string &kind, bool recip );
    TGSuperPoly gen_odals( bool recip );
    superpoly_list gen_ssalx( const string& kind, bool recip );
    superpoly_list gen_malsx( const string& kind, bool recip );
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
