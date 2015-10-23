#ifndef _RUNWAY_H_
#define _RUNWAY_H_

#include <terragear/polygon_set/tg_polygon_set.hxx>
#include <terragear/polygon_set/tg_polygon_accumulator.hxx>
#include <terragear/tg_light.hxx>

#include "apt_math.hxx"
#include "linearfeature.hxx"

class Runway
{
public:

    Runway(char* def);

    SGGeod GetStart()
    {
        return SGGeod::fromDeg(lon[0], lat[0]);
    }

    SGGeod GetEnd()
    {
        return SGGeod::fromDeg(lon[1], lat[1]);
    }

    SGGeod GetMidpoint()
    {
        return SGGeod::fromDeg( (lon[0]+lon[1])/2.0f, (lat[0]+lat[1])/2.0f);
    }

    bool GetsShoulder()
    {
        return (surface < 3) ? true : false;
    }

    void GetMainPolys( Airport* ap, tgPolygonSetList& polys );
    void GetMarkingPolys(tgPolygonSetList& polys );
    void GetCapPolys(tgPolygonSetList& polys );
    void GetShoulderPolys( tgPolygonSetList& polys );
    void GetInnerBasePolys( tgPolygonSetList& polys );
    void GetOuterBasePolys( tgPolygonSetList& polys );

    void GetLights( tglightcontour_list& lights );
    
private:
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

    std::string   material_prefix;

    // storage for the runway - very simple without marking polys
    tgPolygonSetList runway_polys;
    
    // storage for the runway markings
    tgPolygonSetList marking_polys;
    tgPolygonSetList cap_polys;
    
    // storage for Shoulders - The superpolys are generated during rwy construction,
    // but not clipped until shoulder construction.
    tgPolygonSetList shoulder_polys;

    // Build Helpers:
    // generate an area for a runway and include midpoints
    cgalPoly_Polygon gen_runway_w_mid( double length_extend_m, double width_extend_m )
    {
         return ( gen_wgs84_area( GetStart(), GetEnd(), 2.0*length_extend_m, 0.0, 0.0, width + 2.0 * width_extend_m, heading, true) );
    }

    // generate an area for a runway with expansion specified in meters
    // (return result points in degrees)
    cgalPoly_Polygon gen_runway_area_w_extend( double length_extend, double displ1, double displ2, double width_extend )
    {
        return ( gen_wgs84_area( GetStart(), GetEnd(), 2.0*length_extend, displ1, displ2, width + 2.0*width_extend, heading, false) );
    }

    // new API
    cgalPoly_Polygon GetSectionBB( const cgalPoly_Polygon& runway, double startl_pct, double endl_pct, double startw_pct, double endw_pct, double heading  );

    
    LinearFeature* gen_perpendicular_marking_feature( Airport* ap, const SGGeod& start_ref, double heading, double start_dist, double length, double width, int mark );
    LinearFeature* gen_paralell_marking_feature( Airport* ap, const SGGeod& start_ref, double heading, double start_dist, double length, double offset, int mark );
    LinearFeature* gen_chevron_feature( Airport* ap, const SGGeod& start_ref, double heading, double start_dist, double length, double width, double offset, int mark );
    
    void   gen_designation_polygon( Airport* ap, int rwidx, const SGGeod& start_ref, double heading, double start_dist, double length, double width, double offset, const std::string& mark );
    
    void   gen_base( Airport* ap, int rwidx, const SGGeod& start, const SGGeod& end, double heading, double dist, bool with_shoulders );
    void   gen_border( Airport* ap, int rwidx, const SGGeod& start, const SGGeod& end, double heading, double dist );
    SGGeod gen_disp_thresh( Airport* ap, int rwidx, const SGGeod& start, double length, double heading );
    void   gen_threshold( Airport* ap, int rwidx, const SGGeod& start, double heading  );
    void   gen_stopway( Airport* ap, int rwidx, const SGGeod& start, double length, double heading );
    SGGeod gen_designation( Airport* ap, int rwidx, const SGGeod& start, double heading );
    double gen_centerline( Airport* ap, int rwidx, const cgalPoly_Polygon& runway, double start_pct, double heading );
    
    void   gen_feature( Airport* ap, int rwidx, 
                        const cgalPoly_Polygon& runway,
                        double startl_pct, double endl_pct,
                        double startw_pct, double endw_pct,
                        double minu, double maxu, double minv, double maxv,
                        double heading,
                        const std::string& material );
                             
    // generate a section of runway without shoulders
    void gen_section( Airport* ap, int rwidx, 
                      const cgalPoly_Polygon& runway,
                      double startl_pct, double endl_pct,
                      double startw_pct, double endw_pct,
                      double minu, double maxu, double minv, double maxv,
                      double heading,
                      const std::string& material,
                      bool with_shoulders );
    
    // generate a section of shoulder
    tgPolygonSet gen_shoulder_section( Airport* ap, int rwidx, 
                                       cgalPoly_Point& p0, cgalPoly_Point& p1,
                                       cgalPoly_Point& t0, cgalPoly_Point& t1, 
                                       int side,
                                       double heading,
                                       double width,
                                       std::string surface );

    void gen_simple_rwy(Airport* ap);
    void gen_full_rwy(Airport* ap);

    void gen_runway_lights( tglightcontour_list& lights );

    int get_thresh0(bool recip)
    {
        return (recip) ? 1 : 0;
    }

    int get_thresh1(bool recip)
    {
        return (recip) ? 0 : 1;
    }

    bool GetsThreshold(bool recip)
    {
        return (threshold[get_thresh0(recip)] > 60.0) ? true : false;
    }

    SGVec3f             gen_runway_light_vector( float angle, bool recip );
    tglightcontour_list gen_runway_edge_lights( bool recip );
    tglightcontour_list gen_runway_threshold_lights( const int kind, bool recip );
    tglightcontour_list gen_runway_center_line_lights( bool recip );
    tgLightContour      gen_touchdown_zone_lights( bool recip );
    tgLightContour      gen_reil( const int kind, bool recip );
    tglightcontour_list gen_calvert( const std::string &kind, bool recip );
    tglightcontour_list gen_alsf( const std::string &kind, bool recip );
    tgLightContour      gen_odals( const int kind, bool recip );
    tglightcontour_list gen_ssalx( const std::string& kind, bool recip );
    tglightcontour_list gen_malsx( const std::string& kind, bool recip );
    
    // Runway Marking Polys
    FeatureList features;    
};

typedef std::vector <Runway *> RunwayList;


class WaterRunway
{
public:
    WaterRunway(char* def);

    tgContour GetBuoys();

    SGGeod GetStart(void)
    {
        return SGGeod::fromDeg( lon[0], lat[0] );
    }

    SGGeod GetEnd(void)
    {
        return SGGeod::fromDeg( lon[1], lat[1] );
    }

private:
    double  width;
    int     buoys;
    char    rwnum[2][16];
    double  lat[2];
    double  lon[2];
};
typedef std::vector <WaterRunway *> WaterRunwayList;

#endif
