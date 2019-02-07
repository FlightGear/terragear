#ifndef _RUNWAY_H_
#define _RUNWAY_H_

#include <memory>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_accumulator.hxx>
#include <terragear/tg_light.hxx>

#include "apt_math.hxx"

class Runway
{
public:

    explicit Runway(char* def);

    SGGeod GetStart()
    {
        return SGGeod::fromDeg(rwy.lon[0], rwy.lat[0]);
    }

    SGGeod GetEnd()
    {
        return SGGeod::fromDeg(rwy.lon[1], rwy.lat[1]);
    }

    SGGeod GetMidpoint()
    {
        return SGGeod::fromDeg( (rwy.lon[0]+rwy.lon[1])/2.0f, (rwy.lat[0]+rwy.lat[1])/2.0f);
    }

    bool GetsShoulder()
    {
        return (rwy.surface < 3) ? true : false;
    }

    int BuildBtg( tgpolygon_list& rwy_polys,
                  tglightcontour_list& rwy_lights,
                  tgcontour_list& slivers,
                  tgAccumulator& accum,
                  std::string& shapefile_name );

    int BuildBtg( tgpolygon_list& rwy_polys,
                  tglightcontour_list& rwy_lights,
                  tgcontour_list& slivers,
                  tgpolygon_list& apt_base_polys,
                  tgpolygon_list& apt_clearing_polys,
                  tgAccumulator& accum,
                  std::string& shapefile_name );

    void BuildShoulder( tgpolygon_list& rwy_polys,
                        tgcontour_list& slivers,
                        tgAccumulator& accum );

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
    tgpolygon_list  shoulder_polys;

    // Build Helpers:
    // generate an area for a runway and include midpoints
    tgContour gen_runway_w_mid( double length_extend_m, double width_extend_m )
    {
         return ( gen_wgs84_area( GetStart(), GetEnd(), 2.0*length_extend_m, 0.0, 0.0, rwy.width + 2.0 * width_extend_m, rwy.heading, true) );
    }

    // generate an area for a runway with expansion specified in meters
    // (return result points in degrees)
    tgContour gen_runway_area_w_extend( double length_extend, double displ1, double displ2, double width_extend )
    {
        return ( gen_wgs84_area( GetStart(), GetEnd(), 2.0*length_extend, displ1, displ2, rwy.width + 2.0*width_extend, rwy.heading, false) );
    }

    void gen_rw_designation( tgPolygon poly, double heading, std::string rwname,
                             double &start_pct, double &end_pct,
                             tgpolygon_list& rwy_polys,
                             tgcontour_list& slivers,
                             tgAccumulator& accum,
                             std::string& shapefile_name );

    // generate a section of runway with shoulders
    void gen_runway_section( const tgPolygon& runway,
                      double startl_pct, double endl_pct,
                      double startw_pct, double endw_pct,
                      double minu, double maxu, double minv, double maxv,
                      double heading,
                      const std::string& material,
                      tgpolygon_list& rwy_polys,
                      tgpolygon_list& shoulder_polys,
                      tgcontour_list& slivers,
                      tgAccumulator& accum,
                      std::string& shapefile_name );

    // generate a section of runway without shoulders
    void gen_runway_section( const tgPolygon& runway,
                      double startl_pct, double endl_pct,
                      double startw_pct, double endw_pct,
                      double minu, double maxu, double minv, double maxv,
                      double heading,
                      const std::string& material,
                      tgpolygon_list& rwy_polys,
                      tgcontour_list& slivers,
                      tgAccumulator& accum,
                      std::string& shapefile_name );

    // generate a section of shoulder
    tgPolygon gen_shoulder_section( SGGeod& p0, SGGeod& p1,
                                    SGGeod& t0, SGGeod& t1, 
                                    int side,
                                    double heading,
                                    double width,
                                    std::string surface );

    void gen_simple_rwy( tgpolygon_list& rwy_polys, tgcontour_list& slivers, tgAccumulator& accum );
    
    void gen_rwy( tgpolygon_list& rwy_polys,
                  tgcontour_list& slivers,
                  tgAccumulator& accum,
                  std::string& shapefile_name );

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
        return (rwy.threshold[get_thresh0(recip)] > 60.0) ? true : false;
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
};

typedef std::vector <std::shared_ptr<Runway>> RunwayList;


class WaterRunway
{
public:
    explicit WaterRunway(char* def);

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
typedef std::vector <std::shared_ptr<WaterRunway>> WaterRunwayList;

#endif
