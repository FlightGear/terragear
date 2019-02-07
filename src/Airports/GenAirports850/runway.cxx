
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/SGGeodesy.hxx>

#include <terragear/tg_polygon.hxx>

#include "global.hxx"
#include "apt_math.hxx"
#include "beznode.hxx"
#include "runway.hxx"

extern int nudge;

Runway::Runway(char* definition)
{
    double az2;

    // runway format:
    //      width   surface  shoulder  smoothness centerline lights  edge lighting distance remaining signs
    //      46.02   2        1         0.00       1                  2             1 
    //     
    //      runway number  runway end lat runway end long   threshold  overrun  markings  approach lighting
    //      09L            33.63470475     -084.44798671    0.00       120.09   3         7 
    // 
    //      touchdown zone lighting  runway end identifier lights
    //      0                        1                              
    //
    //      runway number  runway end lat runway end long   threshold  overrun  markings  approach lighting
    //      27R             33.63469907   -084.40893004     0.00       120.09   3         6 
    //
    //      touchdown zone lighting  runway end identifier lights
    //      0                        1

    // Parse the line
    //      46.02   2   1 0.00 1 2 1 09L  33.63470475 -084.44798671    0.00  120.09 3  7 0 1 27R  33.63469907 -084.40893004    0.00  120.09 3  6 0 1

    std::istringstream ss(definition);
    ss  >> rwy.width
        >> rwy.surface
        >> rwy.shoulder
        >> rwy.smoothness
        >> rwy.centerline_lights
        >> rwy.edge_lights
        >> rwy.dist_remain_signs
        >> rwy.rwnum[0]
        >> rwy.lat[0]
        >> rwy.lon[0]
        >> rwy.threshold[0]
        >> rwy.overrun[0]
        >> rwy.marking[0]
        >> rwy.approach_lights[0]
        >> rwy.tz_lights[0]
        >> rwy.reil[0]
        >> rwy.rwnum[1]
        >> rwy.lat[1]
        >> rwy.lon[1]
        >> rwy.threshold[1]
        >> rwy.overrun[1]
        >> rwy.marking[1]
        >> rwy.approach_lights[1]
        >> rwy.tz_lights[1]
        >> rwy.reil[1];

    // calculate runway heading and length (used a lot)
    SGGeodesy::inverse( GetStart(), GetEnd(), rwy.heading, az2, rwy.length );

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read runway: (" << rwy.lon[0] << "," << rwy.lat[0] <<
            ") to (" << rwy.lon[1] << "," << rwy.lat[1] <<
            ") heading: " << rwy.heading <<
            " length: " << rwy.length <<
            " width: " << rwy.width );
} 


WaterRunway::WaterRunway(char* definition)
{
    std::istringstream ss(definition);
    ss  >> width
        >> buoys
        >> rwnum[0]
        >> lat[0]
        >> lon[0]
        >> rwnum[1]
        >> lat[1]
        >> lon[1];

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read water runway: (" << lon[0] << "," << lat[0] << ") to (" << lon[1] << "," << lat[1] << ") width: " << width << " buoys = " << buoys );
}

tgContour WaterRunway::GetBuoys()
{
    tgContour buoys_nodes;

    if (buoys){
        double heading, az2, length;
        // calculate runway heading and length
        SGGeodesy::inverse(GetStart(), GetEnd(), heading, az2, length);

        // create a contour with points every 100m
        tgContour area = gen_wgs84_area(GetStart(), GetEnd(),
                                        0, 0, 0, width, heading, false);
        for ( unsigned int i = 0; i < area.GetSize(); ++i ) {
            double dist, course, cs;
            SGGeodesy::inverse(area.GetNode(i), area.GetNode(i==3 ? 0 : i+1), course, cs, dist );
            int divs = (int)(dist / 100.0);
            double step = dist/divs;
            SGGeod pt = area.GetNode(i);
            for (int j = 0; j < divs; ++j) {
                pt = SGGeodesy::direct(pt, course, step );
                buoys_nodes.AddNode( pt );
            }
        }
    }

    return buoys_nodes;
}

int Runway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, tgAccumulator& accum, std::string& shapefile_name )
{
    if ( rwy.surface == 1 /* Asphalt */ )
    {
        material_prefix = "pa_";
    }
    else if ( rwy.surface == 2 /* Concrete */ )
    {
        material_prefix = "pc_";
    }
    else if ( rwy.surface == 3 /* Turf/Grass */ )
    {
        material_prefix = "grass_rwy";
    }
    else if ( rwy.surface == 4 /* Dirt */ || rwy.surface == 5 /* Gravel */ )
    {
        material_prefix = "dirt_rwy";
    }
    else if ( rwy.surface == 12 /* Dry Lakebed */ )
    {
        material_prefix = "lakebed_taxiway";
    }
    else if ( rwy.surface == 13 /* Water runway (buoys) */ )
    {
        // water
    }
    else if ( rwy.surface == 14 /* Snow / Ice */ )
    {
        // Ice
    }
    else if ( rwy.surface == 15 /* Transparent */ )
    {
        //Transparent texture
    }
    else
    {
        TG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << rwy.surface);
        throw sg_exception("unknown runway type!");
    }

    // first, check the surface type - anything but concrete and asphalt are easy
    switch( rwy.surface )
    {
        case 1: // asphalt:
        case 2: // concrete
            TG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: asphalt or concrete " << rwy.surface);
            gen_rwy( rwy_polys, slivers, accum, shapefile_name );
            gen_runway_lights( rwy_lights );
            break;

        case 3: // Grass
        case 4: // Dirt
        case 5: // Gravel
        case 12: // dry lakebed
            TG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: Grass, Dirt, Gravel or Dry Lakebed " << rwy.surface );
            gen_simple_rwy( rwy_polys, slivers, accum );
            gen_runway_lights( rwy_lights );
            break;

        case 13: // water
            TG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: Water");
            break;

        case 14: // snow
            TG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: Snow");
            break;

        case 15: // transparent
            TG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: transparent");
            break;

        default: // unknown
            TG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: unknown: " << rwy.surface);
            break;
    }

    return 0;
}

int Runway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, tgpolygon_list& apt_base_polys, tgpolygon_list& apt_clearing_polys, tgAccumulator& accum, std::string& shapefile_name )
{
    tgContour base_contour, safe_base_contour;
    tgPolygon base, safe_base;
    double shoulder_width = 0.0;

    BuildBtg( rwy_polys, rwy_lights, slivers, accum, shapefile_name );

    // generate area around runways
    if ( (rwy.shoulder > 0) && (rwy.surface < 3) ) {
        shoulder_width += 22.0;
    } else if ( (rwy.surface == 1) || (rwy.surface == 2) ) {
        shoulder_width += 2.0;
    }

    base_contour      = gen_runway_area_w_extend( 20.0, -rwy.overrun[0], -rwy.overrun[1], shoulder_width + 20.0 );
    base_contour      = tgContour::Snap( base_contour, gSnap );
    base.AddContour( base_contour );

    // also clear a safe area around the runway
    safe_base_contour = gen_runway_area_w_extend( 180.0, -rwy.overrun[0], -rwy.overrun[1], shoulder_width + 50.0 );
    safe_base_contour = tgContour::Snap( safe_base_contour, gSnap );
    safe_base.AddContour( safe_base_contour );

    // add this to the airport clearing
    apt_clearing_polys.push_back( safe_base );

    // and add the clearing to the base
    apt_base_polys.push_back( base );

    return 0;
}
