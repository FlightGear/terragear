
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/SGGeodesy.hxx>

#include <Geometry/poly_support.hxx>
#include <Polygon/polygon.hxx>

#include "global.hxx"
#include "apt_math.hxx"
#include "beznode.hxx"
#include "runway.hxx"

extern int nudge;

Runway::Runway(char* definition)
{
    double az2;

    // format:
    // runway   width   surface  shoulder  smoothness centerline lights  edge lighting distance remaining signs
    // 100      46.02   2        1         0.00       1                  2             1 
    //     
    //          runway number  runway end lat runway end long   threshold  overrun  markings  approach lighting
    //          09L            33.63470475     -084.44798671    0.00       120.09   3         7 
    // 
    //          touchdown zone lighting  runway end identifier lights
    //          0                        1                              
    //
    //          runway number  runway end lat runway end long   threshold  overrun  markings  approach lighting
    //          27R             33.63469907   -084.40893004     0.00       120.09   3         6 
    //
    //          touchdown zone lighting  runway end identifier lights
    //          0                        1

    // Parse the line
    // 46.02   2   1 0.00 1 2 1 09L  33.63470475 -084.44798671    0.00  120.09 3  7 0 1 27R  33.63469907 -084.40893004    0.00  120.09 3  6 0 1

    // int fscanf(FILE *stream, const char *format, ...);
    sscanf(definition, "%lf %d %d %lf %d %d %d %s %lf %lf %lf %lf %d %d %d %d %s %lf %lf %lf %lf %d %d %d %d", 
        &rwy.width,  &rwy.surface, &rwy.shoulder, &rwy.smoothness, &rwy.centerline_lights, &rwy.edge_lights, &rwy.dist_remain_signs,
        rwy.rwnum[0], &rwy.lat[0], &rwy.lon[0], &rwy.threshold[0], &rwy.overrun[0], &rwy.marking[0], &rwy.approach_lights[0], &rwy.tz_lights[0], &rwy.reil[0],
        rwy.rwnum[1], &rwy.lat[1], &rwy.lon[1], &rwy.threshold[1], &rwy.overrun[1], &rwy.marking[1], &rwy.approach_lights[1], &rwy.tz_lights[1], &rwy.reil[1]
    );

    // calculate runway heading and length (used a lot)
    SGGeodesy::inverse( GetStart(), GetEnd(), rwy.heading, az2, rwy.length );

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read runway: (" << rwy.lon[0] << "," << rwy.lat[0] << ") to (" << rwy.lon[1] << "," << rwy.lat[1] << ") heading: " << rwy.heading << " length: " << rwy.length << " width: " << rwy.width );

} 


WaterRunway::WaterRunway(char* definition)
{
    sscanf(definition, "%lf %d %s %lf %lf %s %lf %lf", &width, &buoys, rwnum[0], &lat[0], &lon[0], rwnum[1], &lat[1], &lon[1]);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read water runway: (" << lon[0] << "," << lat[0] << ") to (" << lon[1] << "," << lat[1] << ") width: " << width << " buoys = " << buoys );
}

point_list WaterRunway::GetNodes()
{
    point_list buoys_nodes;
    buoys_nodes.clear();

    if (buoys){
        double heading, az2, length;
        // calculate runway heading and length
        SGGeodesy::inverse(GetStart(), GetEnd(), heading, az2, length);

        // create a polygon for the outline and use it to calculate the point list
        int divs = (int)(length / 100.0);
        TGPolygon area = gen_wgs84_area(GetStart(), GetEnd(),
                                        0, 0, 0, width, heading, false);
        Point3D pt, inc;

        for ( int i = 0; i < area.contour_size( 0 ); ++i ) {
            pt = area.get_pt( 0, i );
            inc = (area.get_pt(0, i==3 ? 0 : i+1) - area.get_pt(0,i)) / divs;
            for ( int j = 0; j < divs; ++j) {
                buoys_nodes.push_back( pt);
                pt += inc;
            }
        }
    }
    return buoys_nodes;
}


int Runway::BuildBtg( superpoly_list* rwy_polys, texparams_list* texparams, superpoly_list* rwy_lights, poly_list& slivers, TGPolygon* apt_base, TGPolygon* apt_clearing, bool make_shapefiles )
{
    TGPolygon base, safe_base;

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
        SG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << rwy.surface);
    	throw sg_exception("unknown runway type!");
    }

    // first, check the surface type - anything but concrete and asphalt are easy
    switch( rwy.surface )
    {
        case 1: // asphalt:
        case 2: // concrete
            SG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: asphalt or concrete " << rwy.surface);
            gen_rwy( rwy_polys, texparams, slivers, make_shapefiles );
            gen_runway_lights( rwy_lights );
            break;
    
        case 3: // Grass
        case 4: // Dirt
        case 5: // Gravel
        case 12: // dry lakebed
            SG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: Grass, Dirt, Gravel or Dry Lakebed " << rwy.surface );
            gen_simple_rwy( rwy_polys, texparams, slivers );
            gen_runway_lights( rwy_lights );
            break;

        case 13: // water
            SG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: Water");
            break;

        case 14: // snow
            SG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: Snow");
            break;

        case 15: // transparent
            SG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: transparent");
            break;

        default: // unknown
            SG_LOG( SG_GENERAL, SG_DEBUG, "Build Runway: unknown: " << rwy.surface);
            break;
    }    

    if (apt_base)
    {
        // generate area around runways
        base      = gen_runway_area_w_extend( 20.0, -rwy.overrun[0], -rwy.overrun[1], 20.0 );
        base      = snap( base, gSnap );

        // also clear a safe area around the runway
        safe_base = gen_runway_area_w_extend( 180.0, -rwy.overrun[0], -rwy.overrun[1], 50.0 );
        safe_base = snap( safe_base, gSnap );

        // add this to the airport clearing
        *apt_clearing = tgPolygonUnionClipper( safe_base, *apt_clearing );

        // and add the clearing to the base
        *apt_base = tgPolygonUnionClipper( base, *apt_base );
    }

    return 0;
}
