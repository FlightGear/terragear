
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/SGGeodesy.hxx>

#include <terragear/tg_shapefile.hxx>

#include <cstdio>

#include "global.hxx"
#include "apt_math.hxx"
#include "beznode.hxx"
#include "runway.hxx"

extern int nudge;

Runway::Runway(Airport* ap, char* definition)
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
        &width,  &surface, &shoulder, &smoothness, &centerline_lights, &edge_lights, &dist_remain_signs,
        rwnum[0], &lat[0], &lon[0], &threshold[0], &overrun[0], &marking[0], &approach_lights[0], &tz_lights[0], &reil[0],
        rwnum[1], &lat[1], &lon[1], &threshold[1], &overrun[1], &marking[1], &approach_lights[1], &tz_lights[1], &reil[1]
    );

    // calculate runway heading and length (used a lot)
    SGGeodesy::inverse( GetStart(), GetEnd(), heading, az2, length );

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read runway: (" << lon[0] << "," << lat[0] << ") to (" << lon[1] << "," << lat[1] << ") heading: " << heading << " length: " << length << " width: " << width );

    airport = ap;
} 


WaterRunway::WaterRunway(char* definition)
{
    sscanf(definition, "%lf %d %s %lf %lf %s %lf %lf", &width, &buoys, rwnum[0], &lat[0], &lon[0], rwnum[1], &lat[1], &lon[1]);

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
        cgalPoly_Polygon area = gen_wgs84_area(GetStart(), GetEnd(), 0, 0, 0, width, heading, false);
        for ( unsigned int i = 0; i < area.size(); ++i ) {
            double dist, course, cs;
            unsigned int   srcIdx = i;
            unsigned int   trgIdx = (i==3) ? 0 : i+1;
            SGGeod sgSrc = SGGeod::fromDeg( CGAL::to_double( area[srcIdx].x() ), CGAL::to_double( area[srcIdx].y() ) );
            SGGeod sgTrg = SGGeod::fromDeg( CGAL::to_double( area[trgIdx].x() ), CGAL::to_double( area[trgIdx].y() ) );
        
            SGGeodesy::inverse(sgSrc, sgTrg, course, cs, dist );
            
            int divs = (int)(dist / 100.0);
            double step = dist/divs;
            for (int j = 0; j < divs; ++j) {
                SGGeod pt = SGGeodesy::direct(sgSrc, course, step );
                buoys_nodes.AddNode( pt );
            }
        }
    }
    return buoys_nodes;
}

tgPolygonSetList& Runway::GetMainPolys( void )
{
    tgPolygonSetList polys;
    
    switch( surface ) {
        case 1:
            material_prefix = "pa_";
            gen_full_rwy();
            break;

        case 2:
            material_prefix = "pc_";
            gen_full_rwy();
            break;

        case 3:
            material_prefix = "grass_rwy";
            gen_simple_rwy();
            break;

        case 4:
        case 5:
            material_prefix = "dirt_rwy";
            gen_simple_rwy();
            break;

        case 12: /* Dry Lakebed */
            material_prefix = "lakebed_taxiway";
            gen_simple_rwy();
            break;

        case 13: /* Water runway (buoys) */
            break;

        case 14: /* Snow / Ice */
            break;
    
        case 15: /* Transparent */
            break;

        default:
            TG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << surface);
            throw sg_exception("unknown runway type!");
    }
    
    return runway_polys;
}

tgPolygonSetList& Runway::GetShoulderPolys( void )
{
    return shoulder_polys;
}

tgPolygonSetList& Runway::GetMarkingPolys( void )
{    
    for ( unsigned int i = 0; i < features.size(); i++) {
        tgPolygonSetList featMarks = features[i]->GetPolys();
        marking_polys.insert( marking_polys.end(), featMarks.begin(), featMarks.end() );
    }
    
    return marking_polys;
}

tgPolygonSetList& Runway::GetCapPolys( void )
{
    for ( unsigned int i = 0; i < features.size(); i++) {
        tgPolygonSetList featCaps = features[i]->GetCapPolys();
        cap_polys.insert( cap_polys.end(), featCaps.begin(), featCaps.end() );
    }
    
    return cap_polys;
}

tgPolygonSetList& Runway::GetInnerBasePolys( void )
{
    cgalPoly_Polygon base_contour;
    double shoulder_width = 0.0;
    char   description[64];

    // generate area around runways
    if ( (shoulder > 0) && (surface < 3) ) {
        shoulder_width = 22.0;
    } else if ( (surface == 1) || (surface == 2) ) {
        shoulder_width = 2.0;
    }

    base_contour      = gen_runway_area_w_extend( 20.0, -overrun[0], -overrun[1], shoulder_width + 20.0 );

    sprintf( description, "%s-%s_inner_base", rwnum[0], rwnum[1] );
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, "Grass", description );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );
    
    //tgShapefile::FromPolygon( base, true, false, "./dbg", "innerbase", "runway" );
    
    // and add the clearing to the base
    innerbase_polys.push_back( tgPolygonSet( base_contour, meta ) );
    
    return innerbase_polys;
}

tgPolygonSetList& Runway::GetOuterBasePolys( void )
{
    cgalPoly_Polygon base_contour;
    double shoulder_width = 0.0;
    char   description[64];
    
    // generate area around runways
    if ( (shoulder > 0) && (surface < 3) ) {
        shoulder_width = 22.0;
    } else if ( (surface == 1) || (surface == 2) ) {
        shoulder_width = 2.0;
    }

    // also clear a safe area around the runway
    base_contour = gen_runway_area_w_extend( 180.0, -overrun[0], -overrun[1], shoulder_width + 50.0 );

    sprintf( description, "%s-%s_outer_base", rwnum[0], rwnum[1] );
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, "Grass", description );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );

    // add this to the airport clearing
    outerbase_polys.push_back( tgPolygonSet( base_contour, meta ) );
    
    return outerbase_polys;
}

void Runway::GetLights( tglightcontour_list& lights )
{
    // first, check the surface type - anything but concrete and asphalt are easy
    switch( surface )
    {
        case 1: // asphalt:
        case 2: // concrete
            TG_LOG( SG_GENERAL, SG_DEBUG, "Get Lights: asphalt or concrete " << surface);
            gen_runway_lights( lights );
            break;
            
        case 3: // Grass
        case 4: // Dirt
        case 5: // Gravel
        case 12: // dry lakebed
            TG_LOG( SG_GENERAL, SG_DEBUG, "Get Lights: Grass, Dirt, Gravel or Dry Lakebed " << surface );
            gen_runway_lights( lights );
            break;
            
        case 13: // water
        case 14: // snow
        case 15: // transparent
        default: // unknown
            TG_LOG( SG_GENERAL, SG_DEBUG, "Get Lights: no lights: " << surface);
            break;
    }
}