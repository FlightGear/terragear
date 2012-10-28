
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/SGMath.hxx>

#include "global.hxx"
#include "apt_math.hxx"
#include "beznode.hxx"
#include "taxiway.hxx"

extern int nudge;

Taxiway::Taxiway(char* definition)
{
    // variables for sdjusting 810 rwy format to 850 rwy format 
    double lon = 0, lat = 0;

    // variables to store unused parameters
    char   designation[16];
    double threshold;
    double overrun;
    int    shoulder;
    int    markings;
    double smoothness;
    int    signs;

    // format:
    // taxiway  lat         lon           designation  heading  length      threshold   overrun
    // 10       44.38085600 -074.20606200 xxx          79.29    3384        0.0         0.0
    //     
    // width    lighting    surface       shoulder     markings smoothness  dist remain
    // 60       161161      1             0            0        0.35        0

    // Parse the line
    // 44.38085600 -074.20606200 xxx  79.29  3384 0.0 0.0    60 161161  1 0 0 0.35 0

    // int fscanf(FILE *stream, const char *format, ...);
    sscanf(definition, "%lf %lf %s %lf %lf %lf %lf %lf %s %d %d %d %lf %d", 
        &lat, &lon, designation, &heading, &length, &threshold, &overrun, 
        &width, lighting, &surface, &shoulder, &markings, &smoothness, &signs);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read taxiway: (" << lon << "," << lat << ") heading: " << heading << " length: " << length << " width: " << width );

    // adjust length and width from feet to meters
    length *= SG_FEET_TO_METER;
    width *= SG_FEET_TO_METER;

    // adjust lat / lon to the start of the taxiway, not the middle
    origin = SGGeodesy::direct( SGGeod::fromDeg(lon, lat), heading, -length/2 );
} 

int Taxiway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, tgPolygon& apt_base, tgPolygon& apt_clearing, bool make_shapefiles )
{
    tgContour taxi;
    tgContour base, safe_base;
    BuildBtg( rwy_polys, rwy_lights, slivers, make_shapefiles );

    // generate a poly for this segment
    taxi = gen_wgs84_rect( origin, heading, length, width );

    base = tgContour::Expand( taxi, 20.0);

    safe_base = tgContour::Expand( taxi, 50.0);

    // add this to the airport clearing
    apt_clearing = tgPolygon::Union( safe_base, apt_clearing);

    // and add the clearing to the base
    apt_base = tgPolygon::Union( base, apt_base );

    return 0;
}

int Taxiway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, bool make_shapefiles )
{
    tgContour taxi;
    std::string material;

    if ( surface == 1 /* Asphalt */ )
    {
        if ( (width <= 50) && (lighting[1] == '6') ) {
            material = "pa_taxiway";
        } else {
            material = "pa_tiedown";
        }
    }
    else if ( surface == 2 /* Concrete */ )
    {
        if ( (width <= 50) && (lighting[1] == '6') ) {
            material = "pc_taxiway";
        } else {
            material = "pc_tiedown";
        }
    }
    else if ( surface == 3 /* Turf/Grass */ )
    {
        material = "grass_rwy";
    }
    else if ( surface == 4 /* Dirt */ || surface == 5 /* Gravel */ )
    {
        material = "dirt_rwy";
    }
    else if ( surface == 12 /* Dry Lakebed */ )
    {
        material = "lakebed_taxiway";
    }
    else if ( surface == 13 /* Water runway (buoy's?) */ )
    {
        // water
    }
    else if ( surface == 14 /* Snow / Ice */ )
    {
        // Ice
    }
    else if ( surface == 15 /* Transparent */ )
    {
        //Transparent texture
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << surface);
        throw sg_exception("unknown runway type!");
    }

    // generate a poly for this segment
    taxi = gen_wgs84_rect( origin, heading, length, width );

    tgPolygon clipped = tgContour::DiffWithAccumulator( taxi );
    tgPolygon::RemoveSlivers( clipped, slivers );

    SG_LOG(SG_GENERAL, SG_DEBUG, "tw2 clipped = " << clipped.Contours());

    clipped.SetMaterial( material );
    clipped.SetTexParams( taxi.GetNode(0), width, 250*SG_FEET_TO_METER, heading );
    clipped.SetTexMethod( TG_TEX_BY_TPS_CLIPUV, 0.0, 0.0, 1.0, 1.0 );
    rwy_polys.push_back( clipped );

    tgContour::AddToAccumulator( taxi );

    return 0;
}
