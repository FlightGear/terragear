
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

    taxi_contour = gen_wgs84_rect( origin, heading, length, width );
} 

int Taxiway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, std::string& shapefile_name )
{
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

    if(  shapefile_name.size() ) {
        tgPolygon taxi_poly;
        taxi_poly.AddContour( taxi_contour );

        tgPolygon::ToShapefile( taxi_poly, "./airport_dbg", std::string("preclip"), shapefile_name );
        tgPolygon::AccumulatorToShapefiles( "./airport_dbg", "accum" );
    }

    tgPolygon clipped = tgContour::DiffWithAccumulator( taxi_contour );
    tgPolygon split   = tgPolygon::SplitLongEdges( clipped, 100 );
    
    tgPolygon::RemoveSlivers( split, slivers );

    split.SetMaterial( material );
    split.SetTexParams( taxi_contour.GetNode(0), width, 25*SG_FEET_TO_METER, heading );
    split.SetTexLimits( 0.0, 0.0, 1.0, 1.0 );
    split.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, -1.0, 1.0, 1.0 );
    rwy_polys.push_back( split );

    if(  shapefile_name.size() ) {
        tgPolygon::ToShapefile( split, "./airport_dbg", std::string("postclip"), shapefile_name );
    }

    tgContour::AddToAccumulator( taxi_contour );

    return 0;
}

int Taxiway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, tgPolygon& apt_base, tgPolygon& apt_clearing, std::string& shapefile_name )
{
    tgContour base, safe_base;

    BuildBtg( rwy_polys, rwy_lights, slivers, shapefile_name );

    base = tgContour::Expand( taxi_contour, 20.0);

    safe_base = tgContour::Expand( taxi_contour, 50.0);

    // add this to the airport clearing
    apt_clearing = tgPolygon::Union( safe_base, apt_clearing);

    // and add the clearing to the base
    apt_base = tgPolygon::Union( base, apt_base );

    return 0;
}
