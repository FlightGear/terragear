
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/SGMath.hxx>

#include <terragear/tg_shapefile.hxx>

#include "global.hxx"
#include "apt_math.hxx"
#include "beznode.hxx"
#include "taxiway.hxx"

extern int nudge;

Taxiway::Taxiway(char* definition)
{
    // variables for adjusting 810 rwy format to 850 rwy format
    double lon = 0, lat = 0;

    // variables to store unused parameters
    char   designation[16];
    double threshold;
    double overrun;
    int    shoulder;
    int    markings;
    double smoothness;
    int    signs;

    // taxiway format:
    //      lat         lon           designation  heading  length      threshold   overrun
    //      44.38085600 -074.20606200 xxx          79.29    3384        0.0         0.0
    //
    //      width    lighting    surface       shoulder     markings smoothness  dist remain
    //      60       161161      1             0            0        0.35        0

    // Parse the line
    //      44.38085600 -074.20606200 xxx  79.29  3384 0.0 0.0    60 161161  1 0 0 0.35 0

    std::istringstream ss(definition);
    ss  >> lat
        >> lon
        >> designation
        >> heading
        >> length
        >> threshold
        >> overrun
        >> width
        >> lighting
        >> surface
        >> shoulder
        >> markings
        >> smoothness
        >> signs;

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read taxiway: (" << lon << "," << lat << ") heading: " << heading << " length: " << length << " width: " << width );

    // adjust length and width from feet to meters
    length *= SG_FEET_TO_METER;
    width *= SG_FEET_TO_METER;

    // adjust lat / lon to the start of the taxiway, not the middle
    origin = SGGeodesy::direct( SGGeod::fromDeg(lon, lat), heading, -length/2 );

    taxi_contour = gen_wgs84_rect( origin, heading, length, width );
}

void Taxiway::GenLights(tglightcontour_list& rwy_lights)
{
    // Create blue taxiway edge lights along the long sides of the taxiway
    // Spacing is 10m

    // Vector calculation
    SGVec3f vec = normalize(SGVec3f::fromGeod(taxi_contour.GetNode(0)));

    tgLightContour blue;
    blue.SetType( "RWY_BLUE_TAXIWAY_LIGHTS" );

    for ( unsigned int i = 0; i < taxi_contour.GetSize(); ++i ) {
        double dist, course, cs;
        SGGeodesy::inverse(taxi_contour.GetNode(i), taxi_contour.GetNode(i+1), course, cs, dist );
        int divs = (int)(dist / 10.0);
        double step = dist/divs;
        SGGeod pt = taxi_contour.GetNode(i);
        for (int j = 0; j < divs; ++j) {
            pt = SGGeodesy::direct(pt, course, step );
            blue.AddLight( pt, vec );
        }
        i++;
    }
    rwy_lights.push_back( blue );

}

int Taxiway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, tgAccumulator& accum, std::string& shapefile_name )
{
    std::string material;

    if ( surface == 1 /* Asphalt */ )
    {
        if ( (width <= 50) && (lighting[1] == '6') ) {
            material = "pa_taxiway";
            GenLights(rwy_lights);
        } else {
            material = "pa_tiedown";
        }
    }
    else if ( surface == 2 /* Concrete */ )
    {
        if ( (width <= 50) && (lighting[1] == '6') ) {
            material = "pc_taxiway";
            GenLights(rwy_lights);
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
        TG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << surface);
        throw sg_exception("unknown runway type!");
    }

    if(  shapefile_name.size() ) {
        tgPolygon taxi_poly;
        taxi_poly.AddContour( taxi_contour );

        tgShapefile::FromPolygon( taxi_poly, "./airport_dbg", std::string("preclip"), shapefile_name );
        accum.ToShapefiles( "./airport_dbg", "accum", true );
    }

    tgPolygon clipped = accum.Diff( taxi_contour );
    tgPolygon split   = tgPolygon::SplitLongEdges( clipped, 100 );

    tgPolygon::RemoveSlivers( split, slivers );

    split.SetMaterial( material );
    split.SetTexParams( taxi_contour.GetNode(0), width, 25*SG_FEET_TO_METER, heading );
    split.SetTexLimits( 0.0, 0.0, 1.0, 1.0 );
    split.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1.0, -1.0, 1.0, 1.0 );
    rwy_polys.push_back( split );

    if(  shapefile_name.size() ) {
        tgShapefile::FromPolygon( split, "./airport_dbg", std::string("postclip"), shapefile_name );
    }

    accum.Add( taxi_contour );

    return 0;
}

int Taxiway::BuildBtg( tgpolygon_list& rwy_polys, tglightcontour_list& rwy_lights, tgcontour_list& slivers, tgpolygon_list& apt_base_polys, tgpolygon_list& apt_clearing_polys, tgAccumulator& accum, std::string& shapefile_name )
{
    tgContour base_contour, safe_base_contour;
    tgPolygon base, safe_base;

    BuildBtg( rwy_polys, rwy_lights, slivers, accum, shapefile_name );

    base_contour = tgContour::Expand( taxi_contour, 20.0);
    base.AddContour( base_contour );

    safe_base_contour = tgContour::Expand( taxi_contour, 50.0);
    safe_base.AddContour( safe_base_contour );
    
    // add this to the airport clearing
    apt_clearing_polys.push_back( safe_base );

    // and add the clearing to the base
    apt_base_polys.push_back( base );

    return 0;
}
