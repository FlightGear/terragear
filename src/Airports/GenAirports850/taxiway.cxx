
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/SGMath.hxx>

#include <terragear/tg_shapefile.hxx>

#include <cstdio>

#include "global.hxx"
#include "apt_math.hxx"
#include "beznode.hxx"
#include "taxiway.hxx"

extern int nudge;

Taxiway::Taxiway(char* definition)
{
    double  lon = 0, lat = 0;
    SGGeod  origin;
    double  heading;
    double  length;
    double  width;
    int     surface;
    char    lighting[6];
 
    // unused params in the taxiway def
    char    designation[16];
    double  threshold;
    double  overrun;
    int     shoulder;
    int     markings;
    double  smoothness;
    int     signs;
    
    cgalPoly_Polygon taxi_contour;
    std::string      material;
    
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

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read taxiway: (" << lon << "," << lat << ") heading: " << heading << " length: " << length << " width: " << width );

    // adjust length and width from feet to meters
    length *= SG_FEET_TO_METER;
    width  *= SG_FEET_TO_METER;

    // adjust lat / lon to the start of the taxiway, not the middle
    origin = SGGeodesy::direct( SGGeod::fromDeg(lon, lat), heading, -length/2 );
    taxi_contour = gen_wgs84_rect( origin, heading, length, width );
    
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
        TG_LOG(SG_GENERAL, SG_WARN, "surface_code = " << surface);
        throw sg_exception("unknown runway type!");
    }
    
    tgPolygonSetMeta meta( tgPolygonSetMeta::META_TEXTURED, material, "taxiway" );
    meta.setTextureRef( taxi_contour[0], width, 25*SG_FEET_TO_METER, heading );
    meta.setTextureLimits( 0.0, 0.0, 1.0, 1.0 );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_TPS_CLIPU, -1.0, -1.0, 1.0, 1.0 );
    taxiwayPoly = tgPolygonSet( taxi_contour, meta );

    meta.setMaterial( "Grass" );
    meta.setTextureMethod( tgPolygonSetMeta::TEX_BY_GEODE );
    meta.setDescription( "taxiway_innerbase" );
    tgPolygonSet ib( taxi_contour, meta  );    
    innerBasePoly = ib.offset( 20.0l );
    
    meta.setDescription( "taxiway_outerbase" );
    tgPolygonSet ob( taxi_contour, meta  );    
    outerBasePoly = ob.offset( 20.0l );

    if ( (surface == 1) || (surface == 2) ) {
        // Create blue taxiway edge lights along the long sides of the taxiway
        // Spacing is 10m

        // Vector calculation
        SGVec3f vec = normalize( SGVec3f( CGAL::to_double(taxi_contour[0].x()), CGAL::to_double(taxi_contour[0].y()), 0.0l ));

        tgLightContour blue;
        blue.SetType( "RWY_BLUE_TAXIWAY_LIGHTS" );

        for ( unsigned int i = 0; i < taxi_contour.size(); ++i ) {
            double dist, course, cs;
            unsigned int   srcIdx = i;
            unsigned int   trgIdx = i+1;
            SGGeod sgSrc = SGGeod::fromDeg( CGAL::to_double( taxi_contour[srcIdx].x() ), CGAL::to_double( taxi_contour[srcIdx].y() ) );
            SGGeod sgTrg = SGGeod::fromDeg( CGAL::to_double( taxi_contour[trgIdx].x() ), CGAL::to_double( taxi_contour[trgIdx].y() ) );
            
            SGGeodesy::inverse(sgSrc, sgTrg, course, cs, dist );
            
            int divs = (int)(dist / 10.0);
            double step = dist/divs;
            SGGeod pt = sgSrc;
            for (int j = 0; j < divs; ++j) {
                pt = SGGeodesy::direct(pt, course, step );
                blue.AddLight( pt, vec );
            }
            i++;
        }
        lights.push_back( blue );
    }    
}