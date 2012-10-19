
#include <simgear/compiler.h>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include <Geometry/poly_support.hxx>
#include <Polygon/polygon.hxx>

#include "global.hxx"
#include "apt_math.hxx"
#include "beznode.hxx"
#include "taxiway.hxx"

extern int nudge;

Taxiway::Taxiway(char* definition)
{
    // variables for sdjusting 810 rwy format to 850 rwy format 
    double pt_x = 0, pt_y = 0;
    double az2;

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
    geo_direct_wgs_84( lat, lon, heading, -length/2, &pt_y, &pt_x, &az2 );

    lat = pt_y;
    lon = pt_x;
} 

int Taxiway::BuildBtg( superpoly_list* rwy_polys, texparams_list* texparams, superpoly_list* rwy_lights, ClipPolyType* accum, poly_list& slivers, TGPolygon* apt_base, TGPolygon* apt_clearing, bool make_shapefiles )
{
    TGPolygon taxi;
    TGPolygon base, safe_base;
    TGPolygon pre_accum;
    std::string material;
    void*     ds_id = NULL;        // If we are going to build shapefiles
    void*     l_id  = NULL;        // datasource and layer IDs

    if ( make_shapefiles ) {
        char ds_name[128];
        sprintf(ds_name, "./taxi_debug");
        ds_id = tgShapefileOpenDatasource( ds_name );
    }

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
    taxi = gen_wgs84_rect( lat, lon, heading, length, width );

    TGSuperPoly sp;
    TGTexParams tp;

    if ( make_shapefiles ) {
        char layer_name[128];
        char feature_name[128];

        sprintf( layer_name, "original" );
        l_id = tgShapefileOpenLayer( ds_id, layer_name );
        sprintf( feature_name, "original" );
        tgShapefileCreateFeature( ds_id, l_id, taxi, feature_name );

        pre_accum = *accum;
    }
    
    TGPolygon clipped = tgPolygonDiffClipper( taxi, *accum );
    tgPolygonFindSlivers( clipped, slivers );

    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped = " << clipped.contours());

    sp.erase();
    sp.set_poly( clipped );
    sp.set_material( material );
    sp.set_flag("taxi");

    rwy_polys->push_back( sp );

    *accum = tgPolygonUnionClipper( taxi, *accum );

    /* If debugging this poly, write the poly, and clipped poly and the accum buffer into their own layers */
    if ( make_shapefiles ) {
        char layer_name[128];
        char feature_name[128];

        sprintf( layer_name, "clipped" );
        l_id = tgShapefileOpenLayer( ds_id, layer_name );
        sprintf( feature_name, "clipped" );
        tgShapefileCreateFeature( ds_id, l_id, clipped, feature_name );

        sprintf( layer_name, "pre_accum" );
        l_id = tgShapefileOpenLayer( ds_id, layer_name );
        sprintf( feature_name, "pre_accum" );
        tgShapefileCreateFeature( ds_id, l_id, pre_accum, feature_name );

        sprintf( layer_name, "post_accum" );
        l_id = tgShapefileOpenLayer( ds_id, layer_name );
        sprintf( feature_name, "post_accum" );
        tgShapefileCreateFeature( ds_id, l_id, *accum, feature_name );
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "tp construct");

    tp = TGTexParams( taxi.get_pt(0,0).toSGGeod(), width, 250*SG_FEET_TO_METER, heading );
    texparams->push_back( tp );

    if ( apt_base )
    {           
        base = tgPolygonExpand( taxi, 20.0); 
        if ( make_shapefiles ) {
            char layer_name[128];
            char feature_name[128];

            sprintf( layer_name, "exp_base" );
            l_id = tgShapefileOpenLayer( ds_id, layer_name );
            sprintf( feature_name, "exp_base" );
            tgShapefileCreateFeature( ds_id, l_id, base, feature_name );
        }

        safe_base = tgPolygonExpand( taxi, 50.0);        
        if ( make_shapefiles ) {
            char layer_name[128];
            char feature_name[128];

            SG_LOG(SG_GENERAL, SG_INFO, "expanded safe poly: " << safe_base);

            sprintf( layer_name, "exp_safe_base" );
            l_id = tgShapefileOpenLayer( ds_id, layer_name );
            sprintf( feature_name, "exp_safe_base" );
            tgShapefileCreateFeature( ds_id, l_id, safe_base, feature_name );
        }                
    
        // add this to the airport clearing
        *apt_clearing = tgPolygonUnionClipper( safe_base, *apt_clearing);

        // and add the clearing to the base
        *apt_base = tgPolygonUnionClipper( base, *apt_base );
    }

    if ( make_shapefiles )
    {
        tgShapefileCloseDatasource( ds_id );
    }

    return 0;
}
