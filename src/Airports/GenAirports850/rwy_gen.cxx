// rwy_gen.cxx -- Build a runway
//
// Written by Curtis Olson, started February 2002.
//
// Copyright (C) 2002  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include <Geometry/poly_support.hxx>

#include "global.hxx"
#include "beznode.hxx"
#include "runway.hxx"

#include <stdlib.h>

using std::string;
struct sections
{
    const char* tex;
    int size;
};

// UK Precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections uk_prec[] = {
    { "tz_one_a", 380 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "aim_uk", 400 * SG_FEET_TO_METER },
    { "tz_one_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_b", 200 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_b", 200 * SG_FEET_TO_METER }
};

// UK non-precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections uk_nprec[] = {
    { "centerline", 200 * SG_FEET_TO_METER },
    { "aim_uk", 400 * SG_FEET_TO_METER },
};

// Precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections prec[] = {
    { "tz_three", 380 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "aim", 400 * SG_FEET_TO_METER },
    { "tz_two_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_two_b", 200 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_a", 400 * SG_FEET_TO_METER },
    { "rest", 200 * SG_FEET_TO_METER },
    { "tz_one_b", 200 * SG_FEET_TO_METER }
};

// Non-precision runway sections from after the designation number
// onwards to the middle (one half).
// Set order of sections and their corresponding size
static const struct sections nprec[] = {
    { "centerline", 200 * SG_FEET_TO_METER },
    { "aim", 400 * SG_FEET_TO_METER }
};


void Runway::gen_shoulder_section( Point3D p0, Point3D p1, Point3D t0, Point3D t1, int side, double heading, double width, std::string surface, TGSuperPoly& sp, TGTexParams& tp )
{
    Point3D     s0, s1, s2, s3;
    TGPolygon   poly;

    double wid_hdg = 0.0f;
    double az2     = 0.0f;
    double dist    = 0.0f;
    double pt_x    = 0.0f;
    double pt_y    = 0.0f;

    // calc heading from p1 to p0
    geo_inverse_wgs_84( p0.y(), p0.x(), p1.y(), p1.x(), &wid_hdg, &az2, &dist);
    
    // s0 is width away from t1 in wid_hdg direction
    geo_direct_wgs_84( t1.y(), t1.x(), wid_hdg, width, &pt_y, &pt_x, &az2 );    
    s0 = Point3D( pt_x, pt_y, 0.0f );

    // s1 is width away from t0 in wid_hdg direction
    geo_direct_wgs_84( t0.y(), t0.x(), wid_hdg, width, &pt_y, &pt_x, &az2 );    
    s1 = Point3D( pt_x, pt_y, 0.0f );

    // s2 is nudge away from t0 in -wid_hdg direction
    geo_direct_wgs_84( t0.y(), t0.x(), wid_hdg, -0.01, &pt_y, &pt_x, &az2 );    
    s2 = Point3D( pt_x, pt_y, 0.0f );

    // s3 is nudge away from t1 in -wid_hdg direction
    geo_direct_wgs_84( t1.y(), t1.x(), wid_hdg, -0.01, &pt_y, &pt_x, &az2 );    
    s3 = Point3D( pt_x, pt_y, 0.0f );

    // Generate a poly
    poly.erase();
    poly.add_node( 0, s0 );
    poly.add_node( 0, s1 );
    poly.add_node( 0, s2 );
    poly.add_node( 0, s3 );
    poly = snap( poly, gSnap );

    sp.erase();
    sp.set_poly( poly );
    sp.set_material( surface );
    sp.set_flag( "shoulder" );

    if (side == 0) {
        tp = TGTexParams( poly.get_pt(0,2), width, dist, heading );
        tp.set_minu(0);
        tp.set_maxu(1);
    } else {
        tp = TGTexParams( poly.get_pt(0,1), width, dist, heading );
        tp.set_minu(1);
        tp.set_maxu(0);
    }

    tp.set_minv(0);
    tp.set_maxv(1);
}

// generate a section of texture
void Runway::gen_runway_section( const TGPolygon& runway,
                                 double startl_pct, double endl_pct,
                                 double startw_pct, double endw_pct,
                                 double minu, double maxu, double minv, double maxv,
                                 double heading,
                                 const string& material,
                                 superpoly_list *rwy_polys,
                                 texparams_list *texparams,
                                 superpoly_list *shoulder_polys,
                                 texparams_list *shoulder_tps,
                                 ClipPolyType *accum,
                                 poly_list& slivers,
                                 bool make_shapefiles ) 
{
    int j, k;
    double width = rwy.width;
    double length = rwy.length;
    double lshoulder_width = 0.0f;
    double rshoulder_width = 0.0f;
    std::string shoulder_surface = "";

    void*      ds_id = NULL;        // If we are going to build shapefiles
    void*      l_id  = NULL;        // datasource and layer IDs
    static int poly_id = 0;         // for naming

    Point3D a0 = runway.get_pt(0, 1);
    Point3D a1 = runway.get_pt(0, 2);
    Point3D a2 = runway.get_pt(0, 0);
    Point3D a3 = runway.get_pt(0, 3);

    if ( startl_pct > 0.0 ) {
        startl_pct -= nudge * SG_EPSILON;
    }
    if ( endl_pct < 1.0 ) {
        endl_pct += nudge * SG_EPSILON;
    }
    if ( endl_pct > 1.0 ) {
        endl_pct = 1.0;
    }

    if ( make_shapefiles ) {
        char ds_name[128];
        sprintf(ds_name, "./rwy_debug");
        ds_id = tgShapefileOpenDatasource( ds_name );
    }

    // calculate if we are going to be creating shoulder polys
    if ( (rwy.shoulder > 0) && (rwy.surface < 3) ){
        if (rwy.shoulder == 1){
            shoulder_surface = "pa_shoulder";
        } else if (rwy.shoulder == 2){
            shoulder_surface = "pc_shoulder";
        }

        if ( startw_pct == 0.0f ) {
            lshoulder_width = 11.0;
        }
        if ( endw_pct == 1.0f ) {
            rshoulder_width = 11.0;
        }
    } else {  
        // We add a fake shoulder if the runway has an asphalt or concrete surface
        if ( (rwy.surface == 1) || (rwy.surface == 2) ) {
            if (rwy.surface == 1) {
                shoulder_surface = "pa_shoulder_f";
            } else if (rwy.surface == 2){
                shoulder_surface = "pc_shoulder_f";
            }

            if ( startw_pct == 0.0f ) {
                lshoulder_width = 1.0;
            }
            if ( endw_pct == 1.0f ) {
                rshoulder_width = 1.0;
            }
        }
    }

    // partial "w" percentages could introduce "T" intersections which
    // we compensate for later, but could still cause problems now
    // with our polygon clipping code.  This attempts to compensate
    // for that by nudging the areas a bit bigger so we don't end up
    // with polygon slivers.
    if ( startw_pct > 0.0 || endw_pct < 1.0 ) {
        if ( startw_pct > 0.0 ) {
            startw_pct -= nudge * SG_EPSILON;
        }
        if ( endw_pct < 1.0 ) {
            endw_pct += nudge * SG_EPSILON;
        }
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "start len % = " << startl_pct
           << " end len % = " << endl_pct);

    double dlx, dly;

    dlx = a1.x() - a0.x();
    dly = a1.y() - a0.y();

    Point3D t0 = Point3D( a0.x() + dlx * startl_pct,
                          a0.y() + dly * startl_pct, 0);
    Point3D t1 = Point3D( a0.x() + dlx * endl_pct,
                          a0.y() + dly * endl_pct, 0);

    dlx = a3.x() - a2.x();
    dly = a3.y() - a2.y();

    Point3D t2 = Point3D( a2.x() + dlx * startl_pct,
                          a2.y() + dly * startl_pct, 0);

    Point3D t3 = Point3D( a2.x() + dlx * endl_pct,
                          a2.y() + dly * endl_pct, 0);

    SG_LOG(SG_GENERAL, SG_DEBUG, "start wid % = " << startw_pct
           << " end wid % = " << endw_pct);

    double dwx, dwy;

    dwx = t0.x() - t2.x();
    dwy = t0.y() - t2.y();

    Point3D p0 = Point3D( t2.x() + dwx * startw_pct,
                          t2.y() + dwy * startw_pct, 0);

    Point3D p1 = Point3D( t2.x() + dwx * endw_pct,
                          t2.y() + dwy * endw_pct, 0);

    // check for left shoulder
    if ( lshoulder_width > 0.0f ) {
        TGSuperPoly sp;
        TGTexParams tp;

        gen_shoulder_section( p0, p1, t0, t1, 0, heading, lshoulder_width, shoulder_surface, sp, tp );
        shoulder_polys->push_back( sp );
        shoulder_tps->push_back( tp );

        /* If debugging this runway, write the shoulder poly */
        if (ds_id) {
            poly_id++;

            char layer_name[128];
            sprintf( layer_name, "lshoulder_%d", poly_id );
            l_id = tgShapefileOpenLayer( ds_id, layer_name );

            char feature_name[128];
            sprintf( feature_name, "lshoulder_%d", poly_id);
            tgShapefileCreateFeature( ds_id, l_id, sp.get_poly(), feature_name );
        }
    }

    dwx = t1.x() - t3.x();
    dwy = t1.y() - t3.y();

    Point3D p2 = Point3D( t3.x() + dwx * startw_pct,
                          t3.y() + dwy * startw_pct, 0);

    Point3D p3 = Point3D( t3.x() + dwx * endw_pct,
                          t3.y() + dwy * endw_pct, 0);

    // check for right shoulder
    if ( rshoulder_width > 0.0f ) {
        TGSuperPoly sp;
        TGTexParams tp;

        gen_shoulder_section( p1, p0, t2, t3, 1, heading, rshoulder_width, shoulder_surface, sp, tp );
        shoulder_polys->push_back( sp );
        shoulder_tps->push_back( tp );

        /* If debugging this runway, write the shoulder poly */
        if (ds_id) {
            poly_id++;

            char layer_name[128];
            sprintf( layer_name, "rshoulder_%d", poly_id );
            l_id = tgShapefileOpenLayer( ds_id, layer_name );

            char feature_name[128];
            sprintf( feature_name, "rshoulder_%d", poly_id);
            tgShapefileCreateFeature( ds_id, l_id, sp.get_poly(), feature_name );
        }
    }

    TGPolygon section;
    section.erase();

    section.add_node( 0, p2 );
    section.add_node( 0, p0 );
    section.add_node( 0, p1 );
    section.add_node( 0, p3 );
    section = snap( section, gSnap );

    /* If debugging this runway, write the shoulder poly */
    if (ds_id) {
        char layer_name[128];
        sprintf( layer_name, "poly_%d", poly_id );
        l_id = tgShapefileOpenLayer( ds_id, layer_name );

        char feature_name[128];
        sprintf( feature_name, "section_%d", poly_id);
        tgShapefileCreateFeature( ds_id, l_id, section, feature_name );
    }


    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "pre clipped runway pts " << material_prefix << material);
    for ( j = 0; j < section.contours(); ++j ) {
        for ( k = 0; k < section.contour_size( j ); ++k ) {
            Point3D p = section.get_pt(j, k);
            SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
        }
    }

    // Clip the new polygon against what ever has already been created.
    TGPolygon clipped = tgPolygonDiffClipper( section, *accum );
    tgPolygonFindSlivers( clipped, slivers );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    TGPolygon split = tgPolygonSplitLongEdges( clipped, 400.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    TGSuperPoly sp;
    sp.erase();
    sp.set_poly( split );
    sp.set_material( material_prefix + material );
    rwy_polys->push_back( sp );

    SG_LOG(SG_GENERAL, SG_DEBUG, "section = " << clipped.contours());

    *accum = tgPolygonUnionClipper( section, *accum );

    // Store away what we need to know for texture coordinate
    // calculation.  (CLO 10/20/02: why can't we calculate texture
    // coordinates here?  Oh, becuase later we need to massage the
    // polygons to avoid "T" intersections and clean up other
    // potential artifacts and we may add or remove points and need to
    // do new texture coordinate calcs later.

    double len = length / 2.0;
    double sect_len = len * ( endl_pct - startl_pct );
    double sect_wid = width * ( endw_pct - startw_pct );

    TGTexParams tp;
    tp = TGTexParams( p0,
                      sect_wid,
                      sect_len,
                      heading );
    tp.set_minu( minu );
    tp.set_maxu( maxu );
    tp.set_minv( minv );
    tp.set_maxv( maxv );
    texparams->push_back( tp );

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped runway pts " << material_prefix + material);
    for ( j = 0; j < clipped.contours(); ++j ) {
        for ( k = 0; k < clipped.contour_size( j ); ++k ) {
            Point3D p = clipped.get_pt(j, k);
            SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
        }
    }

    if (ds_id) {
        tgShapefileCloseDatasource( ds_id );        
    }
}

void Runway::gen_rw_designation( TGPolygon poly, double heading, string rwname,
                                 double &start_pct, double &end_pct,
                                 superpoly_list *rwy_polys,
                                 texparams_list *texparams,
                                 ClipPolyType *accum,
                                 poly_list& slivers,
                                 bool make_shapefiles )
{
    if (rwname != "XX") { /* Do not create a designation block if the runway name is set to none */
        string letter = "";
        double length = rwy.length / 2.0;
        for ( unsigned int i = 0; i < rwname.length(); ++i ) {
            string tmp = rwname.substr(i, 1);
            if ( tmp == "L" || tmp == "R" || tmp == "C" ) {
                rwname = rwname.substr(0, i);
                letter = tmp;
            }
        }
        SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation letter = " << letter);

        // create runway designation letter
        if ( !letter.empty() ) {
            start_pct = end_pct;
            end_pct = start_pct + ( 90.0 * SG_FEET_TO_METER / length );
            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                letter,
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );
        }


        // create runway designation number(s)
        if (rwname == "0") {
            rwname = "36";
        }

        SG_LOG(SG_GENERAL, SG_DEBUG, "Runway designation = " << rwname);

        char tex1[32]; tex1[0] = '\0';
        char tex2[32]; tex2[0] = '\0';

        start_pct = end_pct;
        end_pct = start_pct + ( 80.0 * SG_FEET_TO_METER / length );

        if (rwname.length() == 2) {
            sprintf( tex1, "%c%c", rwname[0], 'l');
            sprintf( tex2, "%c%c", rwname[1], 'r');

            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 0.5,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex1,
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );
            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.5, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex2,
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );

        } else if (rwname.length() == 1) {
            sprintf( tex1, "%c%c", rwname[0], 'c');

            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex1,
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );
        }
    }
}

// generate a runway.  The routine modifies
// rwy_polys, texparams, and accum.  For specific details and
// dimensions of precision runway markings, please refer to FAA
// document AC 150/5340-1H
void Runway::gen_rwy( superpoly_list *rwy_polys,
                      texparams_list *texparams,
                      ClipPolyType *accum,
                      poly_list& slivers,
                      bool make_shapefiles )
{
    SG_LOG( SG_GENERAL, SG_DEBUG, "Building runway = " << rwy.rwnum[0] << " / " << rwy.rwnum[1]);

    //
    // Generate the basic runway outlines
    //
    TGPolygon runway = gen_runway_w_mid( 0, 0 );

    TGPolygon runway_half;
    for ( int rwhalf=0; rwhalf<2; ++rwhalf ){

        if (rwhalf == 0) {

            //Create the first half of the runway (first entry in apt.dat)
            runway_half.erase();
            runway_half.add_node( 0, runway.get_pt(0, 3) );
            runway_half.add_node( 0, runway.get_pt(0, 4) );
            runway_half.add_node( 0, runway.get_pt(0, 5) );
            runway_half.add_node( 0, runway.get_pt(0, 2) );
        }
    
        else if (rwhalf == 1) {

            //Create the second runway half from apt.dat
            runway_half.erase();
            runway_half.add_node( 0, runway.get_pt(0, 0) );
            runway_half.add_node( 0, runway.get_pt(0, 1) );
            runway_half.add_node( 0, runway.get_pt(0, 2) );
            runway_half.add_node( 0, runway.get_pt(0, 5) );
        }

        Point3D p;
        SG_LOG(SG_GENERAL, SG_DEBUG, "raw runway half pts (run " << rwhalf << ")");
        for ( int i = 0; i < runway_half.contour_size( 0 ); ++i ) {
	        p = runway_half.get_pt( 0, i );
	        SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
        }

        double length = rwy.length / 2.0;

        // Make sure our runway is long enough for the desired marking variant
        if ( (rwy.marking[rwhalf]==2 || rwy.marking[rwhalf]==4) && length < 1150 * SG_FEET_TO_METER ) {
            SG_LOG( SG_GENERAL, SG_ALERT,
                "Runway " << rwy.rwnum[rwhalf] << " is not long enough ("
                << rwy.length << "m) for non-precision markings!  Setting runway markings to visual!");
            rwy.marking[rwhalf]=1;
        }
        if ( (rwy.marking[rwhalf]==3 || rwy.marking[rwhalf]==5) && length < 3075 * SG_FEET_TO_METER ) {
            SG_LOG( SG_GENERAL, SG_ALERT,
                "Runway " << rwy.rwnum[rwhalf] << " is not long enough ("
                << rwy.length << "m) for precision markings!  Setting runway markings to visual!");
            rwy.marking[rwhalf]=1;
        }

        double start1_pct = 0.0;
        double end1_pct = 0.0;
        double heading = 0.0;
        string rwname;

        //
        // Displaced threshold if it exists
        //
        if (rwhalf == 0) {
            heading = rwy.heading + 180.0;
            rwname = rwy.rwnum[0];
        }
        else if (rwhalf == 1) {
            heading = rwy.heading;
            rwname = rwy.rwnum[1];
        }

        SG_LOG( SG_GENERAL, SG_DEBUG, "runway marking = " << rwy.marking[rwhalf] );
        if ( rwy.threshold[rwhalf] > 0.0 ) {
            SG_LOG( SG_GENERAL, SG_DEBUG, "Displaced threshold for RW side " << rwhalf << " is " << rwy.threshold[rwhalf] );

            // reserve 90' for final arrows
            double thresh = rwy.threshold[rwhalf] - 90.0 * SG_FEET_TO_METER;

            // number of full center arrows
            int count = (int)(thresh / 200.0 * SG_FEET_TO_METER);

            // length of starting partial arrow
            double part_len = thresh - ( count * 200.0 * SG_FEET_TO_METER);
            double tex_pct = (200.0 * SG_FEET_TO_METER - part_len) / 200.0 * SG_FEET_TO_METER;

            // starting (possibly partial chunk)
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( part_len / length );
            gen_runway_section( runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, tex_pct, 1.0,
                            heading,
                            "dspl_thresh",
                            rwy_polys, texparams, 
                            &shoulder_polys, &shoulder_tps,
                            accum, slivers,
                            make_shapefiles );

            // main chunks
            for ( int i = 0; i < count; ++i ) {
                start1_pct = end1_pct;
                end1_pct = start1_pct + ( 200.0 * SG_FEET_TO_METER / length );
                gen_runway_section( runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                "dspl_thresh",
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );
            }

            // final arrows
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( 90.0 * SG_FEET_TO_METER / length );
            gen_runway_section( runway_half,
                            start1_pct, end1_pct,
                            0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            "dspl_arrows",
                            rwy_polys, texparams, 
                            &shoulder_polys, &shoulder_tps,
                            accum, slivers,
                            make_shapefiles );
        }

        if (rwy.marking[rwhalf] == 0) {
            // No marking
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( 10 / length );
            gen_runway_section( runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                "no_threshold",
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );
        } else {
            // Thresholds for all others
            start1_pct = end1_pct;
            end1_pct = start1_pct + ( 202.0 * SG_FEET_TO_METER / length );
            gen_runway_section( runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                "threshold",
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );
        }

        // Runway designation block
        gen_rw_designation( runway_half, heading,
                            rwname, start1_pct, end1_pct, 
                            rwy_polys, texparams, accum, slivers,
                            make_shapefiles );

        // Generate remaining markings depending on type of runway
        if (rwy.marking[rwhalf] > 1) {
            std::vector<sections> rw_marking_list;
            rw_marking_list.clear();

            switch ( rwy.marking[rwhalf] ) {
                case 2:
    	            rw_marking_list.insert(  rw_marking_list.begin(), nprec, nprec + sizeof(nprec) / sizeof(nprec[0]) );
                    break;

                case 3:
                    rw_marking_list.insert(  rw_marking_list.begin(), prec, prec + sizeof(prec) / sizeof(prec[0]) );
                    break;

                case 4:
                    rw_marking_list.insert(  rw_marking_list.begin(), uk_nprec, uk_nprec + sizeof(uk_nprec) / sizeof(uk_nprec[0]) );
                    break;

                case 5:
                    rw_marking_list.insert(  rw_marking_list.begin(), uk_prec, uk_prec + sizeof(uk_prec) / sizeof(uk_prec[0]) );
                    break;
            }

            // Now create the marking sections of the runway type
            for ( unsigned int i=0; i < rw_marking_list.size(); ++i) {
	            SG_LOG(SG_GENERAL, SG_DEBUG, "Runway section texture = " << rw_marking_list[i].tex << " start_pct: " << start1_pct << " end_pct: " << end1_pct << " length: " << rw_marking_list[i].size);

                if ( end1_pct < 1.0 ) {
                    start1_pct = end1_pct;
                    end1_pct = start1_pct + ( rw_marking_list[i].size / length );
                    gen_runway_section( runway_half,
                                        start1_pct, end1_pct,
                                        0.0, 1.0,
                                        0.0, 1.0, 0.0, 1.0,
                                        heading,
                                        rw_marking_list[i].tex,
                                        rwy_polys, texparams, 
                                        &shoulder_polys, &shoulder_tps,
                                        accum, slivers,
                                        make_shapefiles );
                }
            }
        }

        //
        // The rest ...
        //

        // fit the 'rest' texture in as many times as will go evenly into
        // the remaining distance so we don't end up with a super short
        // section at the end.
        double ideal_rest_inc = ( 200.0 * SG_FEET_TO_METER / length );
        int divs = (int)((1.0 - end1_pct) / ideal_rest_inc) + 1;
        double rest1_inc = (1.0 - end1_pct) / divs;

        while ( end1_pct < 1.0 ) {
            start1_pct = end1_pct;
            end1_pct = start1_pct + rest1_inc;

            gen_runway_section( runway_half,
                                start1_pct, end1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                "rest",
                                rwy_polys, texparams, 
                                &shoulder_polys, &shoulder_tps,
                                accum, slivers,
                                make_shapefiles );
        }

        start1_pct = 0.0;
        end1_pct = 0.0;
        
        double part_len = 0.0;
        int count=0;

        if (rwy.overrun[rwhalf] > 0.0) {
            /* Generate approach end overrun */
            count = (int) (rwy.overrun[rwhalf] * 2.0/ rwy.width);
            if(count < 1) {
                count = 1;
            }

            part_len = rwy.overrun[rwhalf] / (double)count;
            for(int i=0; i<count; i++) {
                start1_pct=end1_pct;
                end1_pct = start1_pct + ( part_len / length );
            
                gen_runway_section( runway_half,
                                    -end1_pct, -start1_pct,
                                    0.0, 1.0,
                                    0.0, 1.0, 0.0, 1.0, //last number is lengthwise
                                    heading,
                                    "stopway",
                                    rwy_polys, texparams,
                                    &shoulder_polys, &shoulder_tps,
                                    accum, slivers,
                                    make_shapefiles );
            }
        }
    }
}

void Runway::BuildShoulder( superpoly_list *rwy_polys,
                            texparams_list *texparams,
                            ClipPolyType *accum,
                            poly_list& slivers, 
                            TGPolygon* apt_base, 
                            TGPolygon* apt_clearing )
{
    TGPolygon base, safe_base;
    TGPolygon shoulder;

    for (unsigned int i=0; i<shoulder_polys.size(); i++) {
        shoulder = shoulder_polys[i].get_poly();

        // Clip the new polygon against what ever has already been created.
        TGPolygon clipped = tgPolygonDiffClipper( shoulder, *accum );
        tgPolygonFindSlivers( clipped, slivers );

        // Split long edges to create an object that can better flow with
        // the surface terrain
        TGPolygon split = tgPolygonSplitLongEdges( clipped, 400.0 );
        shoulder_polys[i].set_poly( split );
        
        rwy_polys->push_back( shoulder_polys[i] );
        texparams->push_back( shoulder_tps[i] );

        *accum = tgPolygonUnionClipper( shoulder, *accum );

        if (apt_base)
        {
            // also clear a safe area around the runway
            base      = tgPolygonExpand( shoulder, 20.0); 
            safe_base = tgPolygonExpand( shoulder, 50.0); 

            // add this to the airport clearing
            *apt_clearing = tgPolygonUnionClipper( safe_base, *apt_clearing );

            // and add the clearing to the base
            *apt_base = tgPolygonUnionClipper( base, *apt_base );
        }
    }
}
