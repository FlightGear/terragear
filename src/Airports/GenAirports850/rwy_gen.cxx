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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
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


// generate a section of texture
void Runway::gen_runway_section( const TGPolygon& runway,
                                 double startl_pct, double endl_pct,
                                 double startw_pct, double endw_pct,
                                 double minu, double maxu, double minv, double maxv,
                                 double heading,
                                 const string& material,
                                 superpoly_list *rwy_polys,
                                 texparams_list *texparams,
                                 ClipPolyType *accum,
                                 poly_list& slivers ) 
{
    int j, k;
    double width = rwy.width;
    double length = rwy.length;

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

    dwx = t1.x() - t3.x();
    dwy = t1.y() - t3.y();

    Point3D p2 = Point3D( t3.x() + dwx * startw_pct,
                          t3.y() + dwy * startw_pct, 0);

    Point3D p3 = Point3D( t3.x() + dwx * endw_pct,
                          t3.y() + dwy * endw_pct, 0);

    TGPolygon section;
    section.erase();

    section.add_node( 0, p2 );
    section.add_node( 0, p0 );
    section.add_node( 0, p1 );
    section.add_node( 0, p3 );
    section = snap( section, gSnap );

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
}

void Runway::gen_rw_designation( TGPolygon poly, double heading, string rwname,
                                 double &start_pct, double &end_pct,
                                 superpoly_list *rwy_polys,
                                 texparams_list *texparams,
                                 ClipPolyType *accum,
                                 poly_list& slivers )
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
                                rwy_polys, texparams, accum, slivers );
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
                                rwy_polys, texparams, accum, slivers );
            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.5, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex2,
                                rwy_polys, texparams, accum, slivers );

        } else if (rwname.length() == 1) {
            sprintf( tex1, "%c%c", rwname[0], 'c');

            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                tex1,
                                rwy_polys, texparams, accum, slivers );
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
                      poly_list& slivers )
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
                            rwy_polys, texparams, accum, slivers );

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
                                rwy_polys, texparams, accum, slivers );
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
                            rwy_polys, texparams, accum, slivers );
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
                                rwy_polys, texparams, accum, slivers );
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
                                rwy_polys, texparams, accum, slivers );
        }

        // Runway designation block
        gen_rw_designation( runway_half, heading,
                            rwname, start1_pct, end1_pct, 
                            rwy_polys, texparams, accum, slivers );

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
                                        rwy_polys, texparams, accum, slivers );
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
                                rwy_polys, texparams, accum, slivers );
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
                                    rwy_polys,
                                    texparams,
                                    accum,
                                    slivers );
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

    string shoulder_surface = "";
    double shoulder_width = 0.0f;

    if (rwy.shoulder > 0){  
        // Add a shoulder to the runway
        shoulder_width = 11.0f;

        if (rwy.shoulder == 1){
            shoulder_surface = "pa_shoulder";
        } else if (rwy.shoulder == 2){
            shoulder_surface = "pc_shoulder";
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "Unknown shoulder surface code = " << rwy.shoulder );
        }
    } else {  
        // We add a fake shoulder if the runway has an asphalt or concrete surface
        shoulder_width = 1.0;
        if (rwy.surface == 1){
            shoulder_surface = "pa_shoulder_f";
        } else if (rwy.surface == 2){
            shoulder_surface = "pc_shoulder_f";
        }
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "Shoulder width = " << shoulder_width );
    SG_LOG(SG_GENERAL, SG_DEBUG, "Shoulder surface is: " << shoulder_surface );

    if (shoulder_width > 0.0f) {
        // we need to break these shoulders up into managable pieces, as we're asked to triangulate
        // 3-4 km long by 1m wide strips - jrs-can't handle it.
        double max_dist = (double)shoulder_width * 25.0f;
        if (max_dist > 100.0f) {
            max_dist = 100.0f;
        }
        int    numSegs = (rwy.length / max_dist) + 1;
        double dist    = rwy.length / numSegs;

        TGPolygon   poly;
        TGSuperPoly sp;
        TGTexParams tp;

        double lat = 0.0f;
        double lon = 0.0f;
        double r   = 0.0f;
        Point3D inner_start, inner_end;
        Point3D outer_start, outer_end;
        Point3D curInnerLoc, nextInnerLoc;
        Point3D curOuterLoc, nextOuterLoc;

        // Create two paralell lines from start position to end position, and interpolate in between
        // many airports line the taxiway directly to the corner of the runway.  This can create problems, 
        // so extend the shoulders 0.5 meters past each end of the runway
        for (int i=0; i<2; i++) {
            double rev_hdg =  rwy.heading - 180.0;
            if ( rev_hdg < 0 ) { rev_hdg += 360.0; }

            if (i == 0) {
                // left side
                double left_hdg = rwy.heading - 90.0;
                if ( left_hdg < 0 ) { left_hdg += 360.0; }

                geo_direct_wgs_84 ( 0, rwy.lat[0], rwy.lon[0], left_hdg, rwy.width*.5, &lat, &lon, &r );
                geo_direct_wgs_84 ( 0, lat, lon, rev_hdg, 0.5f, &lat, &lon, &r );

                inner_start = Point3D( lon, lat, 0.0f );
                geo_direct_wgs_84 ( 0, lat, lon, rwy.heading, rwy.length+1.0f, &lat, &lon, &r );
                inner_end   = Point3D( lon, lat, 0.0f );

                geo_direct_wgs_84 ( 0, rwy.lat[0], rwy.lon[0], left_hdg, rwy.width*.5 + shoulder_width, &lat, &lon, &r );
                geo_direct_wgs_84 ( 0, lat, lon, rev_hdg, 0.5f, &lat, &lon, &r );

                outer_start = Point3D( lon, lat, 0.0f );
                geo_direct_wgs_84 ( 0, lat, lon, rwy.heading, rwy.length+1.0f, &lat, &lon, &r );
                outer_end   = Point3D( lon, lat, 0.0f );
            } else {
                // right side
                double right_hdg = rwy.heading + 90.0;
                if ( right_hdg > 360 ) { right_hdg -= 360.0; }

                geo_direct_wgs_84 ( 0, rwy.lat[0], rwy.lon[0], right_hdg, rwy.width*.5, &lat, &lon, &r );
                geo_direct_wgs_84 ( 0, lat, lon, rev_hdg, 0.5f, &lat, &lon, &r );

                inner_start = Point3D( lon, lat, 0.0f );
                geo_direct_wgs_84 ( 0, lat, lon, rwy.heading, rwy.length+1.0f, &lat, &lon, &r );
                inner_end   = Point3D( lon, lat, 0.0f );

                geo_direct_wgs_84 ( 0, rwy.lat[0], rwy.lon[0], right_hdg, rwy.width*.5 + shoulder_width, &lat, &lon, &r );
                geo_direct_wgs_84 ( 0, lat, lon, rev_hdg, 0.5f, &lat, &lon, &r );

                outer_start = Point3D( lon, lat, 0.0f );
                geo_direct_wgs_84 ( 0, lat, lon, rwy.heading, rwy.length+1.0f, &lat, &lon, &r );
                outer_end   = Point3D( lon, lat, 0.0f );
            }

            curInnerLoc = inner_start;
            curOuterLoc = outer_start;

            for (int p=0; p<numSegs; p++)
            {
                // calculate next locations
                nextInnerLoc = CalculateLinearLocation( inner_start, inner_end, (1.0f/numSegs) * (p+1) );                    
                nextOuterLoc = CalculateLinearLocation( outer_start, outer_end, (1.0f/numSegs) * (p+1) );                    

                // Generate a poly
                poly.erase();
                if (i == 0 ) {
                    poly.add_node( 0, curInnerLoc );
                    poly.add_node( 0, nextInnerLoc );
                    poly.add_node( 0, nextOuterLoc );
                    poly.add_node( 0, curOuterLoc );
                } else {
                    poly.add_node( 0, curOuterLoc );
                    poly.add_node( 0, nextOuterLoc );
                    poly.add_node( 0, nextInnerLoc );
                    poly.add_node( 0, curInnerLoc );
                }

                TGPolygon clipped = tgPolygonDiffClipper( poly, *accum );
                tgPolygonFindSlivers( clipped, slivers );

                sp.erase();
                sp.set_poly( clipped );
                sp.set_material( shoulder_surface );
                rwy_polys->push_back( sp );

                *accum = tgPolygonUnionClipper( poly, *accum );

                tp = TGTexParams( poly.get_pt(0,0), shoulder_width, dist, rwy.heading );
                tp.set_maxv(dist);
                // reverse u direction for right side
                if ( i == 1 ) {
                    tp.set_maxu(0);
                    tp.set_minu(1);
                }
                texparams->push_back( tp );

                // Add to base / safe base
                base      = tgPolygonExpand( poly, 20.0f );
                safe_base = tgPolygonExpand( poly, 50.0f );

                // add this to the airport clearing
                *apt_clearing = tgPolygonUnionClipper(safe_base, *apt_clearing);

                // and add the clearing to the base
                *apt_base = tgPolygonUnionClipper( base, *apt_base );

                // now set cur locations for the next iteration
                curInnerLoc = nextInnerLoc;
                curOuterLoc = nextOuterLoc;
            }
       }
    }
}
