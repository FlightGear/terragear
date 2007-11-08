// rwy_common.cxx -- Common runway generation routines
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
// $Id: rwy_common.cxx,v 1.12 2004-11-19 22:25:49 curt Exp $
//

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>

#include "global.hxx"
#include "poly_extra.hxx"
#include "rwy_common.hxx"


void gen_number_block( const TGRunway& rwy_info,
                       const string& material,
                       TGPolygon poly, double heading, int num,
                       double start_pct, double end_pct,
                       superpoly_list *rwy_polys,
                       texparams_list *texparams,
                       TGPolygon *accum )
{
    char tex1[32]; tex1[0] = '\0';
    char tex2[32]; tex2[0] = '\0';

    SG_LOG(SG_GENERAL, SG_DEBUG, "Runway num = " << num);

    if ( num == 0 ) {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Ack! Someone passed in a runway number of '0'" );
        exit(-1);
    }

    if ( num == 11 ) {
	sprintf( tex1, "11" );
    } else if ( num < 10 ) {
	sprintf( tex1, "%dc", num );
    } else {
	sprintf( tex1, "%dl", num / 10 );
	sprintf( tex2, "%dr", num - (num / 10 * 10));
    }

    // printf("tex1 = '%s'  tex2 = '%s'\n", tex1, tex2);

    if ( num < 10 ) {
	gen_runway_section( rwy_info, poly,
			    start_pct, end_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    heading,
			    material, tex1,
			    rwy_polys, texparams, accum );
    } else if ( num == 11 ) {
	gen_runway_section( rwy_info, poly,
			    start_pct, end_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    heading,
			    material, tex1,
			    rwy_polys, texparams, accum );
    } else {
        gen_runway_section( rwy_info, poly,
                            start_pct, end_pct,
                            0.0, 0.5,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, tex1,
                            rwy_polys, texparams, accum );
        gen_runway_section( rwy_info, poly,
                            start_pct, end_pct,
                            0.5, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, tex2,
                            rwy_polys, texparams, accum );
    }
}


// generate a section of runway
void gen_runway_section( const TGRunway& rwy_info,
			 const TGPolygon& runway,
			 double startl_pct, double endl_pct,
			 double startw_pct, double endw_pct,
                         double minu, double maxu, double minv, double maxv,
			 double heading,
			 const string& prefix,
			 const string& material,
			 superpoly_list *rwy_polys,
			 texparams_list *texparams,
			 TGPolygon *accum  ) {

    int j, k;

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

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "pre clipped runway pts " << prefix << material);
    for ( j = 0; j < section.contours(); ++j ) {
	for ( k = 0; k < section.contour_size( j ); ++k ) {
	    Point3D p = section.get_pt(j, k);
	    SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
	}
    }

    // Clip the new polygon against what ever has already been created.
    TGPolygon clipped = tgPolygonDiff( section, *accum );

    // Split long edges to create an object that can better flow with
    // the surface terrain
    TGPolygon split = tgPolygonSplitLongEdges( clipped, 400.0 );

    // Create the final output and push on to the runway super_polygon
    // list
    TGSuperPoly sp;
    sp.erase();
    sp.set_poly( split );
    sp.set_material( prefix + material );
    rwy_polys->push_back( sp );
    SG_LOG(SG_GENERAL, SG_DEBUG, "section = " << clipped.contours());
    *accum = tgPolygonUnion( section, *accum );

    // Store away what we need to know for texture coordinate
    // calculation.  (CLO 10/20/02: why can't we calculate texture
    // coordinates here?  Oh, becuase later we need to massage the
    // polygons to avoid "T" intersections and clean up other
    // potential artifacts and we may add or remove points and need to
    // do new texture coordinate calcs later.

    // we add 2' to the length for texture overlap.  This puts the
    // lines on the texture back to the edge of the runway where they
    // belong.
    double len = rwy_info.length / 2.0 + 2;
    double sect_len = len * ( endl_pct - startl_pct );

    // we add 2' to both sides of the runway (4' total) for texture
    // overlap.  This puts the lines on the texture back to the edge
    // of the runway where they belong.
    double wid = rwy_info.width + 4;
    double sect_wid = wid * ( endw_pct - startw_pct );

    TGTexParams tp;
    tp = TGTexParams( p0,
                      sect_wid * SG_FEET_TO_METER,
                      sect_len * SG_FEET_TO_METER,
                      heading );
    tp.set_minu( minu );
    tp.set_maxu( maxu );
    tp.set_minv( minv );
    tp.set_maxv( maxv );
    texparams->push_back( tp );

    // print runway points
    SG_LOG(SG_GENERAL, SG_DEBUG, "clipped runway pts " << prefix + material);
    for ( j = 0; j < clipped.contours(); ++j ) {
	for ( k = 0; k < clipped.contour_size( j ); ++k ) {
	    Point3D p = clipped.get_pt(j, k);
	    SG_LOG(SG_GENERAL, SG_DEBUG, " point = " << p);
	}
    }
}
