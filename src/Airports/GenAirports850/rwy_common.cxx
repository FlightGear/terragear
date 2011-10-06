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

#include "apt_math.hxx"
#include "global.hxx"
#include "poly_extra.hxx"
#include "runway.hxx"

#include <stdlib.h>

using std::string;


void Runway::gen_number_block( const string& material,
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

    if ( num < 10 || num == 11 ) {
	gen_runway_section( poly,
			    start_pct, end_pct,
			    0.0, 1.0,
                            0.0, 1.0, 0.0, 1.0,
			    heading,
			    material, tex1,
			    rwy_polys, texparams, accum );
    } else {
        gen_runway_section( poly,
                            start_pct, end_pct,
                            0.0, 0.5,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, tex1,
                            rwy_polys, texparams, accum );
        gen_runway_section( poly,
                            start_pct, end_pct,
                            0.5, 1.0,
                            0.0, 1.0, 0.0, 1.0,
                            heading,
                            material, tex2,
                            rwy_polys, texparams, accum );
    }
}

// generate the runway overrun area
void Runway::gen_runway_overrun( const TGPolygon& runway_half,
			 int rwhalf,
                         const string& prefix,
                         superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         TGPolygon* accum ) {
    const float length = rwy.length / 2.0 + 2.0 * SG_FEET_TO_METER;
    double start1_pct = 0.0;
    double end1_pct = 0.0;
    double part_len = 0.0;
    double heading = 0.0;
    double overrun = 0.0;

    int count=0;
    int i=0;

    if (rwhalf == 0) {
	    heading = rwy.heading + 180.0;
	    overrun = rwy.overrun[0];
    }
    else if (rwhalf == 1) {
	    heading = rwy.heading;
	    overrun = rwy.overrun[1];
    }

    if (overrun > 0.0) {
        /* Generate approach end overrun */
        count = (int) (overrun * 2.0/ rwy.width);
        if(count < 1) count = 1;
        part_len = overrun / (double) count;
        for(i=0;i<count;i++)
        {
            start1_pct=end1_pct;
            end1_pct = start1_pct + ( part_len / length );
            gen_runway_section( runway_half,
                                - end1_pct, -start1_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0, //last number is lengthwise
                                heading,
                                prefix,
                                "stopway",
                                rwy_polys,
                                texparams,
                                accum);
        }
    }

}
// call the generic build function with runway-specific parameters
void Runway::gen_runway_section( const TGPolygon& runway,
                         double startl_pct, double endl_pct,
                         double startw_pct, double endw_pct,
                         double minu, double maxu, double minv, double maxv,
                         double heading,
                         const string& prefix,
                         const string& material,
                         superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         TGPolygon *accum  ) {
    gen_tex_section( runway,
                         startl_pct, endl_pct,
                         startw_pct, endw_pct,
                         minu, maxu, minv, maxv,
                         heading, rwy.width, rwy.length,
                         prefix,
                         material,
                         rwy_polys,
                         texparams,
                         accum  );

}
