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
#include "runway.hxx"

#include <stdlib.h>

using std::string;


void Runway::gen_rw_designation( const string& material,
                                 TGPolygon poly, double heading, string rwname,
                                 double &start_pct, double &end_pct,
                                 superpoly_list *rwy_polys,
                                 texparams_list *texparams,
                                 ClipPolyType *accum,
                                 poly_list& slivers )
{
    if (rwname != "XX"){ /* Do not create a designation block if the runway name is set to none */
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
                                material, letter,
                                rwy_polys, texparams, accum, slivers );
        }


        // create runway designation number(s)
        if (rwname == "0")
            rwname = "36";
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
                                material, tex1,
                                rwy_polys, texparams, accum, slivers );
            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.5, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                material, tex2,
                                rwy_polys, texparams, accum, slivers );

        } else if (rwname.length() == 1) {
            sprintf( tex1, "%c%c", rwname[0], 'c');

            gen_runway_section( poly,
                                start_pct, end_pct,
                                0.0, 1.0,
                                0.0, 1.0, 0.0, 1.0,
                                heading,
                                material, tex1,
                                rwy_polys, texparams, accum, slivers );
        }
    }
}

// generate the runway overrun area
void Runway::gen_runway_overrun( const TGPolygon& runway_half,
			 int rwhalf,
                         const string& prefix,
                         superpoly_list *rwy_polys,
                         texparams_list *texparams,
                         ClipPolyType* accum,
                         poly_list& slivers ) {
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
                                accum,
                                slivers );
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
                         ClipPolyType *accum,
                         poly_list& slivers  ) {
    gen_tex_section( runway,
                         startl_pct, endl_pct,
                         startw_pct, endw_pct,
                         minu, maxu, minv, maxv,
                         heading, rwy.width, rwy.length,
                         prefix,
                         material,
                         rwy_polys,
                         texparams,
                         accum,
                         slivers  );

}
