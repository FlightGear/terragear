/****************************************************************
 *
 * MODULE:     v.contri
 *
 * AUTHOR(S):  Ralf Gerlich
 *
 * PURPOSE:    Export a TIN as FlightGear .btg.gz-file
 *
 * COPYRIGHT:  (C) 2010 by Ralf Gerlich
 *
 *             This program is free software under the
 *             GNU General Public License (>=v2).
 *             Read the file COPYING that comes with GRASS
 *             for details.
 *
 ****************************************************************/

#include <stdlib.h>

#include <grass/gis.h>
#include <grass/glocale.h>
#include <grass/vector.h>


int main(int argc, char *argv[])
{
	struct GModule *module;	/* GRASS module for parsing arguments */
	struct Option *old;
	struct Map_info oldmap;
	
	G_gisinit(argv[0]);
	
	/* initialize module */
	module = G_define_module();
	module->description = _("Export a TIN as FlightGear .btg.gz");
	
	/* Define the different options as defined in gis.h */
	old = G_define_standard_option(G_OPT_V_INPUT);
	
	G_define_standard_option(G_OPT_F_OUTPUT);
	
	/* options and flags parser */
	if (G_parser(argc, argv))
		exit(EXIT_FAILURE);

	if ( Vect_open_old( &oldmap, old->answer, NULL ) < 2 ) {
		G_fatal_error("Unable to open vector map \"%s\"",
			      old->answer);
	}
	
	// TODO: stripify triangles by categories
	// TODO: add singular triangles
	// TODO: create texture coordinates
	// TODO: write out btg.gz
	
	Vect_close(&oldmap);
	
	/* Don't forget to report to caller sucessful end of data processing :) */
	exit(EXIT_SUCCESS);
}
