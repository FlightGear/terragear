
/****************************************************************
 *
 * MODULE:     v.contri
 *
 * AUTHOR(S):  Ralf Gerlich
 *
 * PURPOSE:    Flatten a vector map according to several operators
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

#include <grass/dbmi.h>
#include <grass/vector.h>


void apply_setz_operator(struct Map_info* map, double z, int* cats, int ncats, int layer)
{
    const plus_t area_count = Vect_get_num_areas( map );
    plus_t area_idx;
    struct line_cats *lcats = Vect_new_cats_struct();

    for (area_idx = 0; area_idx <= area_count; area_idx++) {        
            if ( !Vect_area_alive( map, area_idx ) ) {
                /* Skip dead areas */
                continue;
            }
            if ( ncats >= 0 ) {
                // TODO: check whether the area has the given categories 
            }
            // TODO: apply setz
    }
}

void apply_mean_operator(struct Map_info* map, int* cats, int ncats, int layer)
{
    if (ncats > 0) {
        /* We have to greedily collect adjacent selected areas */
        // TODO
    } else if (ncats < 0) {
        /* We have to greedily collect adjacent areas */
        // TODO
    }
}

void apply_slope_operator(struct Map_info* map, double slope, int* cats, int ncats, int layer)
{
    if (ncats > 0) {
        /* We apply slope to individual areas */
        // TODO
    } else if (ncats < 0) {
        /* We apply slope to individual areas */
        // TODO
    }
}

int main(int argc, char *argv[])
{
	struct GModule *module;	/* GRASS module for parsing arguments */
	struct Option *old, *new;
	struct Option *whereopt;
	struct Option *operator, *value, *slope;
	struct Option *fieldopt;
	int field;
	struct Map_info oldmap, newmap;
	int ncats, *cats;
	
	G_gisinit(argv[0]);
	
	/* initialize module */
	module = G_define_module();
	module->description = _("Create a conforming delauney triangulation from a vector");
	
	/* Define the different options as defined in gis.h */
	old = G_define_standard_option(G_OPT_V_INPUT);
	whereopt = G_define_standard_option(G_OPT_DB_WHERE);
	fieldopt = G_define_standard_option(G_OPT_V_FIELD);
	new = G_define_standard_option(G_OPT_V_OUTPUT);
	
	operator = G_define_option();
	operator->key = "operator";
	operator->description = _("Flattening operator to apply");
	operator->type = TYPE_STRING;
	operator->required = YES;
	operator->options = "setz,mean,slope";
	
	value = G_define_option();
	value->key = "value";
	value->description = _("The target value for the 'setz' operator");
	value->type = TYPE_DOUBLE;
	value->required = NO;
	value->answer = "0";
	
	slope = G_define_option();
	slope->key = "slope";
	slope->description = _("The slope value for the 'slope' operator");
	slope->type = TYPE_DOUBLE;
	slope->required = NO;
	slope->answer = "0.4";
	
	/* options and flags parser */
	if (G_parser(argc, argv))
		exit(EXIT_FAILURE);

	if ( Vect_open_old( &oldmap, old->answer, NULL ) < 2 ) {
		G_fatal_error("Unable to open vector map \"%s\"",
			      old->answer);
	}

	if ( Vect_open_new( &newmap, new->answer, Vect_is_3d(&oldmap) )!=1 ) {
		G_fatal_error("Unable to create vector map \"%s\"",
			      new->answer);
	}

	Vect_copy_head_data( &oldmap, &newmap );
	Vect_copy_tables( &oldmap, &newmap, 0 );
	Vect_copy_map_lines( &oldmap, &newmap );

	Vect_close(&oldmap);

	if (whereopt->answer!=NULL) {
		field = Vect_get_field_number( &newmap, fieldopt->answer );
		struct field_info* fieldinfo = Vect_get_field( &newmap, field );
		if (!fieldinfo) {
			G_fatal_error(_("Database connection not defined for layer <%s>"),
							fieldopt->answer);
		}

		G_debug(1, "Loading categories from table <%s>", fieldinfo->table);

		dbDriver* driver = db_start_driver_open_database(fieldinfo->driver, fieldinfo->database);
		if (driver == NULL) {
			G_fatal_error(_("Unable to open database <%s> by driver <%s>"),
							fieldinfo->database, fieldinfo->driver);
		}
		
		ncats = db_select_int(driver, fieldinfo->table, fieldinfo->key, whereopt->answer,
								&cats);
		if (ncats == -1) {
				G_fatal_error(_("Unable select records from table <%s>"), fieldinfo->table);
		}
		G_message(_("%d categories loaded from table <%s>"), ncats,
					fieldinfo->table);

		db_close_database(driver);
		db_shutdown_driver(driver);
	} else {
		field = -1;
		cats = NULL;
		ncats = -1;
	}
	
	if (!strcmp(operator->answer, "setz")) {
	    apply_setz_operator( &newmap, atof(value->answer), cats, ncats, field );
	} else if (!strcmp(operator->answer, "mean")) {
	    apply_mean_operator( &newmap, cats, ncats, field );
	} else if (!strcmp(operator->answer, "slope")) {
	    apply_slope_operator( &newmap, atof(slope->answer), cats, ncats, field );
	}

	Vect_build(&newmap);

	if (cats != NULL) {
		G_free(cats);
	}

	Vect_close(&newmap);
	
	/* Don't forget to report to caller sucessful end of data processing :) */
	exit(EXIT_SUCCESS);
}
