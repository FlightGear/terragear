
/****************************************************************
 *
 * MODULE:     v.contri
 *
 * AUTHOR(S):  Ralf Gerlich
 *
 * PURPOSE:    Create a conforming delauney triangulation from a
 *             vector map
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

/* #define SINGLE */

#ifdef SINGLE
#define REAL float
#else /* not SINGLE */
#define REAL double
#endif /* not SINGLE */

#include "triangle.h"


void build_segments(struct triangulateio* tri, struct Map_info* map)
{
	plus_t node_idx, line_idx;
	const plus_t node_num = Vect_get_num_nodes(map);
	const plus_t line_num = Vect_get_num_lines(map);

	struct line_pnts *pnts = Vect_new_line_struct();

	int index, point_idx, segment_idx;
	int n1, n2, a1, a2;
	
	/* Collect points and segments */
	/* The nodes are shared between edges, but the other points are
	 * individual, so we place the nodes at the start for easy indexing
	 * and add the other points lazily.
	 *
	 * FIXME: actually, centroids are also nodes, but should not be
	 *        included as vertices.
	 */
	tri->numberofpoints = node_num;
	tri->numberofsegments = 0;
	tri->numberofpointattributes = 1; // We store the z-coordinate there
	for ( line_idx = 1; line_idx <= line_num; line_idx++ ) {
		Vect_reset_line( pnts );
		const int type = Vect_read_line( map, pnts, NULL, line_idx );
		if ( type == GV_POINT ) {
			tri->numberofpoints++;
		} else if ( type == GV_BOUNDARY ) {
			tri->numberofpoints+=pnts->n_points-2;
			tri->numberofsegments+=pnts->n_points-1;
		}
	}
	tri->pointlist = (REAL*) malloc( sizeof(REAL[2])*tri->numberofpoints );
	tri->pointattributelist = (REAL*) malloc( sizeof(REAL)*tri->numberofpoints );
	tri->segmentlist = (int*) malloc( sizeof(int[2])*tri->numberofsegments );
	tri->segmentmarkerlist = (int*) malloc( sizeof(int)*tri->numberofsegments );

	/* Add the nodes first */
	for ( node_idx = 1; node_idx <= node_num; node_idx++ ) {
		Vect_get_node_coor( map, node_idx,
				    &tri->pointlist[ 2*node_idx - 2 ],
				    &tri->pointlist[ 2*node_idx - 1 ],
				    &tri->pointattributelist[ node_idx - 1] );
	}
	
	point_idx = node_num;
	segment_idx = 0;
	for ( line_idx = 1; line_idx <= line_num; line_idx++ ) {
		Vect_reset_line( pnts );
		const int type = Vect_read_line( map, pnts, NULL, line_idx );
		if ( type == GV_POINT ) {
			tri->pointlist[ 2*point_idx + 0 ] = pnts->x[0];
			tri->pointlist[ 2*point_idx + 1 ] = pnts->y[0];
			tri->pointattributelist[ point_idx ] = pnts->z[0];
			point_idx++;
		} else if ( type == GV_BOUNDARY ) {
			Vect_get_line_nodes( map, line_idx, &n1, &n2 );
			Vect_get_line_areas( map, line_idx, &a1, &a2 );
			int last_index = n1-1;
			int segmarker = (a1==0 || a2==0 ? 1 : 0);
			for ( index = 1; index < pnts->n_points-1; index++ ) {
				tri->pointlist[ 2*point_idx + 0 ] = pnts->x[index];
				tri->pointlist[ 2*point_idx + 1 ] = pnts->y[index];
				tri->pointattributelist[ point_idx ] = pnts->y[index];
				tri->segmentlist[ 2 * segment_idx + 0 ] = last_index;
				tri->segmentlist[ 2 * segment_idx + 1 ] = point_idx;
				tri->segmentmarkerlist[ segment_idx ] = segmarker;
				segment_idx++;
				last_index = point_idx;
				point_idx++;
			}
			tri->segmentlist[ 2 * segment_idx + 0 ] = last_index;
			tri->segmentlist[ 2 * segment_idx + 1 ] = n2-1;
			tri->segmentmarkerlist[ segment_idx ] = segmarker;
			segment_idx++;
		}
	}

	Vect_destroy_line_struct( pnts );
}

void build_regions(struct triangulateio* tri, struct Map_info* map)
{
	plus_t area_idx;
	const plus_t area_num = Vect_get_num_areas(map);

	struct line_pnts *pnts = Vect_new_line_struct();

	int region_idx, hole_idx;
	
	tri->numberofregions = 0;
	tri->numberofholes = 0;
	for ( area_idx = 0; area_idx <= area_num; area_idx++ ) {
		if ( !Vect_area_alive( map, area_idx) ) {
			continue;
		}
		if ( Vect_get_area_centroid( map, area_idx ) > 0 ) {
			tri->numberofregions++;
		} else {
			tri->numberofholes++;
		}
	}
	tri->regionlist = (REAL*) malloc(sizeof(REAL[4]) * tri->numberofregions);
	tri->holelist = (REAL*) malloc(sizeof(REAL[2]) * tri->numberofholes);
	region_idx = 0;
	hole_idx = 0;
	for ( area_idx = 0; area_idx <= area_num; area_idx++ ) {
		if ( !Vect_area_alive( map, area_idx) ) {
			continue;
		}
		const int centroid_idx = Vect_get_area_centroid( map, area_idx );
		if ( centroid_idx > 0 ) {
			Vect_reset_line( pnts );
			Vect_read_line( map, pnts, NULL, centroid_idx );
			tri->regionlist[ 4*region_idx + 0 ] = pnts->x[0];
			tri->regionlist[ 4*region_idx + 1 ] = pnts->y[0];
			tri->regionlist[ 4*region_idx + 2 ] = centroid_idx;
			tri->regionlist[ 4*region_idx + 3 ] = -1.0; // maximum area, unused
			region_idx++;
		} else {
			Vect_get_point_in_area( map,
						area_idx,
						&tri->holelist[ 2*hole_idx + 0],
						&tri->holelist[ 2*hole_idx + 1] );
			hole_idx++;
		}
	}

	Vect_destroy_line_struct( pnts );
}

void build_triangulateio(struct triangulateio* tri, struct Map_info* map)
{
	build_segments(tri, map);
	build_regions(tri, map);
}

void build_outputvector(struct Map_info* newmap,
			struct triangulateio* tri,
			struct Map_info* oldmap)
{
	int index;
	struct line_pnts* points = Vect_new_line_struct();
	struct line_cats* cats = Vect_new_cats_struct();

	/* Write out the segments */
	for ( index = 0; index < tri->numberofedges; index++ ) {
		int * const edge = &tri->edgelist[ 2 * index ];
		Vect_reset_line( points );
		Vect_append_point( points,
				   tri->pointlist[ 2 * edge[0] + 0 ],
				   tri->pointlist[ 2 * edge[0] + 1 ],
				   tri->pointattributelist[ edge[0] ]);
		Vect_append_point( points,
				   tri->pointlist[ 2 * edge[1] + 0 ],
				   tri->pointlist[ 2 * edge[1] + 1 ],
				   tri->pointattributelist[ edge[1] ]);
		Vect_write_line( newmap, GV_BOUNDARY, points, cats );
	}

	/* Write out the region markers */
	for ( index = 0; index < tri->numberoftriangles; index++ ) {
		int * const triele = &tri->trianglelist[ index * tri->numberofcorners ];
		REAL x, y, z;
		x = ( tri->pointlist[ 2 * triele[0] + 0 ] +
		      tri->pointlist[ 2 * triele[1] + 0 ] +
		      tri->pointlist[ 2 * triele[2] + 0 ] ) / 3.0;
		y = ( tri->pointlist[ 2 * triele[0] + 1 ] +
		      tri->pointlist[ 2 * triele[1] + 1 ] +
		      tri->pointlist[ 2 * triele[2] + 1 ] ) / 3.0;
		z = ( tri->pointattributelist[ triele[0] ] +
		      tri->pointattributelist[ triele[1] ] +
		      tri->pointattributelist[ triele[2] ] ) / 3.0;
		Vect_reset_line( points );
		Vect_reset_cats( cats );
		Vect_read_line( oldmap,
				NULL,
				cats,
				(plus_t) tri->triangleattributelist[ index * tri->numberoftriangleattributes ]);
		Vect_append_point( points, x, y, z );
		Vect_write_line( newmap, GV_CENTROID, points, cats );
	}
}

void build_outputholes(struct Map_info* newmap,
		       struct triangulateio* tri)
{
	int index;
	struct line_pnts* points = Vect_new_line_struct();
	struct line_cats* cats = Vect_new_cats_struct();

	/* Write out the hole markers */
	for ( index = 0; index < tri->numberofholes; index++ ) {
		REAL * const hole = &tri->holelist[ 2 * index ];
		Vect_reset_line( points );
		Vect_append_point( points, hole[0], hole[1], 0.0 );
		Vect_write_line( newmap, GV_POINT, points, cats );
	}
}

int main(int argc, char *argv[])
{
	struct GModule *module;	/* GRASS module for parsing arguments */
	struct Option *old, *new;
	struct Map_info oldmap, newmap;
	
	G_gisinit(argv[0]);
	
	/* initialize module */
	module = G_define_module();
	module->description = _("Create a conforming delauney triangulation from a vector");
	
	/* Define the different options as defined in gis.h */
	old = G_define_standard_option(G_OPT_V_INPUT);
	
	new = G_define_standard_option(G_OPT_V_OUTPUT);
	
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

	struct triangulateio in, out;

	build_triangulateio(&in, &oldmap);

	out.pointlist = (REAL*) NULL;
	out.pointattributelist = (REAL*) NULL;
	out.pointmarkerlist = (int*) NULL;
	out.trianglelist = (int*) NULL;
	out.triangleattributelist = (REAL*) NULL;
	out.neighborlist = (int*) NULL;
	out.segmentlist = (int*) NULL;
	out.segmentmarkerlist = (int*) NULL;
	out.edgelist = (int*) NULL;
	out.edgemarkerlist = (int*) NULL;

	triangulate("pzAejC", &in, &out, NULL);

	build_outputvector(&newmap, &out, &oldmap);
	//build_outputholes(&newmap, &in);

	Vect_build(&newmap);

	Vect_close(&oldmap);
	Vect_close(&newmap);
	
	/* Don't forget to report to caller sucessful end of data processing :) */
	exit(EXIT_SUCCESS);
}
