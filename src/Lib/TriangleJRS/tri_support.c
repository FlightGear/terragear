// tri_support.c -- supporting routines for the triangulation library
//
// Written by Curtis Olson, started May 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: tri_support.c,v 1.4 2004-11-19 22:25:50 curt Exp $


#include <stdio.h>

#include "tri_support.h"


void zero_triangulateio( struct triangulateio *in ) {
    in->pointlist = NULL;
    in->pointattributelist = NULL;
    in->pointmarkerlist = NULL;
    in->numberofpoints = 0;
    in->numberofpointattributes = 0;

    in->trianglelist = NULL;
    in->triangleattributelist = NULL;
    in->trianglearealist = NULL; 
    in->neighborlist = NULL;
    in->numberoftriangles = 0;
    in->numberofcorners = 0;
    in->numberoftriangleattributes = 0;

    in->segmentlist = NULL;
    in->segmentmarkerlist = NULL;
    in->numberofsegments = 0;

    in->holelist = NULL;
    in->numberofholes = 0;

    in->regionlist = NULL;
    in->numberofregions = 0;

    in->edgelist = NULL;
    in->edgemarkerlist = NULL;
    in->normlist = NULL;
    in->numberofedges = 0;
}


void print_tri_data( struct triangulateio *out ) {
    int i, j;

    printf( "NODES\n" );
    printf( "%d 2 %d 0\n", 
	    out->numberofpoints, out->numberofpointattributes);
    for ( i = 0; i < out->numberofpoints; ++i ) {
	printf( "%d %.13f %.13f %.2f\n", 
		i, out->pointlist[2*i], out->pointlist[2*i + 1], 0.0);
    }

    printf( "TRIANGLES\n" );
    printf( "%d %d 0\n", out->numberoftriangles, out->numberofcorners );
    for ( i = 0; i < out->numberoftriangles; ++i ) {
        printf( "%d ", i );
        for ( j = 0; j < out->numberofcorners; ++j ) {
	    printf( "%d ", out->trianglelist[i * out->numberofcorners + j] );
        }
        for ( j = 0; j < out->numberoftriangleattributes; ++j ) {
	    printf( "%.13f ", 
		    out->triangleattributelist[i 
					      * out->numberoftriangleattributes
					      + j]
		    );
        }
	printf("\n");
    }

    printf( "SEGMENTS\n" );
    printf( "0 2 1 0\n" );
    printf( "%d 1\n", out->numberofsegments);
    for ( i = 0; i < out->numberofsegments; ++i ) {
	printf( "%d %d %d %d\n", 
		i, out->segmentlist[2*i], out->segmentlist[2*i + 1],
		out->segmentmarkerlist[i] );
    }
    printf( "HOLES\n" );
    printf( "%d\n", out->numberofholes);
    for (i = 0; i < out->numberofholes; ++i) {
	printf( "%d %.13f %.13f\n", 
		i, out->holelist[2*i], out->holelist[2*i + 1] );
    }
    printf( "REGIONS\n" );
    printf( "%d\n", out->numberofregions );
    for ( i = 0; i < out->numberofregions; ++i ) {
	printf( "%d %.13f %.13f %.13f\n", 
		i, out->regionlist[4*i], out->regionlist[4*i + 1],
		out->regionlist[4*i + 2] );
    }

    printf(" EDGES\n" );
    printf( "%d 1\n", out->numberofedges );
    for ( i = 0; i < out->numberofedges; ++i ) {
	printf( "%d %d %d %d\n", i, out->edgelist[2*i], out->edgelist[2*i + 1],
		out->edgemarkerlist[i] );
    }
}


void write_tri_data( struct triangulateio *out ) {
    int i, j;
    FILE *node, *ele, *fp;

    node = fopen("tile.node", "w");
    fprintf(node, "%d 2 %d 0\n", 
	    out->numberofpoints, out->numberofpointattributes);
    for (i = 0; i < out->numberofpoints; ++i) {
	fprintf(node, "%d %.13f %.13f %.2f\n", 
		i, out->pointlist[2*i], out->pointlist[2*i + 1], 0.0);
    }
    fclose(node);

    ele = fopen("tile.ele", "w");
    fprintf(ele, "%d 3 0\n", out->numberoftriangles);
    for (i = 0; i < out->numberoftriangles; ++i) {
        fprintf(ele, "%d ", i);
        for (j = 0; j < out->numberofcorners; ++j) {
	    fprintf(ele, "%d ", out->trianglelist[i * out->numberofcorners + j]);
        }
        for (j = 0; j < out->numberoftriangleattributes; ++j) {
	    fprintf(ele, "%.13f ", 
		    out->triangleattributelist[i 
					      * out->numberoftriangleattributes
					      + j]
		    );
        }
	fprintf(ele, "\n");
    }
    fclose(ele);

    fp = fopen("tile.poly", "w");
    fprintf(fp, "0 2 1 0\n");
    fprintf(fp, "%d 1\n", out->numberofsegments);
    for (i = 0; i < out->numberofsegments; ++i) {
	fprintf(fp, "%d %d %d %d\n", 
		i, out->segmentlist[2*i], out->segmentlist[2*i + 1],
		out->segmentmarkerlist[i] );
    }
    fprintf(fp, "%d\n", out->numberofholes);
    for (i = 0; i < out->numberofholes; ++i) {
	fprintf(fp, "%d %.13f %.13f\n", 
		i, out->holelist[2*i], out->holelist[2*i + 1]);
    }
    fprintf(fp, "%d\n", out->numberofregions);
    for (i = 0; i < out->numberofregions; ++i) {
	fprintf(fp, "%d %.13f %.13f %.13f\n", 
		i, out->regionlist[4*i], out->regionlist[4*i + 1],
		out->regionlist[4*i + 2]);
    }
    fclose(fp);
}


