// tri_support.c -- supporting routines for the triangulation library
//
// Written by Curtis Olson, started May 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - curt@flightgear.org
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$


#include <stdio.h>

#include "tri_support.h"


void write_tri_data( struct triangulateio *out ) {
    int i, j;
    FILE *node, *ele, *fp;

    node = fopen("tile.node", "w");
    fprintf(node, "%d 2 %d 0\n", 
	    out->numberofpoints, out->numberofpointattributes);
    for (i = 0; i < out->numberofpoints; ++i) {
	fprintf(node, "%d %.6f %.6f %.2f\n", 
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
	    fprintf(ele, "%.6f ", 
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
	fprintf(fp, "%d %.6f %.6f\n", 
		i, out->holelist[2*i], out->holelist[2*i + 1]);
    }
    fprintf(fp, "%d\n", out->numberofregions);
    for (i = 0; i < out->numberofregions; ++i) {
	fprintf(fp, "%d %.6f %.6f %.6f\n", 
		i, out->regionlist[4*i], out->regionlist[4*i + 1],
		out->regionlist[4*i + 2]);
    }
    fclose(fp);
}


