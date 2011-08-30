/* a test of the Shewchuk triangulator (lib form) */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define REAL double
#include "triangle.h"
#include "tri_support.h"

int main( int argc, char **argv ) {
    struct triangulateio in, out, vorout;
    char basename[256], nodefile[256], polyfile[256];
    FILE *fp;
    int count, dim, attr, bndmrkrs, end1, end2, boundmark;
    int i, counter;
    double x, y, z;
    char tri_options[256];
    int n1, n2, n3;

    /* make sure all elements of these structs point to "NULL" */
    zero_triangulateio( &in );
    zero_triangulateio( &out );
    zero_triangulateio( &vorout );
    
    /* get base name */
    if ( argc == 2 ) {
	strcpy( basename, argv[1] );
    } else {
	printf( "usage: %s base_name\n", argv[0] );
	return -1;
    }

    /*
     * generate file names
     */

    sprintf( nodefile, "%s.node", basename );
    sprintf( polyfile, "%s.poly", basename );

    /*
     * load node file
     */

    if ( (fp = fopen( nodefile, "r" )) == NULL ) {
	printf( "cannot locate file: %s\n", nodefile );
	return -1;
    }

    /* read in points */
    fscanf( fp, "%d %d %d %d\n", &count, &dim, &attr, &bndmrkrs );
    printf( "loading %d points\n", count );
    in.numberofpoints = count;
    in.numberofpointattributes = 1;
    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
    in.pointattributelist = (REAL *) malloc(in.numberofpoints *
					    in.numberofpointattributes *
					    sizeof(REAL));
    in.pointmarkerlist = NULL;

    for ( i = 0; i < count; ++i ) {
	fscanf( fp, "%d %lf %lf %lf\n", &counter, &x, &y, &z );
	printf( "  read = %d %.2f %.2f %.2f\n", counter, x, y, z );
	in.pointlist[2*counter] = x;
	in.pointlist[2*counter + 1] = y;
	in.pointattributelist[counter] = z;
    }
    fclose( fp );

    /*
     * load poly file
     */

    if ( (fp = fopen( polyfile, "r" )) == NULL ) {
	printf( "cannot locate file: %s\n", polyfile );
	return -1;
    }

    /* first line is ignored, points are specified in .node file */
    fscanf( fp, "%d %d %d %d\n", &count, &dim, &attr, &bndmrkrs );

    /* read in segments */
    fscanf( fp, "%d %d\n", &count, &bndmrkrs );
    printf( "loading %d segments\n", count );
    in.numberofsegments = count;
    in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
    in.segmentmarkerlist = (int *) malloc(in.numberofsegments * sizeof(int));

    for ( i = 0; i < count; ++i ) {
	fscanf( fp, "%d %d %d %d\n", &counter, &end1, &end2, &boundmark );
	printf( "  read = %d %d %d %d\n", counter, end1, end2, boundmark );
	in.segmentlist[2*counter] = end1;
	in.segmentlist[2*counter + 1] = end2;
	in.segmentmarkerlist[counter] = boundmark;
    }

    /* read in holes */
    fscanf( fp, "%d\n", &count );
    printf( "loading %d holes\n", count );
    in.numberofholes = count;
    in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));

    for ( i = 0; i < count; ++i ) {
	fscanf( fp, "%d %lf %lf %lf\n", &counter, &x, &y, &z );
	printf( "  read = %d %.2f %.2f %.2f\n", counter, x, y, z );
	in.holelist[2*counter] = x;
	in.holelist[2*counter + 1] = y;
    }

    /* read in regions */
    /* number of regions is always zero for this example */
    fscanf( fp, "%d\n", &count );
    in.numberofregions = 0;
    in.regionlist = NULL;

    fclose( fp );
    
    /* no triangle list */
    in.numberoftriangles = 0;
    in.numberofcorners = 0;
    in.numberoftriangleattributes = 0;
    in.trianglelist = NULL;
    in.triangleattributelist = NULL;
    in.trianglearealist = NULL;
    in.neighborlist = NULL;

    /* no edge list */
    in.numberofedges = 0;
    in.edgelist = NULL;
    in.edgemarkerlist = NULL;
    in.normlist = NULL;

    /* dump the results */
    print_tri_data( &in );

    /* Triangulate the points.  Switches are chosen to read and write
     * a PSLG (p), number everything from zero (z), and produce an
     * edge list (e), and a triangle neighbor list (n).  no new points
     * on boundary (Y), no internal segment splitting (YY), no quality
     * refinement (q) and Quite (Q)
     */

    strcpy( tri_options, "VVVpzYYenQ" );
    printf( "Triangulation with options = %s\n", tri_options );

    triangulate( tri_options, &in, &out, &vorout );

    zero_triangulateio( &out );
    zero_triangulateio( &vorout );

    triangulate( tri_options, &in, &out, &vorout );

    /* print resulting triangles */
    for ( i = 0; i < out.numberoftriangles; ++i ) {
	n1 = out.trianglelist[i * 3];
	n2 = out.trianglelist[i * 3 + 1];
	n3 = out.trianglelist[i * 3 + 2];
	if ( out.numberoftriangleattributes > 0 ) {
	    z = out.triangleattributelist[i];
	} else {
	    z = 0.0;
	}
	printf( "triangle %d = %d %d %d (%.2f)\n", i, n1, n2, n3, z );
    }

    /* free mem allocated Floating point roundoff is of magnitude 1.1102230246251565e-16
to the "Triangle" structures */
    free(in.pointlist);
    free(in.pointattributelist);
    free(in.pointmarkerlist);
    free(in.regionlist);
    free(out.pointlist);
    free(out.pointattributelist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.triangleattributelist);
    /* free(out.trianglearealist); */
    free(out.neighborlist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);
    free(out.edgelist);
    free(out.edgemarkerlist);
    free(vorout.pointlist);
    free(vorout.pointattributelist);
    free(vorout.edgelist);
    free(vorout.normlist);

    return 0;
}
