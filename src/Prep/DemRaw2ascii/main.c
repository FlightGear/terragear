/* main.cxx -- main loop
 *
 * Written by Curtis Olson, started February 1998.
 *
 * Copyright (C) 1998, 1999  Curtis L. Olson  - curt@flightgear.org
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * $Id$
 */


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <stdio.h>
#include <string.h>

#ifdef HAVE_UNISTD_H
#  include <unistd.h> /* tmp for read() */
#endif
#ifdef HAVE_STDLIB_H
#  include <stdlib.h>
#endif

#include "rawdem.h"


int main(int argc, char **argv) {
    fgRAWDEM raw;
    char basename[256], output_dir[256], hdr_file[256], dem_file[256];
    int start_lat, end_lat;
    int i;
    // double total;
    // unsigned char buf[2];
    // short int value;

    if ( argc != 3 ) {
	printf("Usage: %s <input_file_basename> <output_dir>\n", argv[0]);
	exit(-1);
    }

    /* get basename */
    strcpy(basename, argv[1]);

    /* get output dir */
    strcpy(output_dir, argv[2]);

    /* generate header file name */
    strcpy(hdr_file, basename);
    strcat(hdr_file, ".HDR");

    /* generate input file name (raw dem) */
    strcpy(dem_file, basename);
    strcat(dem_file, ".DEM");
    
    printf("Header file = %s  Input file = %s\n", hdr_file, dem_file);
    printf("Output Directory = %s\n", output_dir);

    /* scan the header file and extract important values */
    rawReadDemHdr(&raw, hdr_file);

    /* open up the raw data file */
    rawOpenDemFile(&raw, dem_file);

    printf(" char size = %d\n", sizeof(char) );
    printf(" short int size = %d\n", sizeof(short int) );

    /*
    int min, max;
    min = max = 0;
    total = 0;
    for ( j = 0; j < raw.nrows; ++j ) {
	for ( i = 0; i < raw.ncols; ++i ) {
	    if ( read(raw.fd, buf, 2) != 2 ) {
                printf("Ackk, die, exit\n");
                exit(-1);
            }
	    value = ( buf[0] << 8 ) | buf[1];
	    printf("%d\n", value );
	    total += value;
	    if ( value < min ) { min = value; }
	    if ( value > max ) { max = value; }
	}
    }
    printf("min = %d  max = %d\n", min, max );
    printf("average = %.2f\n", total / (raw.nrows * raw.ncols));
    exit(0);
    */

    end_lat = raw.rooty / 3600;
    start_lat = end_lat - ((raw.nrows * raw.ydim) / 3600);
    printf("Latitude ranges from %d to %d\n", start_lat, end_lat);

    for ( i = start_lat + 1; i <= end_lat; i++ ) {
	rawProcessStrip(&raw, i, output_dir);
    }

    /* close the raw data file */
    rawCloseDemFile(&raw);

    return(0);
}


