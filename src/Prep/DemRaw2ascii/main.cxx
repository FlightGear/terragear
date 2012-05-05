/* main.cxx -- main loop
 *
 * Written by Curtis Olson, started February 1998.
 * Modified by Geoff McLane, March, 2009
 * to add min, max, lon, lat, to limit the output of ASCII DEM files.
 *
 * Copyright (C) 1998, 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * $Id: main.c,v 1.4 2004-11-19 22:25:51 curt Exp $
 */


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <stdio.h>
#include <string.h>

#ifdef HAVE_UNISTD_H
#  include <unistd.h> /* tmp for read() */
#endif

#include <cstdlib>

extern "C" {    
    #include "rawdem.h"
}

#include <simgear/misc/sg_path.hxx>

static void give_help( char * name )
{
   printf("Usage: %s [OPTIONS] <input_file_basename> <output_dir>\n", name);
   printf("Options:\n");
   printf(" --min-lat=<degs> - set minimum latitude for output.\n");
   printf(" --max-lat=<degs> - set maximum latitude for output.\n");
   printf(" --min-lon=<degs> - set minimum longitude for output.\n");
   printf(" --max-lon=<degs> - set maximum longitude for output.\n");
}

int main(int argc, char **argv) {
    static fgRAWDEM raw;
    static char basename[256], output_dir[256], hdr_file[256], dem_file[256];
    int start_lat, end_lat;
    int i;
    int last_arg = 1;
    double min_lat, max_lat, min_lon, max_lon;

#if defined( _MSC_VER ) && _MSC_VER >= 1400 // set 2-ditit exponent - defaults to 3 - Only available for VS2005
    _set_output_format( _TWO_DIGIT_EXPONENT );
#endif // _MSC_VER

    min_lat = max_lat = min_lon = max_lon = BAD_LATLON;
    for( i = 1; i < argc; i++ )
    {
       char * arg = argv[i];
       if (*arg == '-') {
          // option
          if((strcmp(arg,"-h") == 0)||
             (strcmp(arg,"--help") == 0)) {
                give_help( argv[0] );
                exit(0);
          } else if( strncmp(arg,"--min-lon=", 10) == 0 ) {
             min_lon = atof( &arg[10] );
          } else if( strncmp(arg,"--max-lon=", 10) == 0 ) {
             max_lon = atof( &arg[10] );
          } else if( strncmp(arg,"--min-lat=", 10) == 0 ) {
             min_lat = atof( &arg[10] );
          } else if( strncmp(arg,"--max-lat=", 10) == 0 ) {
             max_lat = atof( &arg[10] );
          } else {
             printf( "ERROR: Unknown argument [%s]. Use -h for help.\n", arg );
             exit(1);
          }
          last_arg = i + 1;
       } else
          break;
    }

    if ( (argc - last_arg) != 2 ) {
       give_help( argv[0] );
       exit(1);
    }
    if(( min_lat != BAD_LATLON )&&
       (( min_lat < -90.0 )||( min_lat > 90.0 ))) {
          printf( "ERROR: Bad min-lat [%f]!\n", min_lat );
          exit(1);
    }
    if(( max_lat != BAD_LATLON )&&
       (( max_lat < -90.0 )||( max_lat > 90.0 ))) {
          printf( "ERROR: Bad max-lat [%f]!\n", max_lat );
          exit(1);
    }
    if(( min_lon != BAD_LATLON )&&
       (( min_lon < -180.0 )||( min_lon > 180.0 ))) {
          printf( "ERROR: Bad min-lon [%f]!\n", min_lon );
          exit(1);
    }
    if(( max_lon != BAD_LATLON )&&
       (( max_lon < -180.0 )||( max_lon > 180.0 ))) {
          printf( "ERROR: Bad max-lon [%f]!\n", max_lon );
          exit(1);
    }
    if(( min_lat != BAD_LATLON )&&
       ( max_lat != BAD_LATLON )&&
       ( min_lat > max_lat ))
    {
       printf( "ERROR: Bad min-lat [%f] NOT less than max-lat [%f]!\n", min_lat, max_lat );
       exit(1);
    }
    if(( min_lon != BAD_LATLON )&&
       ( max_lon != BAD_LATLON )&&
       ( min_lon > max_lon ))
    {
       printf( "ERROR: Bad min-lon [%f] NOT less than max-lon [%f]!\n", min_lon, max_lon );
       exit(1);
    }

    if(( min_lat != BAD_LATLON )||
       ( max_lat != BAD_LATLON )||
       ( min_lon != BAD_LATLON )||
       ( max_lon != BAD_LATLON ))
    {
      printf( "Limited to " );
      if( min_lat != BAD_LATLON )
         printf( "min lat [%f] ", min_lat );
      if( max_lat != BAD_LATLON )
         printf( "max lat [%f] ", max_lat );
      if( min_lon != BAD_LATLON )
         printf( "min lon [%f] ", min_lon );
      if( max_lon != BAD_LATLON )
         printf( "max lon [%f] ", max_lon );
      printf( "\n" );
    }

    /* set any mins and max */
    raw.min_lat = min_lat;
    raw.min_lon = min_lon;
    raw.max_lat = max_lat;
    raw.max_lon = max_lon;

    /* get basename */
    strcpy(basename, argv[last_arg]);

    /* get output dir */
    strcpy(output_dir, argv[last_arg+1]);
    if (!SGPath(output_dir).isDir() ) {
       printf( "ERROR: Ouput directory [%s], does not exist!\n", output_dir );
       exit(1);
    }
    
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


