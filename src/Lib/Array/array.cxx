// array.cxx -- Array management class
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998 - 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <cstring>

#include <simgear/compiler.h>
#include <simgear/misc/sgstream.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/strutils.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/lowlevel.hxx>

#include "array.hxx"

using std::string;


TGArray::TGArray( void ):
  array_in(NULL),
  fitted_in(NULL),
      in_data(NULL)
{

}


TGArray::TGArray( const string &file ):
  array_in(NULL),
  fitted_in(NULL),
      in_data(NULL)
{
    TGArray::open(file);
}


// open an Array file (and fitted file if it exists)
bool TGArray::open( const string& file_base ) {
    // open array data file
    string array_name = file_base + ".arr.gz";

    array_in = gzopen( array_name.c_str(), "rb" );
    if (array_in == NULL) {
        return false;
    }

    // open fitted data file
    string fitted_name = file_base + ".fit.gz";
    fitted_in = new sg_gzifstream( fitted_name );
    if ( ! fitted_in->is_open() ) {
        // not having a .fit file is unfortunate, but not fatal.  We
        // can do a really stupid/crude fit on the fly, but it will
        // not be nearly as nice as what the offline terrafit utility
        // would have produced.
        SG_LOG(SG_GENERAL, SG_DEBUG, "  Cannot open " << fitted_name );
        delete fitted_in;
        fitted_in = NULL;
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  Opening fitted data file: " << fitted_name );
    }

    return (array_in != NULL) ? true : false;
}


// close an Array file
bool
TGArray::close() {
    if (array_in) {
        gzclose(array_in);
        array_in = NULL;
    }

    if (fitted_in ) {
        fitted_in->close();
        delete fitted_in;
        fitted_in = NULL;
    }

    return true;
}


// parse Array file, pass in the bucket so we can make up values when
// the file wasn't found.
bool
TGArray::parse( SGBucket& b ) {
    // Parse/load the array data file
    if ( array_in ) {
        parse_bin();
    } else {
        // file not open (not found?), fill with zero'd data

        originx = ( b.get_center_lon() - 0.5 * b.get_width() ) * 3600.0;
        originy = ( b.get_center_lat() - 0.5 * b.get_height() ) * 3600.0;

        double max_x = ( b.get_center_lon() + 0.5 * b.get_width() ) * 3600.0;
        double max_y = ( b.get_center_lat() + 0.5 * b.get_height() ) * 3600.0;

        cols = 3;
        col_step = (max_x - originx) / (cols - 1);
        rows = 3;
        row_step = (max_y - originy) / (rows - 1);

        SG_LOG(SG_GENERAL, SG_DEBUG, "    origin  = " << originx << "  " << originy );
        SG_LOG(SG_GENERAL, SG_DEBUG, "    cols = " << cols << "  rows = " << rows );
        SG_LOG(SG_GENERAL, SG_DEBUG, "    col_step = " << col_step << "  row_step = " << row_step );


        in_data = new short[cols * rows];
        memset(in_data, 0, sizeof(short) * cols * rows);
        SG_LOG(SG_GENERAL, SG_DEBUG, "    File not open, so using zero'd data" );
    }

    // Parse/load the fitted data file
    if ( fitted_in && fitted_in->is_open() ) {
        int fitted_size;
        double x, y, z;
        *fitted_in >> fitted_size;
        for ( int i = 0; i < fitted_size; ++i ) {
            *fitted_in >> x >> y >> z;
            fitted_list.push_back( Point3D(x, y, z) );
            SG_LOG(SG_GENERAL, SG_DEBUG, " loading fitted = " << Point3D(x, y, z) );
        }
    }

    return true;
}

void TGArray::parse_bin()
{
    int32_t header;
    sgReadLong(array_in, &header);
    if (header != 0x54474152) {
        SG_LOG(SG_GENERAL, SG_ALERT, "\nThe .arr file is not in the correct binary format."
        << "\nPlease rebuild it using the latest TerraGear HGT tools.");
        exit(1);
    }

    int minX, minY, intColStep, intRowStep;
    sgReadInt(array_in, &minX);
    sgReadInt(array_in, &minY);
    originx = minX;
    originy = minY;

    sgReadInt(array_in, &cols);
    sgReadInt(array_in, &intColStep);
    sgReadInt(array_in, &rows);
    sgReadInt(array_in, &intRowStep);

    col_step = intColStep;
    row_step = intRowStep;

    in_data = new short[cols * rows];
    sgReadShort(array_in, cols * rows, in_data);
}

// write an Array file
bool TGArray::write( const string root_dir, SGBucket& b ) {
    // generate output file name
    string base = b.gen_base_path();
    string path = root_dir + "/" + base;
    SGPath sgp( path );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    string array_file = path + "/" + b.gen_index_str() + ".arr.new.gz";
    SG_LOG(SG_GENERAL, SG_DEBUG, "array_file = " << array_file );

    // write the file
    gzFile fp;
    if ( (fp = gzopen( array_file.c_str(), "wb9" )) == NULL ) {
	SG_LOG(SG_GENERAL, SG_ALERT, "ERROR:  cannot open " << array_file << " for writing!" );
	return false;
    }

    SG_LOG(SG_GENERAL, SG_DEBUG, "origin = " << originx << ", " << originy );
    gzprintf( fp, "%d %d\n", (int)originx, (int)originy );
    gzprintf( fp, "%d %d %d %d\n", cols, (int)col_step, rows, (int)row_step );
    for ( int i = 0; i < cols; ++i ) {
	for ( int j = 0; j < rows; ++j ) {
	    gzprintf( fp, "%d ", get_array_elev(i, j) );
	}
	gzprintf( fp, "\n" );
    }
    gzclose(fp);

    return true;
}


// do our best to remove voids by picking data from the nearest neighbor.
void TGArray::remove_voids( ) {
    // need two passes to ensure that all voids are removed (unless entire
    // array is a void.)
    bool have_void = true;
    int last_elev = -32768;
    for ( int pass = 0; pass < 2 && have_void; ++pass ) {
        // attempt to fill in any void data horizontally
        for ( int i = 0; i < cols; i++ ) {
            int j;

            // fill in front ways
            last_elev = -32768;
            have_void = false;
            for ( j = 0; j < rows; j++ ) {
                if ( get_array_elev(i,j) > -9000 ) {
                    last_elev = get_array_elev(i,j) ;
                } else if ( last_elev > -9000 ) {
                    set_array_elev(i, j, last_elev);
                } else {
                    have_void = true;
                }
            }
            // fill in back ways
            last_elev = -32768;
            have_void = false;
            for ( j = rows - 1; j >= 0; j-- ) {
                if ( get_array_elev(i,j) > -9000 ) {
                    last_elev = get_array_elev(i,j);
                } else if ( last_elev > -9000 ) {
                    set_array_elev(i, j, last_elev);
                } else {
                    have_void = true;
                }
            }
        }

        // attempt to fill in any void data vertically
        for ( int j = 0; j < rows; j++ ) {
            int i;

            // fill in front ways
            last_elev = -32768;
            have_void = false;
            for ( i = 0; i < cols; i++ ) {
                if ( get_array_elev(i,j) > -9999 ) {
                    last_elev = get_array_elev(i,j);
                } else if ( last_elev > -9999 ) {
                    set_array_elev(i, j, last_elev);
                } else {
                    have_void = true;
                }
            }

            // fill in back ways
            last_elev = -32768;
            have_void = false;
            for ( i = cols - 1; i >= 0; i-- ) {
                if ( get_array_elev(i,j) > -9999 ) {
                    last_elev = get_array_elev(i,j);
                } else if ( last_elev > -9999 ) {
                    set_array_elev(i, j, last_elev);
                } else {
                    have_void = true;
                }
            }
        }
    }

    if ( have_void ) {
        // after all that work we still have a void, likely the
        // entire array is void.  Fill in the void areas with zero
        // as a panic fall back.
        for ( int i = 0; i < cols; i++ ) {
            for ( int j = 0; j < rows; j++ ) {
                if ( get_array_elev(i,j) <= -9999 ) {
                    set_array_elev(i, j, 0);
                }
            }
        }
    }
}


// Return the elevation of the closest non-void grid point to lon, lat
double TGArray::closest_nonvoid_elev( double lon, double lat ) const {
    double mindist = 99999999999.9;
    double minelev = -9999.0;
    Point3D p0( lon, lat, 0.0 );

    for ( int row = 0; row < rows; row++ ) {
        for ( int col = 0; col < cols; col++ ) {
            Point3D p1(originx + col * col_step, originy + row * row_step, 0.0);
            double dist = p0.distance3D( p1 );
            double elev = get_array_elev(col, row);
            if ( dist < mindist && elev > -9000 ) {
                mindist = dist;
                minelev = elev;
                // cout << "dist = " << mindist;
                // cout << "  elev = " << elev << endl;
            }
        }
    }

    if ( minelev > -9999.0 ) {
        return minelev;
    } else {
        return 0.0;
    }
}


// return the current altitude based on grid data.
// TODO: We should rewrite this to interpolate exact values, but for now this is good enough
double TGArray::altitude_from_grid( double lon, double lat ) const {
    // we expect incoming (lon,lat) to be in arcsec for now

    double xlocal, ylocal, dx, dy, zA, zB, elev;
    int x1, x2, x3, y1, y2, y3;
    float z1, z2, z3;
    int xindex, yindex;

    /* determine if we are in the lower triangle or the upper triangle
       ______
       |   /|
       |  / |
       | /  |
       |/   |
       ------

       then calculate our end points
     */

    xlocal = (lon - originx) / col_step;
    ylocal = (lat - originy) / row_step;

    xindex = (int)(xlocal);
    yindex = (int)(ylocal);

    // printf("xindex = %d  yindex = %d\n", xindex, yindex);

    if ( xindex + 1 == cols ) {
	xindex--;
    }

    if ( yindex + 1 == rows ) {
	yindex--;
    }

    if ( (xindex < 0) || (xindex + 1 >= cols) ||
	 (yindex < 0) || (yindex + 1 >= rows) ) {
	SG_LOG(SG_GENERAL, SG_DEBUG, "WARNING: Attempt to interpolate value outside of array!!!" );
	return -9999;
    }

    dx = xlocal - xindex;
    dy = ylocal - yindex;

    if ( dx > dy ) {
	// lower triangle

	x1 = xindex;
	y1 = yindex;
	z1 = get_array_elev(x1, y1);

	x2 = xindex + 1;
	y2 = yindex;
	z2 = get_array_elev(x2, y2);

	x3 = xindex + 1;
	y3 = yindex + 1;
	z3 = get_array_elev(x3, y3);

        if ( z1 < -9000 || z2 < -9000 || z3 < -9000 ) {
            // don't interpolate off a void
            return closest_nonvoid_elev( lon, lat );
        }

	zA = dx * (z2 - z1) + z1;
	zB = dx * (z3 - z1) + z1;

	if ( dx > SG_EPSILON ) {
	    elev = dy * (zB - zA) / dx + zA;
	} else {
	    elev = zA;
	}
    } else {
	// upper triangle

	x1 = xindex;
	y1 = yindex;
	z1 = get_array_elev(x1, y1);

	x2 = xindex;
	y2 = yindex + 1;
	z2 = get_array_elev(x2, y2);

	x3 = xindex + 1;
	y3 = yindex + 1;
	z3 = get_array_elev(x3, y3);

        if ( z1 < -9000 || z2 < -9000 || z3 < -9000 ) {
            // don't interpolate off a void
            return closest_nonvoid_elev( lon, lat );
        }

	zA = dy * (z2 - z1) + z1;
	zB = dy * (z3 - z1) + z1;

	if ( dy > SG_EPSILON ) {
	    elev = dx * (zB - zA) / dy    + zA;
	} else {
	    elev = zA;
	}
    }

    return elev;
}


TGArray::~TGArray( void )
{
    delete[] in_data;
}

int TGArray::get_array_elev( int col, int row ) const
{
    return in_data[(col * rows) + row];
}

void TGArray::set_array_elev( int col, int row, int val )
{
    in_data[(col * rows) + row] = val;
}

bool TGArray::is_open() const
{
  if ( array_in != NULL ) {
      return true;
  } else {
      return false;
  }
}
