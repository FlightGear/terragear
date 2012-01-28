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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: array.cxx,v 1.24 2005-11-10 16:26:59 curt Exp $


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <iostream>
#include <string>

#include <simgear/constants.h>
#include <simgear/misc/sgstream.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/strutils.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/math/leastsqs.hxx>

#include "array.hxx"

using std::string;


TGArray::TGArray( void ):
  array_in(NULL),
  fitted_in(NULL)
{
    SG_LOG(SG_GENERAL, SG_DEBUG, "class TGArray CONstructor called." );
    in_data = new int*[ARRAY_SIZE_1];
    for (int i = 0; i < ARRAY_SIZE_1; i++) {
        in_data[i] = new int[ARRAY_SIZE_1]; 
    }
}


TGArray::TGArray( const string &file ):
  array_in(NULL),
  fitted_in(NULL)
{
    SG_LOG(SG_GENERAL, SG_DEBUG, "class TGArray CONstructor called." );
    in_data = new int* [ARRAY_SIZE_1];
    for (int i = 0; i < ARRAY_SIZE_1; i++)
        in_data[i] = new int[ARRAY_SIZE_1];

    SG_LOG(SG_GENERAL, SG_ALERT, "ps TGArray CONstructor called." );

    TGArray::open(file);
}


// open an Array file (and fitted file if it exists)
bool TGArray::open( const string& file_base ) {
    // open array data file
    string array_name = file_base + ".arr.gz";
    array_in = new sg_gzifstream( array_name );
    if ( !array_in->is_open() ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  ps: Cannot open " << array_name );
        delete array_in;
        array_in = NULL;
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  Opening array data file: " << array_name );
    }

    // open fitted data file
    string fitted_name = file_base + ".fit.gz";
    fitted_in = new sg_gzifstream( fitted_name );
    if ( ! fitted_in->is_open() ) {
        // not having a .fit file is unfortunate, but not fatal.  We
        // can do a really stupid/crude fit on the fly, but it will
        // not be nearly as nice as what the offline terrafit utility
        // would have produced.
        SG_LOG(SG_GENERAL, SG_DEBUG, "  ps: Cannot open " << fitted_name );
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
        array_in->close();
        delete array_in;
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
    if ( array_in && array_in->is_open() ) {
        // file open, parse
        *array_in >> originx >> originy;
        *array_in >> cols >> col_step;
        *array_in >> rows >> row_step;

        SG_LOG(SG_GENERAL, SG_DEBUG, "    origin  = " << originx << "  " << originy );
        SG_LOG(SG_GENERAL, SG_DEBUG, "    cols = " << cols << "  rows = " << rows );
        SG_LOG(SG_GENERAL, SG_DEBUG, "    col_step = " << col_step << "  row_step = " << row_step );

        for ( int i = 0; i < cols; i++ ) {
            for ( int j = 0; j < rows; j++ ) {
        	*array_in >> in_data[i][j];
            }
        }

        SG_LOG(SG_GENERAL, SG_DEBUG, "    Done parsing" );
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

        for ( int i = 0; i < cols; i++ ) {
            for ( int j = 0; j < rows; j++ ) {
                in_data[i][j] = 0;
            }
        }

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
	    gzprintf( fp, "%d ", (int)in_data[i][j] );
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
                if ( in_data[i][j] > -9000 ) {
                    last_elev = in_data[i][j];
                } else if ( last_elev > -9000 ) {
                    in_data[i][j] = last_elev;
                } else {
                    have_void = true;
                }
            }
            // fill in back ways
            last_elev = -32768;
            have_void = false;
            for ( j = rows - 1; j >= 0; j-- ) {
                if ( in_data[i][j] > -9000 ) {
                    last_elev = in_data[i][j];
                } else if ( last_elev > -9000 ) {
                    in_data[i][j] = last_elev;
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
                if ( in_data[i][j] > -9999 ) {
                    last_elev = in_data[i][j];
                } else if ( last_elev > -9999 ) {
                    in_data[i][j] = last_elev;
                } else {
                    have_void = true;
                }
            }

            // fill in back ways
            last_elev = -32768;
            have_void = false;
            for ( i = cols - 1; i >= 0; i-- ) {
                if ( in_data[i][j] > -9999 ) {
                    last_elev = in_data[i][j];
                } else if ( last_elev > -9999 ) {
                    in_data[i][j] = last_elev;
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
                if ( in_data[i][j] <= -9999 ) {
                    in_data[i][j] = 0;
                }
            }
        }
    }
}


// add a node to the output corner node list
void TGArray::add_corner_node( int i, int j, double val ) {
    
    double x = (originx + i * col_step) / 3600.0;
    double y = (originy + j * row_step) / 3600.0;
    SG_LOG(SG_GENERAL, SG_DEBUG, "originx = " << originx << "  originy = " << originy );
    SG_LOG(SG_GENERAL, SG_DEBUG, "corner = " << Point3D(x, y, val) );
    corner_list.push_back( Point3D(x, y, val) );
}


// add a node to the output fitted node list
void TGArray::add_fit_node( int i, int j, double val ) {
    double x = (originx + i * col_step) / 3600.0;
    double y = (originy + j * row_step) / 3600.0;
    SG_LOG(SG_GENERAL, SG_DEBUG, Point3D(x, y, val) );
    fitted_list.push_back( Point3D(x, y, val) );
}


#if 0
// Use least squares to fit a simpler data set to dem data.  Return
// the number of fitted nodes.  This is a horrible approach that
// doesn't really work, but it's better than nothing if you've got
// nothing.  Using src/Prep/ArrayFit to create .fit files from the
// .arr files is a *much* better approach, but it is slower which is
// why it needs to be done "offline".
int TGArray::fit( double error ) {
    if ( ! fit_on_the_fly ) {
        return fitted_list.size();
    }
    double x[ARRAY_SIZE_1], y[ARRAY_SIZE_1];
    double m, b, max_error, error_sq;
    double x1, y1;
    // double ave_error;
    double cury, lasty;
    int n, row, start, end;
    int colmin, colmax, rowmin, rowmax;
    bool good_fit;
    // FILE *dem, *fit, *fit1;

    error_sq = error * error;

    cout << "  Initializing fitted node list" << endl;
    corner_list.clear();
    fitted_list.clear();

    // determine dimensions
    colmin = 0;
    colmax = cols;
    rowmin = 0;
    rowmax = rows;
    cout << "  Fitting region = " << colmin << "," << rowmin << " to " 
	 << colmax << "," << rowmax << endl;;
    
    // generate corners list
    add_corner_node( colmin, rowmin, in_data[colmin][rowmin] );
    add_corner_node( colmin, rowmax-1, in_data[colmin][rowmax] );
    add_corner_node( colmax-1, rowmin, in_data[colmax][rowmin] );
    add_corner_node( colmax-1, rowmax-1, in_data[colmax][rowmax] );

    cout << "  Beginning best fit procedure" << endl;
    lasty = 0;

    for ( row = rowmin; row < rowmax; row++ ) {
	// fit  = fopen("fit.dat",  "w");
	// fit1 = fopen("fit1.dat", "w");

	start = colmin;

	// cout << "    fitting row = " << row << endl;

	while ( start < colmax - 1 ) {
	    end = start + 1;
	    good_fit = true;

	    x[0] = start * col_step;
	    y[0] = in_data[start][row];

	    x[1] = end * col_step;
	    y[1] = in_data[end][row];

	    n = 2;

	    // cout << "Least square of first 2 points" << endl;
	    least_squares(x, y, n, &m, &b);

	    end++;

	    while ( (end < colmax) && good_fit ) {
		++n;
		// cout << "Least square of first " << n << " points" << endl;
		x[n-1] = x1 = end * col_step;
		y[n-1] = y1 = in_data[end][row];
		least_squares_update(x1, y1, &m, &b);
		// ave_error = least_squares_error(x, y, n, m, b);
		max_error = least_squares_max_error(x, y, n, m, b);

		/*
		printf("%d - %d  ave error = %.2f  max error = %.2f  y = %.2f*x + %.2f\n", 
		start, end, ave_error, max_error, m, b);
		
		f = fopen("gnuplot.dat", "w");
		for ( j = 0; j <= end; j++) {
		    fprintf(f, "%.2f %.2f\n", 0.0 + ( j * col_step ), 
			    in_data[row][j]);
		}
		for ( j = start; j <= end; j++) {
		    fprintf(f, "%.2f %.2f\n", 0.0 + ( j * col_step ), 
			    in_data[row][j]);
		}
		fclose(f);

		printf("Please hit return: "); gets(junk);
		*/

		if ( max_error > error_sq ) {
		    good_fit = false;
		}
		
		end++;
	    }

	    if ( !good_fit ) {
		// error exceeded the threshold, back up
		end -= 2;  // back "end" up to the last good enough fit
		n--;       // back "n" up appropriately too
	    } else {
		// we popped out of the above loop while still within
		// the error threshold, so we must be at the end of
		// the data set
		end--;
	    }
	    
	    least_squares(x, y, n, &m, &b);
	    // ave_error = least_squares_error(x, y, n, m, b);
	    max_error = least_squares_max_error(x, y, n, m, b);

	    /*
	    printf("\n");
	    printf("%d - %d  ave error = %.2f  max error = %.2f  y = %.2f*x + %.2f\n", 
		   start, end, ave_error, max_error, m, b);
	    printf("\n");

	    fprintf(fit1, "%.2f %.2f\n", x[0], m * x[0] + b);
	    fprintf(fit1, "%.2f %.2f\n", x[end-start], m * x[end-start] + b);
	    */

	    if ( start > colmin ) {
		// skip this for the first line segment
		cury = m * x[0] + b;
		add_fit_node( start, row, (lasty + cury) / 2 );
		// fprintf(fit, "%.2f %.2f\n", x[0], (lasty + cury) / 2);
	    }

	    lasty = m * x[end-start] + b;
	    start = end;
	}

	/*
	fclose(fit);
	fclose(fit1);

	dem = fopen("gnuplot.dat", "w");
	for ( j = 0; j < ARRAY_SIZE_1; j++) {
	    fprintf(dem, "%.2f %.2f\n", 0.0 + ( j * col_step ), 
		    in_data[j][row]);
	} 
	fclose(dem);
	*/

	// NOTICE, this is for testing only.  This instance of
        // output_nodes should be removed.  It should be called only
        // once at the end once all the nodes have been generated.
	// newmesh_output_nodes(&nm, "mesh.node");
	// printf("Please hit return: "); gets(junk);
    }

    // outputmesh_output_nodes(fg_root, p);

    // return fit nodes + 4 corners
    return fitted_list.size() + 4;
}
#endif


// Return the elevation of the closest non-void grid point to lon, lat
double TGArray::closest_nonvoid_elev( double lon, double lat ) const {
    double mindist = 99999999999.9;
    double minelev = -9999.0;
    Point3D p0( lon, lat, 0.0 );

    for ( int row = 0; row < rows; row++ ) {
        for ( int col = 0; col < cols; col++ ) {
            Point3D p1(originx + col * col_step, originy + row * row_step, 0.0);
            double dist = p0.distance3D( p1 );
            double elev = in_data[col][row];
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


// return the current altitude based on grid data.  We should rewrite
// this to interpolate exact values, but for now this is good enough
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
	z1 = in_data[x1][y1];

	x2 = xindex + 1; 
	y2 = yindex; 
	z2 = in_data[x2][y2];
				  
	x3 = xindex + 1; 
	y3 = yindex + 1; 
	z3 = in_data[x3][y3];

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
	z1 = in_data[x1][y1];

	x2 = xindex; 
	y2 = yindex + 1; 
	z2 = in_data[x2][y2];
				  
	x3 = xindex + 1; 
	y3 = yindex + 1; 
	z3 = in_data[x3][y3];
 
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


#if 0
// Write out a node file that can be used by the "triangle" program.
// Check for an optional "index.node.ex" file in case there is a .poly
// file to go along with this node file.  Include these nodes first
// since they are referenced by position from the .poly file.
void TGArray::outputmesh_output_nodes( const string& fg_root, SGBucket& p )
{
    double exnodes[MAX_EX_NODES][3];
    struct stat stat_buf;
    string dir, file;
    char exfile[256];
#ifdef WIN32
    char tmp_path[256];
#endif
    string command;
    FILE *fd;
    int colmin, colmax, rowmin, rowmax;
    int i, j, count, excount, result;

    // determine dimensions
    colmin = p.get_x() * ( (cols - 1) / 8);
    colmax = colmin + ( (cols - 1) / 8);
    rowmin = p.get_y() * ( (rows - 1) / 8);
    rowmax = rowmin + ( (rows - 1) / 8);
    cout << "  dumping region = " << colmin << "," << rowmin << " to " <<
	colmax << "," << rowmax << "\n";

    // generate the base directory
    string base_path = p.gen_base_path();
    cout << "  fg_root = " << fg_root << "  Base Path = " << base_path << endl;
    dir = fg_root + "/" + base_path;
    cout << "  Dir = " << dir << endl;
    
    // stat() directory and create if needed
    errno = 0;
    result = stat(dir.c_str(), &stat_buf);
    if ( result != 0 && errno == ENOENT ) {
	cout << "  Creating directory\n";

#ifdef _MSC_VER
	fg_mkdir( dir.cstr() );
#else
	command = "mkdir -p " + dir + "\n";
	system( command.c_str() );
#endif
    } else {
	// assume directory exists
    }

    // get index and generate output file name
    file = dir + "/" + p.gen_index_str() + ".node";

    // get (optional) extra node file name (in case there is matching
    // .poly file.
    exfile = file + ".ex";

    // load extra nodes if they exist
    excount = 0;
    if ( (fd = fopen(exfile, "r")) != NULL ) {
	int junki;
	fscanf(fd, "%d %d %d %d", &excount, &junki, &junki, &junki);

	if ( excount > MAX_EX_NODES - 1 ) {
	    printf("Error, too many 'extra' nodes, increase array size\n");
	    exit(-1);
	} else {
	    printf("    Expecting %d 'extra' nodes\n", excount);
	}

	for ( i = 1; i <= excount; i++ ) {
	    fscanf(fd, "%d %lf %lf %lf\n", &junki, 
		   &exnodes[i][0], &exnodes[i][1], &exnodes[i][2]);
	    printf("(extra) %d %.2f %.2f %.2f\n", 
		    i, exnodes[i][0], exnodes[i][1], exnodes[i][2]);
	}
	fclose(fd);
    }

    printf("Creating node file:  %s\n", file);
    fd = fopen(file, "w");

    // first count regular nodes to generate header
    count = 0;
    for ( j = rowmin; j <= rowmax; j++ ) {
	for ( i = colmin; i <= colmax; i++ ) {
	    if ( out_data[i][j] > -9000.0 ) {
		count++;
	    }
	}
	// printf("    count = %d\n", count);
    }
    fprintf(fd, "%d 2 1 0\n", count + excount);

    // now write out extra node data
    for ( i = 1; i <= excount; i++ ) {
	fprintf(fd, "%d %.2f %.2f %.2f\n", 
		i, exnodes[i][0], exnodes[i][1], exnodes[i][2]);
    }

    // write out actual node data
    count = excount + 1;
    for ( j = rowmin; j <= rowmax; j++ ) {
	for ( i = colmin; i <= colmax; i++ ) {
	    if ( out_data[i][j] > -9000.0 ) {
		fprintf(fd, "%d %.2f %.2f %.2f\n", 
			count++, 
			originx + (double)i * col_step, 
			originy + (double)j * row_step,
			out_data[i][j]);
	    }
	}
	// printf("    count = %d\n", count);
    }

    fclose(fd);
}
#endif


TGArray::~TGArray( void ) {
    for (int i = 0; i < ARRAY_SIZE_1; i++)
        delete [] in_data[i];
    delete [] in_data;
}


