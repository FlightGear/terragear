// dem.cxx -- DEM management class
//
// Written by Curtis Olson, started March 1998.
//
// Copyright (C) 1998  Curtis L. Olson  - http://www.flightgear.org/~curt
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


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <iostream>

#include <simgear/misc/sg_path.hxx>
#include <simgear/misc/strutils.hxx>
#include <simgear/debug/logstream.hxx>

#include "dem.hxx"

using std::cout;
using std::endl;
using std::string;


TGDem::TGDem() :
    in(nullptr),
    dem_data(new float[DEM_SIZE_1][DEM_SIZE_1]),
    output_data(new float[DEM_SIZE_1][DEM_SIZE_1]),
    dem_description(""),
    dem_quadrangle(""),
    option_name("")
{
    originx = originy = 0.0;
    cols =rows = 0;
    col_step = row_step = 0.0;
    dem_x1 = dem_x2 = dem_x3 = dem_x4 = 0.0;
    dem_y1 = dem_y2 = dem_y3 = dem_y4 = 0.0;
    dem_z1 = dem_z2 = 0.0;
    dem_resolution = dem_num_profiles = 0;
    prof_col = prof_row = 0;
    prof_num_cols = prof_num_rows = 0;
    prof_x1 = prof_y1 = 0.0;
    prof_data = 0.0;
    do_data = 0;
    cur_col = cur_row = 0;
    z_units = 2;    // meters

    for (int x = 0; x < DEM_SIZE_1; ++x) {
        for (int y = 0; y < DEM_SIZE_1; ++y) {
            dem_data[x][y] = 0.0;
            output_data[x][y] = 0.0;
        }
    }
}


TGDem::TGDem( const string &file ) :
    TGDem::TGDem()
{
    TGDem::open(file);
}


TGDem::~TGDem() {
    // printf("class TGDem DEstructor called.\n");
    delete [] dem_data;
    delete [] output_data;
}


// open a DEM file
bool
TGDem::open ( const string& file ) {
    // open input file (or read from stdin)
    if ( file ==  "-" ) {
        SG_LOG(SG_GENERAL, SG_INFO, "Loading DEM data file: stdin");
	// fd = stdin;
	// fd = gzdopen(STDIN_FILENO, "r");
        SG_LOG(SG_GENERAL, SG_INFO, "Not yet ported ...");
	return false;
    } else {
	in = new sg_gzifstream( file );
	if ( !(*in) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Cannot open " << file );
	    return false;
	}
	SG_LOG(SG_GENERAL, SG_INFO, "Loading DEM data file: " << file );
    }

    return true;
}


// close a DEM file
bool
TGDem::close () {

    in->close();

    return true;
}


// return next token from input stream
string
TGDem::next_token() {
    string token;

    *in >> token;

    // cout << "    returning " + token + "\n";

    return token;
}


// return next integer from input stream
int
TGDem::next_int() {
    int result;

    *in >> result;

    return result;
}


// return next double from input stream
double
TGDem::next_double() {
    double result;

    *in >> result;

    return result;
}


// return next exponential num from input stream
double
TGDem::next_exp() {
    string token;

    token = next_token();

    const char* p = token.c_str();
    char buf[64];
    char* bp = buf;

    for ( ; *p != 0; ++p )
    {
	if ( *p == 'D' )
	    *bp++ = 'E';
	else
	    *bp++ = *p;
    }
    *bp = 0;
    return ::atof( buf );
}


// read and parse DEM "A" record
bool
TGDem::read_a_record() {
    int i, inum;
    string name, token, buf;
    char c;

    // get the name field (144 characters)
    for ( i = 0; i < 144; i++ ) {
	in->get(c);
	name += c;
    }

    // clean off the trailing whitespace
    name = simgear::strutils::rstrip(name);
    cout << "    Quad name field: " << name << endl;

    // DEM level code, 3 reflects processing by DMA
    inum = next_int();
    cout << "    DEM level code = " << inum << "\n";

    if ( inum > 3 ) {
	return false;
    }

    // Pattern code, 1 indicates a regular elevation pattern
    inum = next_int();
    cout << "    Pattern code = " << inum << "\n";

    // Planimetric reference system code, 0 indicates geographic
    // coordinate system.
    inum = next_int();
    cout << "    Planimetric reference code = " << inum << "\n";

    // Zone code
    inum = next_int();
    cout << "    Zone code = " << inum << "\n";

    // Map projection parameters (ignored)
    for ( i = 0; i < 15; i++ ) {
        double dnum = next_exp();
        SG_LOG(SG_GENERAL, SG_DEBUG, i << ": "  << dnum);
    }

    // Units code, 3 represents arc-seconds as the unit of measure for
    // ground planimetric coordinates throughout the file.
    inum = next_int();
    if ( inum != 3 ) {
	cout << "    Unknown (X,Y) units code = " << inum << "!\n";
	exit(-1);
    }

    // Units code; 2 represents meters as the unit of measure for
    // elevation coordinates throughout the file.
    z_units = inum = next_int();
    if ( z_units != 1 && z_units != 2 ) {
	cout << "    Unknown (Z) units code = " << inum << "!\n";
	exit(-1);
    }

    // Number (n) of sides in the polygon which defines the coverage of
    // the DEM file (usually equal to 4).
    inum = next_int();
    if ( inum != 4 ) {
	cout << "    Unknown polygon dimension = " << inum << "!\n";
	exit(-1);
    }

    // Ground coordinates of bounding box in arc-seconds
    dem_x1 = originx = next_exp();
    dem_y1 = originy = next_exp();
    cout << "    Origin = (" << originx << "," << originy << ")\n";

    dem_x2 = next_exp();
    dem_y2 = next_exp();

    dem_x3 = next_exp();
    dem_y3 = next_exp();

    dem_x4 = next_exp();
    dem_y4 = next_exp();

    // Minimum/maximum elevations in meters
    dem_z1 = next_exp();
    dem_z2 = next_exp();
    if ( z_units == 1 ) {
        // convert to meters
        dem_z1 *= SG_FEET_TO_METER;
        dem_z2 *= SG_FEET_TO_METER;
     }
    cout << "    Elevation range " << dem_z1 << " to " << dem_z2 << "\n";

    // Counterclockwise angle from the primary axis of ground
    // planimetric referenced to the primary axis of the DEM local
    // reference system.

    // eat and discard 24 characters (we don't use this value)
    for ( i = 0; i < 24; i++ ) {
	in->get(c);
    }

    // Accuracy code; 0 indicates that a record of accuracy does not
    // exist and that no record type C will follow.

    // DEM spacial resolution.  Usually (3,3,1) (3,6,1) or (3,9,1)
    // depending on latitude

    // I will eventually have to do something with this for data at
    // higher latitudes */

    // get the accuracy field (6 characters)
    name.erase();
    for ( i = 0; i < 6; i++ ) {
	in->get(c);
	name += c;
    }
    inum = atoi( name.c_str() );

    // get the spacial resolution (3*12 characters)
    name.erase();
    for ( i = 0; i < 12; i++ ) {
	in->get(c);
	name += c;
    }
    col_step = atof( name.c_str() );
    name.erase();
    for ( i = 0; i < 12; i++ ) {
	in->get(c);
	name += c;
    }
    row_step = atof( name.c_str() );
    name.erase();
    for ( i = 0; i < 12; i++ ) {
	in->get(c);
	name += c;
    }
    cout << "    Accuracy code = " << inum << "\n";
    cout << "    column step = " << col_step <<
	"  row step = " << row_step << "\n";

    // dimension of arrays to follow (1)
    token = next_token();

    // number of profiles
    dem_num_profiles = cols = next_int();
    cout << "    Expecting " << dem_num_profiles << " profiles\n";

    // eat characters to the end of the A record which we [hopefully]
    // know is guaranteed to be 160 characters away.
    for ( i = 0; i < 160; i++ ) {
	in->get(c);
    }

    return true;
}


// read and parse DEM "B" record
bool
TGDem::read_b_record( ) {
    string token;
    int i;

    // row / column id of this profile
    prof_row = next_int();
    prof_col = next_int();
    // printf("col id = %d  row id = %d\n", prof_col, prof_row);

    // Number of columns and rows (elevations) in this profile
    prof_num_rows = rows = next_int();
    prof_num_cols = next_int();
    // printf("    profile num rows = %d\n", prof_num_rows);

    // Ground planimetric coordinates (arc-seconds) of the first
    // elevation in the profile
    prof_x1 = next_exp();
    prof_y1 = next_exp();
    // printf("    Starting at %.2f %.2f\n", prof_x1, prof_y1);

    // Elevation of local datum for the profile.  Always zero for
    // 1-degree DEM, the reference is mean sea level.
    token = next_token();

    // Minimum and maximum elevations for the profile.
    token = next_token();
    token = next_token();

    // One (usually) dimensional array (1,prof_num_rows) of elevations
    float last = 0.0;
    for ( i = 0; i < prof_num_rows; i++ ) {
	prof_data = (float)next_int();

        if ( z_units == 1 ) {
            // convert to meters
            prof_data *= SG_FEET_TO_METER;
        }

	// a bit of sanity checking that is unfortunately necessary
	if ( prof_data > 10000 ) { // meters
	    prof_data = last;
	}

	dem_data[cur_col][i] = (float)prof_data;
	last = prof_data;
    }

    return true;
}


// parse dem file
bool
TGDem::parse( ) {
    int i;

    cur_col = 0;

    if ( !read_a_record() ) {
	return false;
    }

    for ( i = 0; i < dem_num_profiles; i++ ) {
	// printf("Ready to read next b record\n");
	read_b_record();
	cur_col++;

	if ( cur_col % 100 == 0 ) {
	    cout << "    loaded " << cur_col << " profiles of data\n";
	}
    }

    cout << "    Done parsing\n";

    return true;
}

// write out the area of data covered by the specified bucket.  Data
// is written out column by column starting at the lower left hand
// corner.
bool
TGDem::write_area( const string& root, SGBucket& b ) {
    // calculate some boundaries
    double min_x = ( b.get_center_lon() - 0.5 * b.get_width() ) * 3600.0;
    double max_x = ( b.get_center_lon() + 0.5 * b.get_width() ) * 3600.0;

    double min_y = ( b.get_center_lat() - 0.5 * b.get_height() ) * 3600.0;
    double max_y = ( b.get_center_lat() + 0.5 * b.get_height() ) * 3600.0;

    cout << b << endl;
    cout << "width = " << b.get_width() << " height = " << b.get_height()
         << endl;
    cout << "min = " << min_x << "," << min_y
         << "  max = " << max_x << "," << max_y << endl;
    int start_x = (int)((min_x - originx) / col_step);
    int span_x = (int)(b.get_width() * 3600.0 / col_step);

    int start_y = (int)((min_y - originy) / row_step);
    int span_y = (int)(b.get_height() * 3600.0 / row_step);

    cout << "start_x = " << start_x << "  span_x = " << span_x << endl;
    cout << "start_y = " << start_y << "  span_y = " << span_y << endl;

    // Do a simple sanity checking.  But, please, please be nice to
    // this write_area() routine and feed it buckets that coincide
    // well with the underlying grid structure and spacing.

    if ( ( min_x < originx )
         || ( max_x > originx + cols * col_step )
         || ( min_y < originy )
         || ( max_y > originy + rows * row_step ) ) {
        cout << "  ERROR: bucket at least partially outside DEM data range!" <<
            endl;
        return false;
    }

    // If the area is all ocean, skip it.
    if ( !has_non_zero_elev(start_x, span_x, start_y, span_y) ) {
        cout << "Tile is all zero elevation: skipping" << endl;
        return false;
    }

    // generate output file name
    string base = b.gen_base_path();
    string path = root + "/" + base;
    SGPath sgp( path );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    string array_file = path + "/" + b.gen_index_str() + ".arr.gz";
    cout << "array_file = " << array_file << endl;

    // write the file
    gzFile fp;
    if ( (fp = gzopen( array_file.c_str(), "wb9" )) == NULL ) {
        cout << "ERROR:  cannot open " << array_file << " for writing!" << endl;
        exit(-1);
    }

    gzprintf( fp, "%d %d\n", (int)min_x, (int)min_y );
    gzprintf( fp, "%d %f %d %f\n", span_x + 1, col_step,
              span_y + 1, row_step );
    for ( int i = start_x; i <= start_x + span_x; ++i ) {
        for ( int j = start_y; j <= start_y + span_y; ++j ) {
            gzprintf( fp, "%d ", (int)dem_data[i][j] );
        }
        gzprintf( fp, "\n" );
    }
    gzclose(fp);

    return true;
}


bool
TGDem::has_non_zero_elev (int start_x, int span_x,
                          int start_y, int span_y) const
{
    for (int i = start_x; i < start_x + span_x; i++) {
        for (int j = start_y; j < start_y + span_y; j++) {
            if (dem_data[i][j] != 0)
                return true;
        }
    }
    return false;
}

