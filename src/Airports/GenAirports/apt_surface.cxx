// apt_surface.cxx -- class to manage airport terrain surface
//                    approximation and smoothing
//
// Written by Curtis Olson, started March 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - curt@flightgear.org
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
//


#include <nurbs++/nurbsS.h>
#include <nurbs++/nurbsSub.h>

#include <simgear/constants.h>
#include <simgear/math/sg_types.hxx>

#include <Array/array.hxx>

#include "apt_surface.hxx"

SG_USING_NAMESPACE( PLib );


// fix node elevations.  Offset is added to the final elevation
static void calc_elevations( const string &root, const string_list elev_src,
                             Matrix_Point3Df &Pts ) {
    // string_list elev_src;
    // elev_src.clear();
    // elev_src.push_back( "SRTM-1" );
    // elev_src.push_back( "SRTM-3" );
    // elev_src.push_back( "DEM-3" );
    // elev_src.push_back( "DEM-30" );

    bool done = false;
    int i, j;
    TGArray array;

    // just bail if no work to do
    if ( Pts.rows() == 0 || Pts.cols() == 0 ) {
        return;
    }

    // set all elevations to -9999
    for ( i = 0; i < Pts.rows(); ++i ) {
        for ( j = 0; j < Pts.cols(); ++j ) {
            Point3Df p = Pts(i,j);
            p.z() = -9999.0;
        }
    }

    while ( !done ) {
	// find first node with -9999 elevation
        Point3Df first;
        bool found_one = false;
        for ( i = 0; i < Pts.rows(); ++i ) {
            for ( j = 0; j < Pts.cols(); ++j ) {
                Point3Df p = Pts(i,j);
                if ( p.z() < -9000.0 && !found_one ) {
                    first = p;
                    found_one = true;
                }
            }
        }

	if ( found_one ) {
	    SGBucket b( first.x(), first.y() );
	    string base = b.gen_base_path();

	    // try the various elevation sources
            j = 0;
            bool found_file = false;
            while ( !found_file && j < (int)elev_src.size() ) {
                string array_path = root + "/" + elev_src[j] + "/" + base 
                    + "/" + b.gen_index_str();
                if ( array.open(array_path) ) {
                    found_file = true;
                    cout << "Using array_path = " << array_path << endl;
                }                    
                j++;
            }
            
            // this will fill in a zero structure if no array data
            // found/opened
	    array.parse( b );

            // this will do a hasty job of removing voids by inserting
            // data from the nearest neighbor (sort of)
            array.remove_voids();

	    // update all the non-updated elevations that are inside
	    // this array file
	    double elev;
	    done = true;
            for ( i = 0; i < Pts.rows(); ++i ) {
                for ( j = 0; j < Pts.cols(); ++j ) {
                    Point3Df p = Pts(i,j);
                    if ( p.z() < -9000.0 ) {
                        done = false;
                        elev = array.altitude_from_grid( p.x() * 3600.0,
                                                         p.y() * 3600.0 );
                        if ( elev > -9000 ) {
                            p.z() = elev;
                            Pts(i,j) = p;
                            // cout << "interpolating for " << p << endl;
                        }
                    }
                }
            }

	    array.close();
	} else {
	    done = true;
	}
    }

    // do some post processing for sanity's sake

    // find the average height of the queried points
    double total = 0.0;
    int count = 0;
    for ( i = 0; i < Pts.rows(); ++i ) {
        for ( j = 0; j < Pts.cols(); ++j ) {
            Point3Df p = Pts(i,j);
            total += p.z();
            count++;
        }
    }
    double average = total / (double) count;
    cout << "Average surface height = " << average << endl;

    // now go through the elevations and clamp them all to within
    // +/-10m (33') of the average.
    const double dz = 10.0;
    for ( i = 0; i < Pts.rows(); ++i ) {
        for ( j = 0; j < Pts.cols(); ++j ) {
            Point3Df p = Pts(i,j);
            if ( p.z() < average - dz ) {
                p.z() = average - dz;
                Pts(i,j) = p;
            }
            if ( p.z() > average + dz ) {
                p.z() = average + dz;
                Pts(i,j) = p;
            }
        }
    }
}


// Constructor, specify min and max coordinates of desired area in
// lon/lat degrees
TGAptSurface::TGAptSurface( const string& path,
                            const string_list& elev_src,
                            Point3D _min_deg, Point3D _max_deg )
{
    // Calculate desired size of grid
    min_deg = _min_deg;
    max_deg = _max_deg;

    // The following size calculations are for the purpose of
    // determining grid divisions so it's not important that they be
    // *exact*, just ball park.
    double y_deg = max_deg.lat() - min_deg.lat();
    double y_rad = y_deg * SG_DEGREES_TO_RADIANS;
    double y_nm = y_rad * SG_RAD_TO_NM;
    double y_m = y_nm * SG_NM_TO_METER;

    double xfact = cos( min_deg.lat() * SG_DEGREES_TO_RADIANS );
    double x_deg = max_deg.lon() - min_deg.lon();
    double x_rad = x_deg * SG_DEGREES_TO_RADIANS;
    double x_nm = x_rad * SG_RAD_TO_NM * xfact;
    double x_m = x_nm * SG_NM_TO_METER;

    cout << "Area size = " << x_m << " x " << y_m << " (m)" << endl;

    int xdivs = (int)(x_m / 600.0) + 1;
    int ydivs = (int)(y_m / 600.0) + 1;

    if ( xdivs < 3 ) { xdivs = 3; }
    if ( ydivs < 3 ) { ydivs = 3; }

    cout << "  M(" << xdivs << "," << ydivs << ")" << endl;
    double dlon = x_deg / xdivs;
    double dlat = y_deg / ydivs;

    // Build the extra res input grid
    int mult = 10;
    Matrix_Point3Df dPts((xdivs+2) * mult + 1, (ydivs+2) * mult + 1);
    for ( int i = 0; i < dPts.rows(); ++i ) {
        for ( int j = 0; j < dPts.cols(); ++j ) {
            dPts(i,j) = Point3Df( min_deg.lon() + (i-mult)*(dlon/(double)mult),
                                  min_deg.lat() + (j-mult)*(dlat/(double)mult),
                                  -9999 );
        }
    }

    // Determine elevation of the grid points
    calc_elevations( path, elev_src, dPts );

    // Build the normal res input grid from the double res version
    Matrix_Point3Df Pts(xdivs + 1, ydivs + 1);
    for ( int i = 0; i < xdivs + 1; ++i ) {
        for ( int j = 0; j < ydivs + 1; ++j ) {
            cout << i << "," << j << endl;
            double accum = 0.0;
            for ( int ii = 0; ii < mult; ++ii ) {
                for ( int jj = 0; jj < mult; ++jj ) {
                    accum += dPts(mult*(i+1) - (mult/2) + ii,
                                  mult*(j+1) - (mult/2) + jj).z();
                }
            }
            Pts(i,j) = Point3Df( min_deg.lon() + i * dlon,
                                 min_deg.lat() + j * dlat,
                                 accum / (mult*mult) );
        }
    }

    // Create the nurbs surface

    cout << "ready to create nurbs surface" << endl;
    apt_surf = new PlNurbsSurfacef;
    apt_surf->globalInterp( Pts, 3, 3);
    cout << "  successful." << endl;
}


TGAptSurface::~TGAptSurface() {
    delete apt_surf;
}


// Query the elevation of a point, return -9999 if out of range
double TGAptSurface::query( double lon_deg, double lat_deg ) {
    // sanity check
    if ( lon_deg < min_deg.lon() || lon_deg > max_deg.lon() ||
         lat_deg < min_deg.lat() || lat_deg > max_deg.lat() )
    {
        cout << "Warning: query out of bounds for NURBS surface!" << endl;
        return -9999.0;
    }

    // convert lon,lat to nurbs space
    double x = (lon_deg - min_deg.lon()) / (max_deg.lon() - min_deg.lon());
    double y = (lat_deg - min_deg.lat()) / (max_deg.lat() - min_deg.lat());

    cout << "  querying for " << x << ", " << y << " = ";
    Point3Df p = apt_surf->pointAt( x, y );
    cout << p.z() << endl;

    return p.z();
}
