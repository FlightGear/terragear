// elevations.cxx -- routines to help calculate DEM elevations for a
//                   set of points
//
// Written by Curtis Olson, started April 2004.
//
// Copyright (C) 2004  Curtis L. Olson  - curt@flightgear.org
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
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#include "global.hxx"
#include "apt_surface.hxx"

SG_USING_NAMESPACE( PLib );


// lookup node elevations for each point in the point_list.  Returns
// average of all points.  Doesn't modify the original list.

double tgAverageElevation( const string &root, const string_list elev_src,
                           const point_list points_source )
{
    bool done = false;
    unsigned int i;
    TGArray array;

    // make a copy so our routine is non-destructive.
    point_list points = points_source;

    // just bail if no work to do
    if ( points.size() == 0 ) {
        return 0.0;
    }

    // set all elevations to -9999
    for ( i = 0; i < points.size(); ++i ) {
        points[i].setz( -9999.0 );
    }

    while ( !done ) {
	// find first node with -9999 elevation
        Point3D first;
        bool found_one = false;
        for ( i = 0; i < points.size(); ++i ) {
            if ( points[i].z() < -9000.0 && !found_one ) {
                first = points[i];
                found_one = true;
            }
        }

	if ( found_one ) {
	    SGBucket b( first.x(), first.y() );
	    string base = b.gen_base_path();

	    // try the various elevation sources
            i = 0;
            bool found_file = false;
            while ( !found_file && i < elev_src.size() ) {
                string array_path = root + "/" + elev_src[i] + "/" + base 
                    + "/" + b.gen_index_str();
                if ( array.open(array_path) ) {
                    found_file = true;
                    SG_LOG( SG_GENERAL, SG_DEBUG, "Using array_path = "
                            << array_path );
                }                    
                i++;
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
            for ( i = 0; i < points.size(); ++i ) {
                if ( points[i].z() < -9000.0 ) {
                    done = false;
                    elev = array.altitude_from_grid( points[i].lon() * 3600.0,
                                                     points[i].lat() * 3600.0 );
                    if ( elev > -9000 ) {
                        points[i].setz( elev );
                        // cout << "interpolating for " << p << endl;
                        // cout << p.x() << " " << p.y() << " " << p.z()
                        //      << endl;
                    }
                }
            }

	    array.close();
	} else {
	    done = true;
	}
    }

    // now find the average height of the queried points
    double total = 0.0;
    int count = 0;
    for ( i = 0; i < points.size(); ++i ) {
        total += points[i].z();
        count++;
    }
    double average = total / (double) count;
    SG_LOG(SG_GENERAL, SG_DEBUG, "Average surface height of point list = "
           << average);

    return average;
}


// lookup node elevations for each point in the specified nurbs++
// matrix.  Returns average of all points.

void tgCalcElevations( const string &root, const string_list elev_src,
                       Matrix_Point3Dd &Pts, const double average ) {
    bool done = false;
    int i, j;
    TGArray array;

    // just bail if no work to do
    if ( Pts.rows() == 0 || Pts.cols() == 0 ) {
        return;
    }

    // set all elevations to -9999
    for ( i = 0; i < Pts.cols(); ++i ) {
        for ( j = 0; j < Pts.rows(); ++j ) {
            Point3Dd p = Pts(j,i);
            p.z() = -9999.0;
        }
    }

    while ( !done ) {
	// find first node with -9999 elevation
        Point3Dd first;
        bool found_one = false;
        for ( i = 0; i < Pts.cols(); ++i ) {
            for ( j = 0; j < Pts.rows(); ++j ) {
                Point3Dd p = Pts(j,i);
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
                    SG_LOG( SG_GENERAL, SG_DEBUG, "Using array_path = "
                            << array_path );
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
            for ( i = 0; i < Pts.cols(); ++i ) {
                for ( j = 0; j < Pts.rows(); ++j ) {
                    Point3Dd p = Pts(j,i);
                    if ( p.z() < -9000.0 ) {
                        done = false;
                        elev = array.altitude_from_grid( p.x() * 3600.0,
                                                         p.y() * 3600.0 );
                        if ( elev > -9000 ) {
                            p.z() = elev;
                            Pts(j,i) = p;
                            // cout << "interpolating for " << p << endl;
                            // cout << p.x() << " " << p.y() << " " << p.z()
                            //      << endl;
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
    for ( i = 0; i < Pts.cols(); ++i ) {
        for ( j = 0; j < Pts.rows(); ++j ) {
            Point3Dd p = Pts(j,i);
            total += p.z();
            count++;
        }
    }
    double grid_average = total / (double) count;
    SG_LOG(SG_GENERAL, SG_DEBUG, "Average surface height of nurbs matrix = "
           << grid_average);
}


// clamp all elevations to the specified range

void tgClampElevations( Matrix_Point3Dd &Pts,
                        double center_m, double max_clamp_m )
{
    int i, j;

    // go through the elevations and clamp all elevations to within
    // +/-max_m of the center_m elevation.
    for ( i = 0; i < Pts.cols(); ++i ) {
        for ( j = 0; j < Pts.rows(); ++j ) {
            Point3Dd p = Pts(j,i);
            if ( p.z() < center_m - max_clamp_m ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << center_m - max_clamp_m );
                p.z() = center_m - max_clamp_m;
                Pts(j,i) = p;
            }
            if ( p.z() > center_m + max_clamp_m ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << center_m + max_clamp_m );
                p.z() = center_m + max_clamp_m;
                Pts(j,i) = p;
            }
        }
    }
}
