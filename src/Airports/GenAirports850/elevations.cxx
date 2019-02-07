// elevations.cxx -- routines to help calculate DEM elevations for a
//                   set of points
//
// Written by Curtis Olson, started April 2004.
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/constants.h>
#include <simgear/math/SGMath.hxx>
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#include "global.hxx"
#include "debug.hxx"


// lookup node elevations for each point in the SGGeod list.  Returns
// average of all points.  Doesn't modify the original list.
double tgAverageElevation( const std::string &root, const string_list elev_src,
                               const std::vector<SGGeod>& points_source )
{
    bool done = false;
    unsigned int i;
    TGArray array;

    // make a copy so our routine is non-destructive.
    std::vector<SGGeod> points = points_source;

    // just bail if no work to do
    if ( points.empty() ) {
        return 0.0;
    }

    // set all elevations to -9999
    for ( i = 0; i < points.size(); ++i ) {
        points[i].setElevationM( -9999.0 );
    }

    while ( !done ) {
        // find first node with -9999 elevation
        SGGeod first = SGGeod();
        bool found_one = false;
        for ( i = 0; i < points.size(); ++i ) {
            if ( points[i].getElevationM() < -9000.0 && !found_one ) {
                first = points[i];
                TG_LOG( SG_GENERAL, SG_DEBUG, "found first = " << first );

                found_one = true;
            }
        }

        if ( found_one ) {
            SGBucket b( first );
            std::string base = b.gen_base_path();

            // try the various elevation sources
            i = 0;
            bool found_file = false;
            while ( !found_file && i < elev_src.size() ) {
                std::string array_path = root + "/" + elev_src[i] + "/" + base + "/" + b.gen_index_str();

                if ( array.open(array_path) ) {
                    found_file = true;
                    TG_LOG( SG_GENERAL, SG_DEBUG, "Using array_path = " << array_path );
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
                if ( points[i].getElevationM() < -9000.0 ) {
                    done = false;
                    elev = array.altitude_from_grid( points[i].getLongitudeDeg() * 3600.0,
                                                     points[i].getLatitudeDeg() * 3600.0 );
                    if ( elev > -9000 ) {
                        points[i].setElevationM( elev );
                    }
                }
            }

            array.unload();
        } else {
            done = true;
        }
    }

    // now find the average height of the queried points
    double total = 0.0;
    int count = 0;
    for ( i = 0; i < points.size(); ++i ) {
        total += points[i].getElevationM();
        count++;
    }
    double average = total / (double) count;
    TG_LOG(SG_GENERAL, SG_DEBUG, "Average surface height of point list = " << average);

    return average;
}
