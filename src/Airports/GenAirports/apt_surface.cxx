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
#include <simgear/debug/logstream.hxx>

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
    for ( i = 0; i < Pts.cols(); ++i ) {
        for ( j = 0; j < Pts.rows(); ++j ) {
            Point3Df p = Pts(j,i);
            p.z() = -9999.0;
        }
    }

    while ( !done ) {
	// find first node with -9999 elevation
        Point3Df first;
        bool found_one = false;
        for ( i = 0; i < Pts.cols(); ++i ) {
            for ( j = 0; j < Pts.rows(); ++j ) {
                Point3Df p = Pts(j,i);
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
                    SG_LOG(SG_GENERAL, SG_DEBUG, "Using array_path = " << array_path);
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
                    Point3Df p = Pts(j,i);
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
            Point3Df p = Pts(j,i);
            total += p.z();
            count++;
        }
    }
    double average = total / (double) count;
    SG_LOG(SG_GENERAL, SG_DEBUG, "Average surface height = " << average);

    // now go through the elevations and clamp them all to within
    // +/-10m (33') of the average.
    const double dz = 100.0;
    for ( i = 0; i < Pts.cols(); ++i ) {
        for ( j = 0; j < Pts.rows(); ++j ) {
            Point3Df p = Pts(j,i);
            if ( p.z() < average - dz ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << average - dz );
                p.z() = average - dz;
                Pts(j,i) = p;
            }
            if ( p.z() > average + dz ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << average + dz );
                p.z() = average + dz;
                Pts(j,i) = p;
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

    SG_LOG(SG_GENERAL, SG_DEBUG, "Area size = " << x_m << " x " << y_m << " (m)");

    int xdivs = (int)(x_m / 600.0) + 1;
    int ydivs = (int)(y_m / 600.0) + 1;

    if ( xdivs < 3 ) { xdivs = 3; }
    if ( ydivs < 3 ) { ydivs = 3; }

    SG_LOG(SG_GENERAL, SG_DEBUG, "  M(" << xdivs << "," << ydivs << ")");
    double dlon = x_deg / xdivs;
    double dlat = y_deg / ydivs;

    // Build the extra res input grid
    int mult = 10;
    Matrix_Point3Df dPts( (ydivs+2) * mult + 1, (xdivs+2) * mult + 1 );
    for ( int i = 0; i < dPts.cols(); ++i ) {
        for ( int j = 0; j < dPts.rows(); ++j ) {
            dPts(j,i) = Point3Df( min_deg.lon() + (i-mult)*(dlon/(double)mult),
                                  min_deg.lat() + (j-mult)*(dlat/(double)mult),
                                  -9999 );
        }
    }

    // Determine elevation of the grid points
    calc_elevations( path, elev_src, dPts );

    // Build the normal res input grid from the double res version
    Matrix_Point3Df Pts(ydivs + 1, xdivs + 1);
    for ( int i = 0; i < Pts.cols(); ++i ) {
        for ( int j = 0; j < Pts.rows(); ++j ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, i << "," << j);
            double accum = 0.0;
            for ( int ii = 0; ii < mult; ++ii ) {
                for ( int jj = 0; jj < mult; ++jj ) {
                    double value = dPts(mult*(j+1) - (mult/2) + jj,
                                        mult*(i+1) - (mult/2) + ii).z();
                    SG_LOG( SG_GENERAL, SG_DEBUG, "value = " << value );
                    accum += dPts(mult*(j+1) - (mult/2) + jj,
                                  mult*(i+1) - (mult/2) + ii).z();
                }
            }
            SG_LOG( SG_GENERAL, SG_DEBUG,
                    "  average = " << accum / (mult*mult) );
            Pts(j,i) = Point3Df( min_deg.lon() + i * dlon,
                                 min_deg.lat() + j * dlat,
                                 accum / (mult*mult) );
        }
    }

#if 0
    // Add some "slope" sanity to the resulting surface grid points
    for ( i = 0; i < Pts.cols() - 1; ++i ) {
        for ( j = 0; j < Pts.rows() - 1; ++j ) {
            Point3Df p = Pts(j,i);
            Point3Df p1 = Pts(j,i+1);
            geo_inverse_wgs_84( 0.0, 

            if ( p.z() < average - dz ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << average - dz );
                p.z() = average - dz;
                Pts(j,i) = p;
            }
            if ( p.z() > average + dz ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << average + dz );
                p.z() = average + dz;
                Pts(j,i) = p;
            }
        }
    }
#endif

    // Create the nurbs surface

    SG_LOG(SG_GENERAL, SG_DEBUG, "ready to create nurbs surface");
    apt_surf = new PlNurbsSurfacef;
    apt_surf->globalInterp( Pts, 3, 3);
    SG_LOG(SG_GENERAL, SG_DEBUG, "  successful.");

#if 0
    // For debugging only: output an array of surface points suitable
    // for plotting with gnuplot.  This is useful for comparing the
    // approximated and smoothed surface to the original rough
    // surface.
    cout << "DEBUGGING TEST" << endl;
    for ( int i = 0; i < (xdivs+2) * mult + 1; ++i ) {
        for ( int j = 0; j < (ydivs+2) * mult + 1; ++j ) {
            double x = min_deg.lon() + (i-mult)*(dlon/(double)mult);
            double y = min_deg.lat() + (j-mult)*(dlat/(double)mult);
            cout << x << " " << y << " " << query(x,y) << endl;
        }
    }
#endif
}


TGAptSurface::~TGAptSurface() {
    delete apt_surf;
}


// Query the elevation of a point, return -9999 if out of range
double TGAptSurface::query( double lon_deg, double lat_deg ) {
    const double eps = 0.00001;

    // sanity check
    if ( lon_deg < min_deg.lon() || lon_deg > max_deg.lon() ||
         lat_deg < min_deg.lat() || lat_deg > max_deg.lat() )
    {
        SG_LOG(SG_GENERAL, SG_WARN, "Warning: query out of bounds for NURBS surface!");
        return -9999.0;
    }

    double lat_range = max_deg.lat() - min_deg.lat();
    double lon_range = max_deg.lon() - min_deg.lon();

    // convert lon,lat to nurbs space
    double u = (lat_deg - min_deg.lat()) / lat_range;
    double v = (lon_deg - min_deg.lon()) / lon_range;

    int count = 0;
    Point3Df p;
    while ( count < 20 ) {
        if ( u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 ) {
            // oops, something goofed in the solver
        }

        p = apt_surf->pointAt( u, v );
        // cout << "  querying for " << u << ", " << v << " = " << p.z()
        //      << endl;
        // cout << "    found " << p.x() << ", " << p.y() << " = " << p.z()
        //      << endl;
        double dx = lon_deg - p.x();
        double dy = lat_deg - p.y();
        // cout << "      dx = " << dx << " dy = " << dy << endl;

        if ( (fabs(dx) < eps) && (fabs(dy) < eps) ) {
            return p.z();
        } else {
            u = u + (dy/lat_range);
            v = v + (dx/lon_range);
            if ( u < 0.0 ) { u = 0.0; }
            if ( u > 1.0 ) { u = 1.0; }
            if ( v < 0.0 ) { v = 0.0; }
            if ( v > 1.0 ) { v = 1.0; }
        }

        ++count;
    }

    // failed to converge with fast approach, try a binary paritition
    // scheme
    double min_u = 0.0; 
    double max_u = 1.0;
    double min_v = 0.0;
    double max_v = 1.0;
    count = 0;
    while ( count < 30 ) {
        u = (max_u + min_u) / 2.0;
        v = (max_v + min_v) / 2.0;
        p = apt_surf->pointAt( u, v );
        // cout << "  binary querying for " << u << ", " << v << " = "
        //      << p.z() << endl;
        // cout << "    found " << p.x() << ", " << p.y() << " = " << p.z()
        //      << endl;
        double dx = lon_deg - p.x();
        double dy = lat_deg - p.y();
        // cout << "      dx = " << dx << " dy = " << dy << endl;

        if ( (fabs(dx) < eps) && (fabs(dy) < eps) ) {
            return p.z();
        } else {
            if ( dy >= eps ) {
                min_u = u;
            } else if ( dy <= eps ) {
                max_u = u;
            }
            if ( dx >= eps ) {
                min_v = v;
            } else if ( dx <= eps ) {
                max_v = v;
            }
        }

        ++count;
    }

    return p.z();
}
