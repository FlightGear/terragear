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
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#include "elevations.hxx"
#include "global.hxx"
#include "apt_surface.hxx"

SG_USING_NAMESPACE( PLib );


#if 0
// fix node elevations.  Offset is added to the final elevation,
// returns average of all points.
static double calc_elevations( const string &root, const string_list elev_src,
                             Matrix_Point3Dd &Pts ) {
    bool done = false;
    int i, j;
    TGArray array;

    // just bail if no work to do
    if ( Pts.rows() == 0 || Pts.cols() == 0 ) {
        return 0.0;
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
    double average = total / (double) count;
    SG_LOG(SG_GENERAL, SG_DEBUG, "Average surface height = " << average);

    // now go through the elevations and clamp them all to within
    // +/-50m (164') of the average.
    for ( i = 0; i < Pts.cols(); ++i ) {
        for ( j = 0; j < Pts.rows(); ++j ) {
            Point3Dd p = Pts(j,i);
            if ( p.z() < average - max_clamp ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << average - max_clamp );
                p.z() = average - max_clamp;
                Pts(j,i) = p;
            }
            if ( p.z() > average + max_clamp ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.z()
                       << " to " << average + max_clamp );
                p.z() = average + max_clamp;
                Pts(j,i) = p;
            }
        }
    }

    return average;
}
#endif


// Constructor, specify min and max coordinates of desired area in
// lon/lat degrees
TGAptSurface::TGAptSurface( const string& path,
                            const string_list& elev_src,
                            Point3D _min_deg, Point3D _max_deg,
                            double _average_elev_m )
{
    // Calculate desired size of grid
    min_deg = _min_deg;
    max_deg = _max_deg;
    average_elev_m = _average_elev_m;

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

    SG_LOG( SG_GENERAL, SG_DEBUG,
            "Area size = " << y_m << " x " << x_m << " (m)" );

    int xdivs = (int)(x_m / coarse_grid) + 1;
    int ydivs = (int)(y_m / coarse_grid) + 1;

    if ( xdivs < 3 ) { xdivs = 3; }
    if ( ydivs < 3 ) { ydivs = 3; }

    SG_LOG(SG_GENERAL, SG_DEBUG, "  M(" << ydivs << "," << xdivs << ")");
    double dlon = x_deg / xdivs;
    double dlat = y_deg / ydivs;

    double dlon_h = dlon * 0.5;
    double dlat_h = dlat * 0.5;

    // Build the extra res input grid (shifted SW by half (dlon,dlat)
    // with an added major row column on the NE sides.)
    int mult = 10;
    Matrix_Point3Dd dPts( (ydivs + 1) * mult + 1, (xdivs + 1) * mult + 1 );
    for ( int i = 0; i < dPts.cols(); ++i ) {
        for ( int j = 0; j < dPts.rows(); ++j ) {
            dPts(j,i) = Point3Dd( min_deg.lon() - dlon_h
                                    + i * (dlon / (double)mult),
                                  min_deg.lat() - dlat_h
                                    + j * (dlat / (double)mult),
                                  -9999 );
        }
    }

    // Lookup the elevations of all the grid points
    tgCalcElevations( path, elev_src, dPts );

    // Clamp the elevations against the externally provided average
    // elevation.
    tgClampElevations( dPts, average_elev_m, max_clamp );

    // Build the normal res input grid from the double res version
    Matrix_Point3Dd Pts(ydivs + 1, xdivs + 1);
    double ave_divider = (mult+1) * (mult+1);
    for ( int i = 0; i < Pts.cols(); ++i ) {
        for ( int j = 0; j < Pts.rows(); ++j ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, i << "," << j);
            double accum = 0.0;
            double lon_accum = 0.0;
            double lat_accum = 0.0;
            for ( int ii = 0; ii <= mult; ++ii ) {
                for ( int jj = 0; jj <= mult; ++jj ) {
                    double value = dPts(mult*j + jj, mult*i + ii).z();
                    SG_LOG( SG_GENERAL, SG_DEBUG, "value = " << value );
                    accum += value;
                    lon_accum += dPts(mult*j + jj, mult*i + ii).x();
                    lat_accum += dPts(mult*j + jj, mult*i + ii).y();
                }
            }
            double val_ave = accum / ave_divider;
            double lon_ave = lon_accum / ave_divider;
            double lat_ave = lat_accum / ave_divider;

            SG_LOG( SG_GENERAL, SG_DEBUG, "  val_ave = " << val_ave );
            Pts(j,i) = Point3Dd( min_deg.lon() + i * dlon,
                                 min_deg.lat() + j * dlat,
                                 val_ave );

            SG_LOG( SG_GENERAL, SG_DEBUG, "lon_ave = " << lon_ave
                    << "  lat_ave = " << lat_ave );
            SG_LOG( SG_GENERAL, SG_DEBUG, "lon = " << min_deg.lon() + i * dlon
                    << "  lat = " << min_deg.lat() + j * dlat );
        }
    }

    bool slope_error = true;
    while ( slope_error ) {
        SG_LOG( SG_GENERAL, SG_DEBUG, "start of slope processing pass" );
        slope_error = false;
        // Add some "slope" sanity to the resulting surface grid points
        for ( int i = 0; i < Pts.cols() - 1; ++i ) {
            for ( int j = 0; j < Pts.rows() - 1; ++j ) {
                Point3Dd p1, p2;
                double az1, az2, dist;
                double slope;

                p1 = Pts(j,i);
                p2 = Pts(j,i+1);
                geo_inverse_wgs_84( 0, p1.y(), p1.x(), p2.y(), p2.x(),
                                    &az1, &az2, &dist );
                slope = (p2.z() - p1.z()) / dist;

                if ( fabs(slope) > (slope_max + slope_eps) ) {
                    // need to throttle slope, let's move the point
                    // furthest away from average towards the center.

                    slope_error = true;

                    SG_LOG( SG_GENERAL, SG_DEBUG, " (a) detected slope of "
                            << slope << " dist = " << dist );

                    double e1 = average_elev_m - p1.z();
                    double e2 = average_elev_m - p2.z();
                    // cout << "  z1 = " << p1.z() << "  z2 = " << p2.z() << endl;
                    // cout << "  e1 = " << e1 << "  e2 = " << e2 << endl;

                    if ( fabs(e1) > fabs(e2) ) {
                        // cout << "  p1 error larger" << endl;
                        if ( slope > 0 ) {
                            p1.z() = p2.z() - (dist * slope_max);
                        } else {
                            p1.z() = p2.z() + (dist * slope_max);
                        }
                        Pts(j,i) = p1;
                    } else {
                        // cout << "  p2 error larger" << endl;
                        if ( slope > 0 ) {
                            p2.z() = p1.z() + (dist * slope_max);
                        } else {
                            p2.z() = p1.z() - (dist * slope_max);
                        }
                        Pts(j,i+1) = p2;
                    }
                    // cout << "   z1 = " << p1.z() << "  z2 = " << p2.z() << endl;
                }

                p1 = Pts(j,i);
                p2 = Pts(j+1,i);
                geo_inverse_wgs_84( 0, p1.y(), p1.x(), p2.y(), p2.x(),
                                    &az1, &az2, &dist );
                slope = ( p2.z() - p1.z() ) / dist;

                if ( fabs(slope) > (slope_max+slope_eps) ) {
                    // need to throttle slope, let's move the point
                    // furthest away from average towards the center.

                    slope_error = true;

                    SG_LOG( SG_GENERAL, SG_DEBUG, " (b) detected slope of "
                            << slope << " dist = " << dist );

                    double e1 = average_elev_m - p1.z();
                    double e2 = average_elev_m - p2.z();

                    if ( fabs(e1) > fabs(e2) ) {
                        if ( slope > 0 ) {
                            p1.z() = p2.z() - (dist * slope_max);
                        } else {
                            p1.z() = p2.z() + (dist * slope_max);
                        }
                        Pts(j,i) = p1;
                    } else {
                        if ( slope > 0 ) {
                            p2.z() = p1.z() + (dist * slope_max);
                        } else {
                            p2.z() = p1.z() - (dist * slope_max);
                        }
                        Pts(j+1,i) = p2;
                    }
                }
            }
        }
    }

    // Create the nurbs surface

    SG_LOG(SG_GENERAL, SG_DEBUG, "ready to create nurbs surface");
    apt_surf = new PlNurbsSurfaced;
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
    // sanity check
    if ( lon_deg < min_deg.lon() || lon_deg > max_deg.lon() ||
         lat_deg < min_deg.lat() || lat_deg > max_deg.lat() )
    {
        SG_LOG(SG_GENERAL, SG_WARN, "Warning: query out of bounds for NURBS surface!");
        return -9999.0;
    }

    double lat_range = max_deg.lat() - min_deg.lat();
    double lon_range = max_deg.lon() - min_deg.lon();

    // convert lon,lat to nurbs space (NOTE: that this is a dumb
    // approximation that assumes that nurbs surface space exactly
    // corresponds to real world x,y space which it doesn't, but maybe
    // the error is small enough that it doesn't matter?)
    double u = (lat_deg - min_deg.lat()) / lat_range;
    double v = (lon_deg - min_deg.lon()) / lon_range;

    Point3Dd p = apt_surf->pointAt( u, v );

    // double az1, az2, dist;
    // geo_inverse_wgs_84( 0, lat_deg, lon_deg, p.y(), p.x(),
    //                     &az1, &az2, &dist );
    // cout << "query distance error = " << dist << endl;
    
    return p.z();
}

// Query the elevation of a point, return -9999 if out of range
double TGAptSurface::query_solver( double lon_deg, double lat_deg ) {
    const double eps = 0.0000005;

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

    // cout << "running quick solver ..." << endl;

    int count;
    double dx = 1000;
    double dy = 1000;
    Point3Dd p;

    // cout << " solving for u,v simultaneously" << endl;
    count = 0;
    while ( count < 0 ) {
        if ( u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 ) {
            // oops, something goofed in the solver
        }

        p = apt_surf->pointAt( u, v );
        // cout << "  querying for " << u << ", " << v << " = " << p.z()
        //      << endl;
        // cout << "    found " << p.x() << ", " << p.y() << " = " << p.z()
        //      << endl;
        dx = lon_deg - p.x();
        dy = lat_deg - p.y();
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

    // cout << "quick query solver failed..." << endl;

    // failed to converge with fast approach, try a binary paritition
    // scheme, however we have to solve each axis independently
    // because when we move in one axis we may change our position in
    // the other axis.

    double min_u = 0.0; 
    double max_u = 1.0;
    double min_v = 0.0; 
    double max_v = 1.0;

    u = (max_u + min_u) / 2.0;
    v = (max_v + min_v) / 2.0;

    dx = 1000.0;
    dy = 1000.0;

    while ( true ) {
        // cout << "solving for u" << endl;
        
        min_u = 0.0; 
        max_u = 1.0;
        count = 0;
        while ( true ) {
            p = apt_surf->pointAt( u, v );
            // cout << "  binary querying for " << u << ", " << v << " = "
            //      << p.z() << endl;
            // cout << "    found " << p.x() << ", " << p.y() << " = " << p.z()
            //      << endl;
            dy = lat_deg - p.y();
            dx = lon_deg - p.x();
            // cout << "      dx = " << dx << " dy = " << dy << endl;

            // double az1, az2, dist;
            // geo_inverse_wgs_84( 0, lat_deg, lon_deg, p.y(), p.x(),
            //                     &az1, &az2, &dist );
            // cout << "      query distance error = " << dist << endl;

            if ( fabs(dy) < eps ) {
                break;
            } else {
                if ( dy >= eps ) {
                    min_u = u;
                } else if ( dy <= eps ) {
                    max_u = u;
                }
            }

            u = (max_u + min_u) / 2.0;
            ++count;
        }

        // cout << "solving for v" << endl;

        min_v = 0.0; 
        max_v = 1.0;
        count = 0;
        while ( true ) {
            p = apt_surf->pointAt( u, v );
            // cout << "  binary querying for " << u << ", " << v << " = "
            //      << p.z() << endl;
            // cout << "    found " << p.x() << ", " << p.y() << " = " << p.z()
            //      << endl;
            dx = lon_deg - p.x();
            dy = lat_deg - p.y();
            // cout << "      dx = " << dx << " dy = " << dy << endl;

            // double az1, az2, dist;
            // geo_inverse_wgs_84( 0, lat_deg, lon_deg, p.y(), p.x(),
            //                     &az1, &az2, &dist );
            // cout << "      query distance error = " << dist << endl;
            if ( fabs(dx) < eps ) {
                break;
            } else {
                if ( dx >= eps ) {
                    min_v = v;
                } else if ( dx <= eps ) {
                    max_v = v;
                }
            }
            v = (max_v + min_v) / 2.0;

            ++count;
        }

        if ( (fabs(dx) < eps) && (fabs(dy) < eps) ) {
            // double az1, az2, dist;
            // geo_inverse_wgs_84( 0, lat_deg, lon_deg, p.y(), p.x(),
            //                     &az1, &az2, &dist );
            // cout << "        final query distance error = " << dist << endl;
            return p.z();
        }
    }

    cout << "binary solver failed..." << endl;

    return p.z();
}
