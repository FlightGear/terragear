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


static bool limit_slope( Matrix_Point3Dd &Pts, int i1, int j1, int i2, int j2,
                         double average_elev_m )
{
    bool slope_error = false;

    Point3Dd p1, p2;
    p1 = Pts(i1,j1);
    p2 = Pts(i2,j2);

    double az1, az2, dist;
    double slope;

    geo_inverse_wgs_84( 0, p1.y(), p1.x(), p2.y(), p2.x(),
                        &az1, &az2, &dist );
    slope = (p2.z() - p1.z()) / dist;

    if ( fabs(slope) > (slope_max + slope_eps) ) {
        // need to throttle slope, let's move the point
        // furthest away from average towards the center.

        slope_error = true;

        SG_LOG( SG_GENERAL, SG_DEBUG, " (a) detected slope of "
                << slope << " dist = " << dist );

        double e1 = fabs(average_elev_m - p1.z());
        double e2 = fabs(average_elev_m - p2.z());
        // cout << "  z1 = " << p1.z() << "  z2 = " << p2.z() << endl;
        // cout << "  e1 = " << e1 << "  e2 = " << e2 << endl;

        if ( e1 > e2 ) {
            // cout << "  p1 error larger" << endl;
            if ( slope > 0 ) {
                p1.z() = p2.z() - (dist * slope_max);
            } else {
                p1.z() = p2.z() + (dist * slope_max);
            }
            Pts(i1,j1) = p1;
        } else {
            // cout << "  p2 error larger" << endl;
            if ( slope > 0 ) {
                p2.z() = p1.z() + (dist * slope_max);
            } else {
                p2.z() = p1.z() - (dist * slope_max);
            }
            Pts(i2,j2) = p2;
        }
        // cout << "   z1 = " << p1.z() << "  z2 = " << p2.z() << endl;
    }

    return slope_error;
}


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

#if defined( _NURBS_GLOBAL_INTERP )
    if ( xdivs < 3 ) { xdivs = 3; }
    if ( ydivs < 3 ) { ydivs = 3; }
#elif defined( _NURBS_LEAST_SQUARES )
    // Minimum divs appears to need to be at least 5 before the
    // leastsquares nurbs surface approximation stops crashing.
    if ( xdivs < 6 ) { xdivs = 6; }
    if ( ydivs < 6 ) { ydivs = 6; }
#else
# error "Need to define _NURBS_GLOBAL_INTER or _NURBS_LEAST_SQUARES"
#endif
    SG_LOG(SG_GENERAL, SG_INFO, "  M(" << ydivs << "," << xdivs << ")");
    double dlon = x_deg / xdivs;
    double dlat = y_deg / ydivs;

    double dlon_h = dlon * 0.5;
    double dlat_h = dlat * 0.5;

    // Build the extra res input grid (shifted SW by half (dlon,dlat)
    // with an added major row column on the NE sides.)
    int mult = 10;
    Matrix_Point3Dd dPts( (ydivs + 1) * mult + 1, (xdivs + 1) * mult + 1 );
    for ( int j = 0; j < dPts.cols(); ++j ) {
        for ( int i = 0; i < dPts.rows(); ++i ) {
            dPts(i,j) = Point3Dd( min_deg.lon() - dlon_h
                                    + j * (dlon / (double)mult),
                                  min_deg.lat() - dlat_h
                                    + i * (dlat / (double)mult),
                                  -9999 );
        }
    }

    // Lookup the elevations of all the grid points
    tgCalcElevations( path, elev_src, dPts, 0.0 );
#ifdef DEBUG
    for ( int j = 0; j < dPts.cols(); ++j ) {
        for ( int i = 0; i < dPts.rows(); ++i ) {
            printf("%.5f %.5f %.1f\n", dPts(i,j).x(), dPts(i,j).y(),
                   dPts(i,j).z() );
        }
    }
#endif

    // Clamp the elevations against the externally provided average
    // elevation.
    tgClampElevations( dPts, average_elev_m, max_clamp );

#ifdef DEBUG
    for ( int j = 0; j < dPts.cols(); ++j ) {
        for ( int i = 0; i < dPts.rows(); ++i ) {
            printf("%.5f %.5f %.1f\n", dPts(i,j).x(), dPts(i,j).y(),
                   dPts(i,j).z() );
        }
    }
#endif

    // Build the normal res input grid from the double res version
    Matrix_Point3Dd Pts(ydivs + 1, xdivs + 1);
    double ave_divider = (mult+1) * (mult+1);
    for ( int j = 0; j < Pts.cols(); ++j ) {
        for ( int i = 0; i < Pts.rows(); ++i ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, i << "," << j);
            double accum = 0.0;
            double lon_accum = 0.0;
            double lat_accum = 0.0;
            for ( int jj = 0; jj <= mult; ++jj ) {
                for ( int ii = 0; ii <= mult; ++ii ) {
                    double value = dPts(mult*i + ii, mult*j + jj).z();
                    SG_LOG( SG_GENERAL, SG_DEBUG, "value = " << value );
                    accum += value;
                    lon_accum += dPts(mult*i + ii, mult*j + jj).x();
                    lat_accum += dPts(mult*i + ii, mult*j + jj).y();
                }
            }
            double val_ave = accum / ave_divider;
            double lon_ave = lon_accum / ave_divider;
            double lat_ave = lat_accum / ave_divider;

            SG_LOG( SG_GENERAL, SG_DEBUG, "  val_ave = " << val_ave );
            Pts(i,j) = Point3Dd( min_deg.lon() + j * dlon,
                                 min_deg.lat() + i * dlat,
                                 val_ave );
            SG_LOG( SG_GENERAL, SG_DEBUG, "lon_ave = " << lon_ave
                    << "  lat_ave = " << lat_ave );
            SG_LOG( SG_GENERAL, SG_DEBUG, "lon = " << min_deg.lon() + j * dlon
                    << "  lat = " << min_deg.lat() + i * dlat );
        }
    }

#ifdef DEBUG
    for ( int j = 0; j < Pts.cols(); ++j ) {
        for ( int i = 0; i < Pts.rows(); ++i ) {
            printf("%.5f %.5f %.1f\n", Pts(i,j).x(), Pts(i,j).y(),
                   Pts(i,j).z() );
        }
    }
#endif

    bool slope_error = true;
    while ( slope_error ) {
        SG_LOG( SG_GENERAL, SG_DEBUG, "start of slope processing pass" );
        slope_error = false;
        // Add some "slope" sanity to the resulting surface grid points
        for ( int j = 0; j < Pts.cols() - 1; ++j ) {
            for ( int i = 0; i < Pts.rows() - 1; ++i ) {
                if ( limit_slope( Pts, i, j, i+1, j, average_elev_m ) ) {
                    slope_error = true;
                }
                if ( limit_slope( Pts, i, j, i, j+1, average_elev_m ) ) {
                    slope_error = true;
                }
                if ( limit_slope( Pts, i, j, i+1, j+1, average_elev_m ) ) {
                    slope_error = true;
                }
            }
        }
    }

#ifdef DEBUG
    for ( int j = 0; j < Pts.cols(); ++j ) {
        for ( int i = 0; i < Pts.rows(); ++i ) {
            printf("%.5f %.5f %.1f\n", Pts(i,j).x(), Pts(i,j).y(),
                   Pts(i,j).z() );
        }
    }
#endif

    // Create the nurbs surface

    SG_LOG(SG_GENERAL, SG_DEBUG, "ready to create nurbs surface");
    apt_surf = new PlNurbsSurfaced;
#if defined( _NURBS_GLOBAL_INTERP )
    apt_surf->globalInterp( Pts, 3, 3);
#elif defined( _NURBS_LEAST_SQUARES )
    cout << "Col = " << Pts.cols() << " Rows = " << Pts.rows() << endl;
    int nU = Pts.rows() / 2; if ( nU < 4 ) { nU = 4; }
    int nV = Pts.cols() / 2; if ( nV < 4 ) { nV = 4; }
    cout << "nU = " << nU << " nV = " << nV << endl;
    apt_surf->leastSquares( Pts, 3, 3, nU, nV );

    // sanity check: I'm finding that leastSquares() can produce nan
    // surfaces.  We test for this and fall back to globalInterp() if
    // the least squares fails.
    double result =  query_solver( (min_deg.lon() + max_deg.lon()) / 2.0,
                                   (min_deg.lat() + max_deg.lat()) / 2.0 );
    Point3Dd p = apt_surf->pointAt( 0.5, 0.5 );

    if ( (result > -9000.0) && (p.z() <= 0.0 || p.z() >= 0.0) ) {
        // ok, a valid number
    } else {
        // no, sorry, a nan is not <= 0.0 or >= 0.0
        SG_LOG(SG_GENERAL, SG_WARN,
               "leastSquares() nurbs interpolation failed!!!");
        char command[256];
        sprintf( command,
                 "echo least squares nurbs interpolation failed, using globalInterp() >> last_apt" );
        system( command );

        // we could fall back to globalInterp() rather than aborting
        // if we wanted to ...
        apt_surf->globalInterp( Pts, 3, 3);
    }
#else
# error "Need to define _NURBS_GLOBAL_INTER or _NURBS_LEAST_SQUARES"
#endif
    SG_LOG(SG_GENERAL, SG_DEBUG, "  successful.");

#ifdef DEBUG
    // For debugging only: output an array of surface points suitable
    // for plotting with gnuplot.  This is useful for comparing the
    // approximated and smoothed surface to the original rough
    // surface.
    cout << "DEBUGGING TEST" << endl;
    const int divs = 100;
    double dx = x_deg / divs;
    double dy = y_deg / divs;
    for ( int j = 0; j < divs; ++j ) {
        for ( int i = 0; i < divs; ++i ) {
            double lon = min_deg.lon() + j * dx;
            double lat = min_deg.lat() + i * dy;
            printf("%.5f %.5f %.1f\n", lon, lat, query_solver(lon, lat) );
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
    while ( count < 0 /* disable fast solver for now */ ) {
        if ( u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 ) {
            cout << "oops, something goofed in the solver" << endl;
            exit(-1);
        }

        p = apt_surf->pointAt( u, v );
        // cout << "  querying for " << u << ", " << v << " = " << p.z()
        //      << endl;
        // cout << "    found " << p.x() << ", " << p.y() << " = " << p.z()
        //      << endl;
        dx = lon_deg - p.x();
        dy = lat_deg - p.y();
        // cout << "      dx = " << dx << " dy = " << dy << endl;

        if ( (fabs(dx) < nurbs_eps) && (fabs(dy) < nurbs_eps) ) {
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

    // failed to converge with fast approach, try a binary partition
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

    int gcount = 0;

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
            dx = lon_deg - p.x();
            dy = lat_deg - p.y();
            // cout << "      dx = " << dx << " dy = " << dy << " z = " << p.z() << endl;

            // double az1, az2, dist;
            // geo_inverse_wgs_84( 0, lat_deg, lon_deg, p.y(), p.x(),
            //                     &az1, &az2, &dist );
            // cout << "      query distance error = " << dist << endl;

            if ( fabs(dy) < nurbs_eps ) {
                break;
            } else {
                if ( dy >= nurbs_eps ) {
                    min_u = u;
                } else if ( dy <= nurbs_eps ) {
                    max_u = u;
                }
            }

            u = (max_u + min_u) / 2.0;

            if ( count > 100 ) {
                // solver failed
                cout << "binary solver failed..." << endl;
                return -9999.0;
            }

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
            // cout << "      dx = " << dx << " dy = " << dy << " z = " << p.z() << endl;

            // double az1, az2, dist;
            // geo_inverse_wgs_84( 0, lat_deg, lon_deg, p.y(), p.x(),
            //                     &az1, &az2, &dist );
            // cout << "      query distance error = " << dist << endl;
            if ( fabs(dx) < nurbs_eps ) {
                break;
            } else {
                if ( dx >= nurbs_eps ) {
                    min_v = v;
                } else if ( dx <= nurbs_eps ) {
                    max_v = v;
                }
            }
            v = (max_v + min_v) / 2.0;

            if ( count > 100 ) {
                // solver failed
                cout << "binary solver failed..." << endl;
                return -9999.0;
            }

            ++count;
        }

        // cout << "query count = " << gcount << "  dist = " << dx << ", "
        //      << dy << endl;

        if ( (fabs(dx) < nurbs_eps) && (fabs(dy) < nurbs_eps) ) {
            // double az1, az2, dist;
            // geo_inverse_wgs_84( 0, lat_deg, lon_deg, p.y(), p.x(),
            //                     &az1, &az2, &dist );
            // cout << "        final query distance error = " << dist << endl;
            return p.z();
        }
    }

    return p.z();
}
