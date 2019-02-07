// apt_surface.cxx -- class to manage airport terrain surface
//                    approximation and smoothing
//
// Written by Curtis Olson, started March 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - http://www.flightgear.org/~curt
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

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/math/SGMath.hxx>
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#include "TNT/jama_qr.h"
#include "tg_surface.hxx"

// Final grid size for surface (in meters)
const double coarse_grid = 300.0;

// compared to the average surface elevation, clamp all values within
// this many meters of the average
const double max_clamp = 100.0;

static bool limit_slope( tgMatrix* Pts, int i1, int j1, int i2, int j2,
                         double average_elev_m, double slope_max, double slope_eps )
{
    bool slope_error = false;

    SGGeod p1, p2;
    p1 = Pts->element(i1,j1);
    p2 = Pts->element(i2,j2);

    double dist = SGGeodesy::distanceM(p1, p2);
    double slope = (p2.getElevationM() - p1.getElevationM()) / dist;

    if ( fabs(slope) > (slope_max + slope_eps) ) {
        // need to throttle slope, let's move the point
        // furthest away from average towards the center.

        slope_error = true;

        SG_LOG( SG_GENERAL, SG_DEBUG, " (a) detected slope of " << slope << " dist = " << dist );

        double e1 = fabs(average_elev_m - p1.getElevationM());
        double e2 = fabs(average_elev_m - p2.getElevationM());

        if ( e1 > e2 ) {
            // p1 error larger
            if ( slope > 0 ) {
                p1.setElevationM( p2.getElevationM() - (dist * slope_max) );
            } else {
                p1.setElevationM( p2.getElevationM() + (dist * slope_max) );
            }
            Pts->set(i1, j1, p1);
        } else {
            // p2 error larger
            if ( slope > 0 ) {
                p2.setElevationM( p1.getElevationM() + (dist * slope_max) );
            } else {
                p2.setElevationM( p1.getElevationM() - (dist * slope_max) );
            }
            Pts->set(i2, j2, p2);
        }
    }

    return slope_error;
}


// lookup node elevations for each point in the specified simple
// matrix.  Returns average of all points.
static void tgCalcElevations( const std::string &root, const string_list elev_src,
                              tgMatrix &Pts, const double average )
{
    bool done = false;
    int i, j;
    TGArray array;

    // just bail if no work to do
    if ( Pts.rows() == 0 || Pts.cols() == 0 ) {
        return;
    }

    // set all elevations to -9999
    for ( j = 0; j < Pts.rows(); ++j ) {
        for ( i = 0; i < Pts.cols(); ++i ) {
            SGGeod p = Pts.element(i, j);
            p.setElevationM(-9999.0);
            Pts.set(i, j, p);
        }
    }

    while ( !done ) {
        // find first node with -9999 elevation
        SGGeod first = SGGeod();
        bool found_one = false;
        for ( j = 0; j < Pts.rows(); ++j ) {
            for ( i = 0; i < Pts.cols(); ++i ) {
                SGGeod p = Pts.element(i,j);
                if ( p.getElevationM() < -9000.0 && !found_one ) {
                    first = p;
                    found_one = true;
                }
            }
        }

        if ( found_one ) {
            SGBucket b( first );
            std::string base = b.gen_base_path();

            // try the various elevation sources
            j = 0;
            bool found_file = false;
            while ( !found_file && j < (int)elev_src.size() ) {
                std::string array_path = root + "/" + elev_src[j] + "/" + base + "/" + b.gen_index_str();

                if ( array.open(array_path) ) {
                    found_file = true;
                    SG_LOG( SG_GENERAL, SG_DEBUG, "Using array_path = " << array_path );
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
            for ( j = 0; j < Pts.rows(); ++j ) {
                for ( i = 0; i < Pts.cols(); ++i ) {
                    SGGeod p = Pts.element(i,j);
                    if ( p.getElevationM() < -9000.0 ) {
                        done = false;
                        elev = array.altitude_from_grid( p.getLongitudeDeg() * 3600.0,
                                                         p.getLatitudeDeg() * 3600.0 );
                        if ( elev > -9000 ) {
                            p.setElevationM( elev );
                            Pts.set(i, j, p);
                        }
                    }
                }
            }

            array.unload();

        } else {
            done = true;
        }
    }

#ifdef DEBUG
    // do some post processing for sanity's sake
    // find the average height of the queried points
    double total = 0.0;
    int count = 0;
    for ( j = 0; j < Pts.rows(); ++j ) {
        for ( i = 0; i < Pts.cols(); ++i ) {
            SGGeod p = Pts.element(i,j);
            total += p.getElevationM();
            count++;
        }
    }
    double grid_average = total / (double) count;
    SG_LOG(SG_GENERAL, SG_DEBUG, "Average surface height of matrix = " << grid_average);
#endif
}

// clamp all elevations to the specified range
static void tgClampElevations( tgMatrix& Pts,
                               double center_m,
                               double max_clamp_m )
{
    int i, j;

    // go through the elevations and clamp all elevations to within
    // +/-max_m of the center_m elevation.
    for ( j = 0; j < Pts.rows(); ++j ) {
        for ( i = 0; i < Pts.cols(); ++i ) {
            SGGeod p = Pts.element(i,j);
            if ( p.getElevationM() < center_m - max_clamp_m ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.getElevationM() << " to " << center_m - max_clamp_m );
                p.setElevationM( center_m - max_clamp_m );
                Pts.set(i, j, p);
            }
            if ( p.getElevationM() > center_m + max_clamp_m ) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "   clamping " << p.getElevationM() << " to " << center_m + max_clamp_m );
                p.setElevationM( center_m + max_clamp_m );
                Pts.set(i, j, p);
            }
        }
    }
}

// Constructor, specify min and max coordinates of desired area in
// lon/lat degrees
tgSurface::tgSurface( const std::string& path,
                      const string_list& elev_src,
                      tgRectangle aptBounds,
                      double average_elev_m,
                      double slope_max,
                      double slope_eps
                    ) :
    _aptBounds(aptBounds)
{
    // Calculate desired size of grid
    _min_deg = _aptBounds.getMin();
    _max_deg = _aptBounds.getMax();
    _average_elev_m = average_elev_m;

    // The following size calculations are for the purpose of
    // determining grid divisions so it's not important that they be
    // *exact*, just ball park.
    double y_deg = _max_deg.getLatitudeDeg() - _min_deg.getLatitudeDeg();
    double y_rad = y_deg * SG_DEGREES_TO_RADIANS;
    double y_nm = y_rad * SG_RAD_TO_NM;
    double y_m = y_nm * SG_NM_TO_METER;

    double xfact = cos( _min_deg.getLatitudeRad() );
    double x_deg = _max_deg.getLongitudeDeg() - _min_deg.getLongitudeDeg();
    double x_rad = x_deg * SG_DEGREES_TO_RADIANS;
    double x_nm = x_rad * SG_RAD_TO_NM * xfact;
    double x_m = x_nm * SG_NM_TO_METER;

    SG_LOG( SG_GENERAL, SG_DEBUG, "Area size = " << y_m << " x " << x_m << " (m)" );

    int xdivs = (int)(x_m / coarse_grid) + 1;
    int ydivs = (int)(y_m / coarse_grid) + 1;

    // set an arbitrary minumum number of divisions to keep things
    // interesting
    if ( xdivs < 8 ) { xdivs = 8; }
    if ( ydivs < 8 ) { ydivs = 8; }

    SG_LOG(SG_GENERAL, SG_DEBUG, "  M(" << ydivs << "," << xdivs << ")");

    double dlon = x_deg / xdivs;
    double dlat = y_deg / ydivs;

    double dlon_h = dlon * 0.5;
    double dlat_h = dlat * 0.5;

    // Build the extra res input grid (shifted SW by half (dlon,dlat)
    // with an added major row column on the NE sides.)
    int mult = 10;
    tgMatrix dPts( (xdivs + 1) * mult + 1, (ydivs + 1) * mult + 1 );
    for ( int j = 0; j < dPts.rows(); ++j ) {
        for ( int i = 0; i < dPts.cols(); ++i ) {
            dPts.set(i, j, SGGeod::fromDegM( _min_deg.getLongitudeDeg() - dlon_h + i * (dlon / (double)mult),
                     _min_deg.getLatitudeDeg() - dlat_h + j * (dlat / (double)mult),
                     -9999 ) );
        }
    }

    // Lookup the elevations of all the grid points
    tgCalcElevations( path, elev_src, dPts, 0.0 );

    // Clamp the elevations against the externally provided average
    // elevation.
    tgClampElevations( dPts, _average_elev_m, max_clamp );

    // Build the normal res input grid from the double res version
    Pts = new tgMatrix(xdivs + 1, ydivs + 1 );
    double ave_divider = (mult+1) * (mult+1);
    for ( int j = 0; j < Pts->rows(); ++j ) {
        for ( int i = 0; i < Pts->cols(); ++i ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, i << "," << j);
            double accum = 0.0;
            double lon_accum = 0.0;
            double lat_accum = 0.0;
            for ( int jj = 0; jj <= mult; ++jj ) {
                for ( int ii = 0; ii <= mult; ++ii ) {
                    double value = dPts.element(mult*i + ii, mult*j + jj).getElevationM();
                    SG_LOG( SG_GENERAL, SG_DEBUG, "value = " << value );
                    accum += value;
                    lon_accum += dPts.element(mult*i + ii, mult*j + jj).getLongitudeDeg();
                    lat_accum += dPts.element(mult*i + ii, mult*j + jj).getLatitudeDeg();
                }
            }
            double val_ave = accum / ave_divider;

            SG_LOG( SG_GENERAL, SG_DEBUG, "  val_ave = " << val_ave );
            Pts->set(i, j, SGGeod::fromDegM( _min_deg.getLongitudeDeg() + i * dlon,
				    _min_deg.getLatitudeDeg() + j * dlat,
				    val_ave )
		    );
        }
    }

    bool slope_error = true;
    while ( slope_error ) {
        SG_LOG( SG_GENERAL, SG_DEBUG, "start of slope processing pass" );
        slope_error = false;
        // Add some "slope" sanity to the resulting surface grid points
        for ( int j = 0; j < Pts->rows() - 1; ++j ) {
            for ( int i = 0; i < Pts->cols() - 1; ++i ) {
                if ( limit_slope( Pts, i, j, i+1, j, _average_elev_m, slope_max, slope_eps ) ) {
                    slope_error = true;
                }
                if ( limit_slope( Pts, i, j, i, j+1, _average_elev_m, slope_max, slope_eps ) ) {
                    slope_error = true;
                }
                if ( limit_slope( Pts, i, j, i+1, j+1, _average_elev_m, slope_max, slope_eps ) ) {
                    slope_error = true;
                }
            }
        }
    }

    // compute an central offset point.
    double clon = (_min_deg.getLongitudeDeg() + _max_deg.getLongitudeDeg()) / 2.0;
    double clat = (_min_deg.getLatitudeDeg() + _max_deg.getLatitudeDeg()) / 2.0;
    area_center = SGGeod::fromDegM( clon, clat, _average_elev_m );
    SG_LOG(SG_GENERAL, SG_DEBUG, "Central offset point = " << area_center);

    // Create the fitted surface
    SG_LOG(SG_GENERAL, SG_DEBUG, "ready to create fitted surface");
    fit();
    SG_LOG(SG_GENERAL, SG_DEBUG, "  fit process successful.");
}


tgSurface::~tgSurface() {
    delete Pts;
}


// Use a linear least squares method to fit a 3d polynomial to the
// sampled surface data
void tgSurface::fit() {

    // the fit function is:
    // f(x,y) = A1*x + A2*x*y + A3*y +
    //          A4*x*x + A5+x*x*y + A6*x*x*y*y + A7*y*y + A8*x*y*y +
    //          A9*x*x*x + A10*x*x*x*y + A11*x*x*x*y*y + A12*x*x*x*y*y*y +
    //            A13*y*y*y + A14*x*y*y*y + A15*x*x*y*y*y

    int nobs = Pts->cols() * Pts->rows();	// number of observations

    SG_LOG(SG_GENERAL, SG_DEBUG, "QR triangularisation" );

    // Create an array (matrix) with 16 columns (predictor values) A[n]
    TNT::Array2D<double> mat(nobs,16);

    // put all elevation values into a second array
    TNT::Array1D<double> zmat(nobs);

    // generate the required fit data
    for ( int j = 0; j < Pts->rows(); j++ ) {
        for ( int i = 0; i < Pts->cols(); i++ ) {
            SGGeod p = Pts->element( i, j );
            int index = ( j * Pts->cols() ) + i;
            double x = p.getLongitudeDeg() - area_center.getLongitudeDeg();
            double y = p.getLatitudeDeg() - area_center.getLatitudeDeg();
            double z = p.getElevationM() - area_center.getElevationM();

            zmat[index] = z;

            mat[index][0] = 1.0;
            mat[index][1] = x;
            mat[index][2] = x*y;
            mat[index][3] = y;
            mat[index][4] = x*x;
            mat[index][5] = x*x*y;
            mat[index][6] = x*x*y*y;
            mat[index][7] = y*y;
            mat[index][8] = x*y*y;
            mat[index][9] = x*x*x;
            mat[index][10] = x*x*x*y;
            mat[index][11] = x*x*x*y*y;
            mat[index][12] = x*x*x*y*y*y;
            mat[index][13] = y*y*y;
            mat[index][14] = x*y*y*y;
            mat[index][15] = x*x*y*y*y;
        }
    }

    // Do QR decompostion
    JAMA::QR<double> qr( mat );
    // find the least squares solution using the QR factors
    surface_coefficients = qr.solve(zmat);
}


// Query the elevation of a point, return -9999 if out of range
double tgSurface::query( SGGeod query ) const {

    // sanity check
    if ( !_aptBounds.isInside(query) )
    {
        SG_LOG(SG_GENERAL, SG_WARN, "Warning: Elevation query out of bounds for fitted surface!");
        return -9999.0;
    }

    // compute the function with solved coefficients

    // the fit function is:
    // f(x,y) = A1*x + A2*x*y + A3*y +
    //          A4*x*x + A5+x*x*y + A6*x*x*y*y + A7*y*y + A8*x*y*y +
    //          A9*x*x*x + A10*x*x*x*y + A11*x*x*x*y*y + A12*x*x*x*y*y*y +
    //            A13*y*y*y + A14*x*y*y*y + A15*x*x*y*y*y

    double x = query.getLongitudeDeg() - area_center.getLongitudeDeg();
    double y = query.getLatitudeDeg() - area_center.getLatitudeDeg();
    TNT::Array1D<double> A = surface_coefficients;

    double result = A[0] + A[1]*x + A[2]*x*y + A[3]*y + A[4]*x*x + A[5]*x*x*y
    + A[6]*x*x*y*y + A[7]*y*y + A[8]*x*y*y + A[9]*x*x*x + A[10]*x*x*x*y
    + A[11]*x*x*x*y*y + A[12]*x*x*x*y*y*y + A[13]*y*y*y + A[14]*x*y*y*y
    + A[15]*x*x*y*y*y;
    result += area_center.getElevationM();

    return result;
}

double tgSurface::calc_elevation( const SGGeod& node, double offset )
{
    double elev = query( node );
    elev += offset;

    return elev;
}

tgSurface::PointList tgSurface::calc_elevations( const PointList& geod_nodes, double offset )
{
    PointList result = geod_nodes;
    for ( unsigned int i = 0; i < result.size(); ++i ) {
        double elev = query( result[i] );
        result[i].setElevationM( elev + offset );
    }

    return result;
}

tgContour tgSurface::calc_elevations( const tgContour& geod_nodes, double offset )
{
    tgContour result = geod_nodes;
    for ( unsigned int i = 0; i < result.GetSize(); ++i ) {
        SGGeod node = result.GetNode(i);
        double elev = query( node );
        node.setElevationM( elev + offset );
        result.SetNode( i, node );
    }

    return result;
}

tgPolygon tgSurface::calc_elevations( const tgPolygon& poly, double offset )
{
    tgPolygon result;

    for ( unsigned int i = 0; i < poly.Contours(); ++i ) {
        tgContour contour = poly.GetContour( i );
        tgContour elevated = calc_elevations( contour, offset );

        result.AddContour( elevated );
    }

    return result;
}
