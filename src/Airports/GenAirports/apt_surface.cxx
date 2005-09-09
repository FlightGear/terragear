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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
//

#include <simgear/compiler.h>

// libnewmat includes and defines
#define WANT_STREAM		// include.h will get stream fns
#define WANT_MATH		// include.h will get math fns
				// newmatap.h will get include.h
#include <newmat/newmatap.h>	// need matrix applications
#include <newmat/newmatio.h>	// need matrix output routines

#include <simgear/constants.h>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/math/sg_types.hxx>
#include <simgear/debug/logstream.hxx>

#include <Array/array.hxx>

#include "elevations.hxx"
#include "global.hxx"
#include "apt_surface.hxx"


static bool limit_slope( SimpleMatrix *Pts, int i1, int j1, int i2, int j2,
                         double average_elev_m )
{
    bool slope_error = false;

    Point3D p1, p2;
    p1 = Pts->element(i1,j1);
    p2 = Pts->element(i2,j2);

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
	      p1.setz( p2.z() - (dist * slope_max) );
            } else {
	      p1.setz( p2.z() + (dist * slope_max) );
            }
            Pts->set(i1, j1, p1);
        } else {
            // cout << "  p2 error larger" << endl;
            if ( slope > 0 ) {
	      p2.setz( p1.z() + (dist * slope_max) );
            } else {
	      p2.setz( p1.z() - (dist * slope_max) );
            }
            Pts->set(i2, j2, p2);
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

    // set an arbitrary minumum number of divisions to keep things
    // interesting
    if ( xdivs < 6 ) { xdivs = 6; }
    if ( ydivs < 6 ) { ydivs = 6; }
    SG_LOG(SG_GENERAL, SG_INFO, "  M(" << ydivs << "," << xdivs << ")");

    double dlon = x_deg / xdivs;
    double dlat = y_deg / ydivs;

    double dlon_h = dlon * 0.5;
    double dlat_h = dlat * 0.5;

    // Build the extra res input grid (shifted SW by half (dlon,dlat)
    // with an added major row column on the NE sides.)
    int mult = 10;
    SimpleMatrix dPts( (ydivs + 1) * mult + 1, (xdivs + 1) * mult + 1 );
    for ( int j = 0; j < dPts.cols(); ++j ) {
        for ( int i = 0; i < dPts.rows(); ++i ) {
	  dPts.set(i, j, Point3D( min_deg.lon() - dlon_h
				    + j * (dlon / (double)mult),
				   min_deg.lat() - dlat_h
				    + i * (dlat / (double)mult),
				   -9999 )
		   );
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
    Pts = new SimpleMatrix(ydivs + 1, xdivs + 1);
    double ave_divider = (mult+1) * (mult+1);
    for ( int j = 0; j < Pts->cols(); ++j ) {
        for ( int i = 0; i < Pts->rows(); ++i ) {
            SG_LOG(SG_GENERAL, SG_DEBUG, i << "," << j);
            double accum = 0.0;
            double lon_accum = 0.0;
            double lat_accum = 0.0;
            for ( int jj = 0; jj <= mult; ++jj ) {
                for ( int ii = 0; ii <= mult; ++ii ) {
                    double value = dPts.element(mult*i + ii, mult*j + jj).z();
                    SG_LOG( SG_GENERAL, SG_DEBUG, "value = " << value );
                    accum += value;
                    lon_accum += dPts.element(mult*i + ii, mult*j + jj).x();
                    lat_accum += dPts.element(mult*i + ii, mult*j + jj).y();
                }
            }
            double val_ave = accum / ave_divider;
            double lon_ave = lon_accum / ave_divider;
            double lat_ave = lat_accum / ave_divider;

            SG_LOG( SG_GENERAL, SG_DEBUG, "  val_ave = " << val_ave );
            Pts->set(i, j, Point3D( min_deg.lon() + j * dlon,
				   min_deg.lat() + i * dlat,
				   val_ave )
		    );
            SG_LOG( SG_GENERAL, SG_DEBUG, "lon_ave = " << lon_ave
                    << "  lat_ave = " << lat_ave );
            SG_LOG( SG_GENERAL, SG_DEBUG, "lon = " << min_deg.lon() + j * dlon
                    << "  lat = " << min_deg.lat() + i * dlat );
        }
    }

#ifdef DEBUG
    for ( int j = 0; j < Pts->cols(); ++j ) {
        for ( int i = 0; i < Pts->rows(); ++i ) {
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
        for ( int j = 0; j < Pts->cols() - 1; ++j ) {
            for ( int i = 0; i < Pts->rows() - 1; ++i ) {
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
    for ( int j = 0; j < Pts->cols(); ++j ) {
        for ( int i = 0; i < Pts->rows(); ++i ) {
            printf("%.5f %.5f %.1f\n", Pts(i,j).x(), Pts(i,j).y(),
                   Pts(i,j).z() );
        }
    }
#endif

    // Create the fitted surface
    SG_LOG(SG_GENERAL, SG_DEBUG, "ready to create fitted surface");
    fit();

    // sanity check: I'm finding that leastSquares() can produce nan
    // surfaces.  We test for this and fall back to globalInterp() if
    // the least squares fails.
    double result =  query( (min_deg.lon() + max_deg.lon()) / 2.0,
			    (min_deg.lat() + max_deg.lat()) / 2.0 );
    if ( result > -9000.0 ) {
        // ok, a valid number
    } else {
        SG_LOG(SG_GENERAL, SG_WARN,
               "leastSquares() fit seemed to fail!!!");
        char command[256];
        sprintf( command,
                 "echo least squares nurbs interpolation failed, using globalInterp() >> last_apt" );
        system( command );

	// abort and force developer to debug for now
	exit(-1);
    }
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
    delete Pts;
}


static ColumnVector qr_method( Real* y,
			       Real* t1, Real* t2, Real* t3, Real* t4,
			       Real* t5, Real* t6, Real* t7, Real* t8,
			       int nobs, int npred )
{
  cout << "QR triangularisation" << endl;;

  // QR triangularisation method
 
  // load data - 1s into col 1 of matrix
  int npred1 = npred+1;
  Matrix X(nobs,npred1); ColumnVector Y(nobs);
  X.column(1) = 1.0;
  X.column(2) << t1;
  X.column(3) << t2;
  X.column(4) << t3;
  X.column(5) << t4;
  X.column(6) << t5;
  X.column(7) << t6;
  X.column(8) << t7;
  X.column(9) << t8;
  Y << y;

  // do Householder triangularisation
  // no need to deal with constant term separately
  Matrix X1 = X;                 // Want copy of matrix
  ColumnVector Y1 = Y;
  UpperTriangularMatrix U; ColumnVector M;
  QRZ(X1, U); QRZ(X1, Y1, M);    // Y1 now contains resids
  ColumnVector A = U.i() * M;
  ColumnVector Fitted = X * A;
  Real ResVar = sum_square(Y1) / (nobs-npred1);

  // get variances of estimates
  U = U.i(); DiagonalMatrix D; D << U * U.t();

  // Get diagonals of Hat matrix
  DiagonalMatrix Hat;  Hat << X1 * X1.t();

  // print out answers
  cout << "\nEstimates and their standard errors\n\n";
  ColumnVector SE(npred1);
  for (int i=1; i<=npred1; i++) SE(i) = sqrt(D(i)*ResVar);
  cout << setw(11) << setprecision(5) << (A | SE) << endl;
  cout << "\nObservations, fitted value, residual value, hat value\n";
  cout << setw(9) << setprecision(3) << 
    (X.columns(2,3) | Y | Fitted | Y1 | Hat.as_column());
  cout << "\n\n";

  return A;
}


// Use a linear least squares method to fit a 3d polynomial to the
// sampled surface data
void TGAptSurface::fit() {

  // the fit function is: f(x,y) = a*x + b*x^2 + c*y + d*y^2 + e*x*y +
  // f*x*y^2 + g*x^2*y + h*x^2*y^2
  int nobs = Pts->cols() * Pts->rows();	// number of observations
  int npred = 8;		        // number of predictor values

  Real z[nobs];
  Real t1[nobs];
  Real t2[nobs];
  Real t3[nobs];
  Real t4[nobs];
  Real t5[nobs];
  Real t6[nobs];
  Real t7[nobs];
  Real t8[nobs];

  // generate the required fit data
  for ( int j = 0; j < Pts->cols(); j++ ) {
    for ( int i = 0; i <= Pts->rows(); i++ ) {
      Point3D p = Pts->element( i, j );
      int index = ( j * Pts->rows() ) + i;
      Real xi = p.x();
      Real yi = p.y();
      z[index] = p.z();
      t1[index] = xi;
      t2[index] = xi*xi;
      t3[index] = yi;
      t4[index] = yi*yi;
      t5[index] = xi*yi;
      t6[index] = xi*yi*yi;
      t7[index] = xi*xi*yi;
      t8[index] = xi*xi*yi*yi;
    }
  }

  // we want to find the values of a,b,c to give the best
  // fit

  Try {
    surface_coefficients
      = qr_method(z, t1, t2, t3, t4, t5, t6, t7, t8, nobs, npred);
  }
  CatchAll { cout << BaseException::what(); }

}


// Query the elevation of a point, return -9999 if out of range
double TGAptSurface::query( double lon_deg, double lat_deg ) {
    // sanity check
    if ( lon_deg < min_deg.lon() || lon_deg > max_deg.lon() ||
         lat_deg < min_deg.lat() || lat_deg > max_deg.lat() )
    {
        SG_LOG(SG_GENERAL, SG_WARN,
	       "Warning: query out of bounds for fitted surface!");
        return -9999.0;
    }

    // compute the function with solved coefficients

    // the fit function is: f(x,y) = a*x + b*x^2 + c*y + d*y^2 + e*x*y +
    //                               f*x*y^2 + g*x^2*y + h*x^2*y^2

    double x = lon_deg;
    double y = lat_deg;
    ColumnVector A = surface_coefficients;
    double result = A(0)*x + A(1)*x*x + A(2)*y + A(3)*y*y + A(4)*x*y
      + A(5)*x*y*y + A(6)*x*x*y + A(7)*x*x*y*y;

    return result;
}
