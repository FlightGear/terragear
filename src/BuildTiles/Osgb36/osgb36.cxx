// osgb36.cxx -- Routines to convert UK OS coordinates to and
//               from world WGS84 coordinates.
//
// Written by David Luff, started December 2000.
//
// Copyright (C) 2000  David C. Luff  - david.luff@nottingham.ac.uk
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

#include <simgear/compiler.h>

#include STL_IOSTREAM
#include <math.h>

#include "osgb36.hxx"

SG_USING_STD(cout);

double DEG_TO_RAD = 2.0 * 3.14159265358979323846264338327950288419716939967511 / 360.0;
double RAD_TO_DEG = 360.0 / (2.0 * 3.14159265358979323846264338327950288419716939967511);

/********************************************************************
*
*        FORWARD DECLARATIONS
*
********************************************************************/
//*******************************************************************
//
// NOTE All functions which take or return Latitude and Longitude
//	take and return degrees.  Conversion to and from radians
//	is performed internaly.  XYZ coordinates and ellipsoid
//	heights are returned and required in meters.
//
//*******************************************************************

//Convert OSGB36 Lat and Lon to OSGB36 Eastings and Northings (Grid Reference)
Point3D ConvertLatLonToEastingsNorthings(Point3D LatLon);

//Convert OSGB36 Eastings and Northings to OSGB36 Lat and Lon
Point3D ConvertEastingsNorthingsToLatLon(Point3D GridRef);

//Convert OSGB36 coordinates from polar to cartesian (x,y,z) form
Point3D ConvertAiry1830PolarToCartesian(Point3D LatLon);

//Convert WGS84 coordinates from polar to cartesian (x,y,z) form
Point3D ConvertGRS80PolarToCartesian(Point3D LatLon);

//Convert OSGB36 coordinates from cartesian (x,y,z) to polar form
Point3D ConvertAiry1830CartesianToPolar(Point3D XYZCoord);

//Convert WGS84 coordinates from cartesian (x,y,z) to polar form
Point3D ConvertGRS80CartesianToPolar(Point3D XYZCoord);

//Transform a point in WGS84 cartesian coordinates to OSGB36 cartesian coordinates
//Uses the Helmert Transformation with coefficients from www.gps.gov.org
//Only accurate to around 5m since OSGB36 is an inhomogenous TRF
Point3D ConvertWGS84ToOSGB36(Point3D WGS84);

//Transform a point in OSGB36 cartesian coordinates to WGS84 cartesian coordinates
//Uses the Helmert Transformation with coefficients from www.gps.gov.org
//Only accurate to around 5m since OSGB36 is an inhomogenous TRF
Point3D ConvertOSGB36ToWGS84(Point3D OSGB36);

/****************************************************************************/


// Start of implementation proper.

// Convert WGS84 Lat/Lon to OSGB36 Eastings and Northings
Point3D WGS84ToOSGB36(Point3D p) {
    p = ConvertGRS80PolarToCartesian(p);
    p = ConvertWGS84ToOSGB36(p);
    p = ConvertAiry1830CartesianToPolar(p);
    p = ConvertLatLonToEastingsNorthings(p);
    return(p);
}

// Convert OSGB36 Eastings and Northings to WGS84 Lat/Lon
Point3D OSGB36ToWGS84(Point3D p) {
    p = ConvertEastingsNorthingsToLatLon(p);
    p = ConvertAiry1830PolarToCartesian(p);
    p = ConvertOSGB36ToWGS84(p);
    p = ConvertGRS80CartesianToPolar(p);
    return(p);
}

//Convert OSGB36 Lat and Lon to OSGB36 Eastings and Northings (Grid Reference)
Point3D ConvertLatLonToEastingsNorthings(Point3D LatLon)
{	
    // ellipsoid constants
    double a;	//semi-major axis (m)
    double b;	//semi-minor axis (m)
    double e_squared;	//ellipsoid squared eccentricity constant
    
    // Airy 1830
    a = 6377563.396;
    b = 6356256.910;
    
    // GRS80 (aka WGS84)
    //	a = 6378137.000;
    //	b = 6356752.3141;
    
    e_squared = ((a*a)-(b*b))/(a*a);
    
    // Projection constants
    double N_zero;		//northing of true origin (m)
    double E_zero;		//easting of true origin (m)
    double F_zero;		//scale factor on central meridian
    double lat_zero;	//latitude of true origin (rad)
    double lon_zero;	//longitude of true origin and central meridian (rad)
    
    // OSGB36 (UK National Grid)
    N_zero = -100000;
    E_zero = 400000;
    F_zero = 0.9996012717;
    lat_zero = 49 * DEG_TO_RAD;
    lon_zero = -2 * DEG_TO_RAD;
    
    double n;
    double v;
    double rho;
    double neta_squared;
    double M;
    double I;
    double II;
    double III;
    double IIIA;
    double IV;
    double V;
    double VI;
    
    double lon = LatLon.lon() * DEG_TO_RAD;
    double lat = LatLon.lat() * DEG_TO_RAD;
    
    n = (a-b)/(a+b);
    v = a*F_zero*pow( (1-e_squared*sin(lat)*sin(lat)) , -0.5 );
    rho = a*F_zero*(1-e_squared)*pow((1-e_squared*sin(lat)*sin(lat)) , -1.5 );
    neta_squared = v/rho - 1.0;
    
    // terms in the M equation to make it more manageable (eqn A11)
    double term1;
    double term2;
    double term3;
    double term4;
    
    term1 = (1.0 + n + (5.0/4.0)*n*n + (5.0/4.0)*n*n*n)*(lat-lat_zero);
    term2 = ((3.0*n) + (3.0*n*n) + ((21.0/8.0)*n*n*n))*(sin(lat-lat_zero))*(cos(lat+lat_zero));
    term3 = (((15.0/8.0)*n*n) + ((15.0/8.0)*n*n*n))*(sin(2.0*(lat-lat_zero)))*(cos(2.0*(lat+lat_zero)));
    term4 = ((35.0/24.0)*n*n*n)*(sin(3.0*(lat-lat_zero)))*(cos(3.0*(lat+lat_zero)));
    
    M = b * F_zero * (term1 - term2 + term3 - term4);	//Eqn A11
    
    I = M + N_zero;
    II = (v/2.0)*sin(lat)*cos(lat);
    III = (v/24.0)*sin(lat)*pow(cos(lat),3)*(5.0 - pow(tan(lat),2) + 9.0*neta_squared);
    IIIA = (v/720.0)*sin(lat)*pow(cos(lat),5)*(61.0 - 58.0*pow(tan(lat),2) + pow(tan(lat),4));
    IV = v*cos(lat);
    V = (v/6.0)*pow(cos(lat),3)*(v/rho - pow(tan(lat),2));
    VI = (v/120.0)*pow(cos(lat),5)*(5.0 - 18.0*pow(tan(lat),2) + pow(tan(lat),4) + 14.0*neta_squared - 58.0*pow(tan(lat),2)*neta_squared);
    
    double N;	//calculated Northing
    double E;	//calculated Easting
    
    N = I + II*(lon-lon_zero)*(lon-lon_zero) + III*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero) + IIIA*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero);  //eqn A12
    E = E_zero + IV*(lon-lon_zero) + V*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero) + VI*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero)*(lon-lon_zero);	//eqn A13
    
    Point3D OSref;
    
    OSref.setx(E);
    OSref.sety(N);
    OSref.setz(0.0);	// Not used at present but could be used to return ODN height in future
    
    return(OSref);
}


Point3D ConvertEastingsNorthingsToLatLon(Point3D GridRef)
{	
    // ellipsoid constants
    double a;	//semi-major axis (m)
    double b;	//semi-minor axis (m)
    double e_squared;	//ellipsoid squared eccentricity constant
    
    // Airy 1830
    a = 6377563.396;
    b = 6356256.910;
    
    // GRS80 (aka WGS84)
    //	a = 6378137.000;
    //	b = 6356752.3141;
    
    e_squared = ((a*a)-(b*b))/(a*a);
    
    // Projection constants
    double N_zero;		//northing of true origin (m)
    double E_zero;		//easting of true origin (m)
    double F_zero;		//scale factor on central meridian
    double lat_zero;	//latitude of true origin (rad)
    double lon_zero;	//longitude of true origin and central meridian (rad)
    
    // OSGB36 (UK National Grid)
    N_zero = -100000;
    E_zero = 400000;
    F_zero = 0.9996012717;
    lat_zero = 49 * DEG_TO_RAD;
    lon_zero = -2 * DEG_TO_RAD;
    
    double n;
    double M;
    
    n = (a-b)/(a+b);
    
    double E = GridRef.x();
    double N = GridRef.y();
    
    // terms in the M equation to make it more manageable (eqn A11)
    double term1;
    double term2;
    double term3;
    double term4;
    
    double lat_prime;
    
    //calculate initial lat_prime
    lat_prime = ((N-N_zero)/(a*F_zero)) + lat_zero;
    
    int i;
    
    for(i=0;i<100;i++)
    {
	
	
	
	term1 = (1.0 + n + ((5.0/4.0)*n*n) + ((5.0/4.0)*n*n*n))*(lat_prime-lat_zero);
	term2 = ((3.0*n) + (3.0*n*n) + ((21.0/8.0)*n*n*n))*(sin(lat_prime-lat_zero))*(cos(lat_prime+lat_zero));
	term3 = (((15.0/8.0)*n*n) + ((15.0/8.0)*n*n*n))*(sin(2.0*(lat_prime-lat_zero)))*(cos(2.0*(lat_prime+lat_zero)));
	term4 = ((35.0/24.0)*n*n*n)*(sin(3.0*(lat_prime-lat_zero)))*(cos(3.0*(lat_prime+lat_zero)));
	
	M = b * F_zero * (term1 - term2 + term3 - term4);	//Eqn A11
	
	if((N - N_zero - M) < 0.001)
	    break;	    //N - N_zero - M is less than 1mm
	else	//recompute lat_prime and keep iterating until it is
	    lat_prime += ((N - N_zero - M)/(a*F_zero));
	
	if(i == 99)
	    cout << "WARNING: Iteration failed to converge in ConvertEastingsNorthingsToLatLon\n";
    }
    
    double v;
    double rho;
    double neta_squared;
    double VII;
    double VIII;
    double IX;
    double X;
    double XI;
    double XII;
    double XIIA;
    
    v = a*F_zero*pow( (1-e_squared*sin(lat_prime)*sin(lat_prime)) , -0.5 );
    rho = a*F_zero*(1-e_squared)*pow((1-e_squared*sin(lat_prime)*sin(lat_prime)) , -1.5 );
    neta_squared = v/rho - 1.0;
    
    VII = tan(lat_prime) / (2.0*rho*v);
    VIII = (tan(lat_prime)/(24.0*rho*v*v*v)) * (5.0 + 3.0*pow(tan(lat_prime),2.0) + neta_squared - 9.0*pow(tan(lat_prime),2.0)*neta_squared);
    IX = (tan(lat_prime)/(720.0*rho*v*v*v*v*v)) * (61.0 + 90.0*pow(tan(lat_prime),2.0) + 45.0*pow(tan(lat_prime),4.0));
    X = (1/cos(lat_prime)) / v;
    XI = ((1/cos(lat_prime)) / (6.0 * pow(v,3))) * (v/rho + 2*pow(tan(lat_prime),2));
    XII = ((1/cos(lat_prime)) / (120.0 * pow(v,5))) * (5.0 + 28.0*pow(tan(lat_prime),2) + 24.0*pow(tan(lat_prime),4));
    XIIA = ((1/cos(lat_prime)) / (5040.0 * pow(v,7))) * (61.0 + 662.0*pow(tan(lat_prime),2) + 1320.0*pow(tan(lat_prime),4) + 720.0*pow(tan(lat_prime),6));
    
    double lat;
    double lon;
    
    lat = lat_prime - VII*pow((E-E_zero),2) + VIII*pow((E-E_zero),4) + IX*pow((E-E_zero),6);
    lon = lon_zero + X*(E-E_zero) - XI*pow((E-E_zero),3) + XII*pow((E-E_zero),5) - XIIA*pow((E-E_zero),7);
    
    Point3D LatLon;
    LatLon.setlon(lon * RAD_TO_DEG);
    LatLon.setlat(lat * RAD_TO_DEG);
    LatLon.setelev(0.0);
    
    return(LatLon);
}


Point3D ConvertAiry1830PolarToCartesian(Point3D LatLon)
{
    
    double lon = LatLon.lon() * DEG_TO_RAD;
    double lat = LatLon.lat() * DEG_TO_RAD;
    double H = LatLon.elev() * DEG_TO_RAD;
    
    // ellipsoid constants
    double a;	//semi-major axis (m)
    double b;	//semi-minor axis (m)
    double e_squared;	//ellipsoid squared eccentricity constant
    
    // Airy 1830
    a = 6377563.396;
    b = 6356256.910;
    
    e_squared = ((a*a)-(b*b))/(a*a);
    
    double v;
    
    v = a*pow( (1-e_squared*sin(lat)*sin(lat)) , -0.5 );
    
    double x;
    double y;
    double z;
    Point3D Cartesian;

    x = (v+H)*cos(lat)*cos(lon);
    y = (v+H)*cos(lat)*sin(lon);
    z = ((1-e_squared)*v + H)*sin(lat);

    Cartesian.setx(x);
    Cartesian.sety(y);
    Cartesian.setz(z);

    return(Cartesian);
}


Point3D ConvertGRS80PolarToCartesian(Point3D LatLon)
{
    
    double lon = LatLon.lon() * DEG_TO_RAD;
    double lat = LatLon.lat() * DEG_TO_RAD;
    double H = LatLon.elev() * DEG_TO_RAD;
    
    // ellipsoid constants
    double a;	//semi-major axis (m)
    double b;	//semi-minor axis (m)
    double e_squared;	//ellipsoid squared eccentricity constant
    
    
    // GRS80 (aka WGS84)
    a = 6378137.000;
    b = 6356752.3141;
    
    e_squared = ((a*a)-(b*b))/(a*a);
    
    double v;
    
    v = a*pow( (1-e_squared*sin(lat)*sin(lat)) , -0.5 );
    
    double x;
    double y;
    double z;
    Point3D Cartesian;

    x = (v+H)*cos(lat)*cos(lon);
    y = (v+H)*cos(lat)*sin(lon);
    z = ((1-e_squared)*v + H)*sin(lat);

    Cartesian.setx(x);
    Cartesian.sety(y);
    Cartesian.setz(z);

    return(Cartesian);
}


Point3D ConvertAiry1830CartesianToPolar(Point3D XYZCoord)
{
    double x = XYZCoord.x();
    double y = XYZCoord.y();
    double z = XYZCoord.z();
    double p;
    double lat1;
    double lat2;
    double lat;
    double lon;
    double H;
    Point3D LatLon;

    // ellipsoid constants
    double a;	//semi-major axis (m)
    double b;	//semi-minor axis (m)
    double e_squared;	//ellipsoid squared eccentricity constant
    
    // Airy 1830
    a = 6377563.396;
    b = 6356256.910;
    
    e_squared = ((a*a)-(b*b))/(a*a);
    
    double v;
    
    lon = atan(y/x);

    p = sqrt(x*x + y*y);

    //lat is obtained iteratively
    //obtain initial value of lat
    lat1 = atan(z/(p*(1-e_squared)));

//    cout << "Initial value of lat = " << lat1 << '\n';

    int i;

    for(i=0;i<100;i++)
    {
	v = a*pow( (1-e_squared*sin(lat1)*sin(lat1)) , -0.5 );

	lat2 = atan((z + e_squared*v*sin(lat1))/p);

//	cout << "lat2 = " << lat2 << '\n';

	if(fabs(lat2-lat1) < 0.00000001)
	{
	    lat = lat2;
//	    cout << i << " iterations required in ConvertAiry1830CartesianToPolar\n";
	    break;
	}
	else
	{
	    lat1 = lat2;
	}

	if(i == 99)
	    cout << "WARNING: Iteration failed to converge in ConvertAiry1830CartesianToPolar\n";
    }

    H = p/cos(lat) - v;

    LatLon.setlon(lon * RAD_TO_DEG);
    LatLon.setlat(lat * RAD_TO_DEG);
    LatLon.setelev(H);

    return(LatLon);
}


Point3D ConvertGRS80CartesianToPolar(Point3D XYZCoord)
{
    double x = XYZCoord.x();
    double y = XYZCoord.y();
    double z = XYZCoord.z();
    double p;
    double lat1;
    double lat2;
    double lat;
    double lon;
    double H;
    Point3D LatLon;

    // ellipsoid constants
    double a;	//semi-major axis (m)
    double b;	//semi-minor axis (m)
    double e_squared;	//ellipsoid squared eccentricity constant
    
    // GRS80 (aka WGS84)
    a = 6378137.000;
    b = 6356752.3141;
    
    e_squared = ((a*a)-(b*b))/(a*a);
    
    double v;
    
    lon = atan(y/x);

    p = sqrt(x*x + y*y);

    //lat is obtained iteratively
    //obtain initial value of lat
    lat1 = atan(z/(p*(1-e_squared)));

    int i;

    for(i=0;i<100;i++)
    {
	v = a*pow( (1-e_squared*sin(lat1)*sin(lat1)) , -0.5 );

	lat2 = atan((z + e_squared*v*sin(lat1))/p);

	if(fabs(lat2-lat1) < 0.00000001)
	{
	    lat = lat2;
//	    cout << i << " iterations required in ConvertGRS80CartesianToPolar\n";
	    break;
	}
	else
	{
	    lat1 = lat2;
	}

	if(i == 99)
	    cout << "WARNING: Iteration failed to converge in ConvertGRS80CartesianToPolar\n";
    }

    H = p/cos(lat) - v;

    LatLon.setlon(lon * RAD_TO_DEG);
    LatLon.setlat(lat * RAD_TO_DEG);
    LatLon.setelev(H);

    return(LatLon);
}



//Transform a point in WGS84 cartesian coordinates to OSGB36 cartesian coordinates
//Uses the Helmert Transformation with coefficients from www.gps.gov.org
//Only accurate to around 5m since OSGB36 is an imhomogenous TRF
Point3D ConvertWGS84ToOSGB36(Point3D WGS84)
{
    Point3D OSGB36;

    double tx = -446.448;   // (m)
    double ty = 125.157;
    double tz = -542.060;
    double rx = -0.1502;    // (sec)
    double ry = -0.2470;
    double rz = -0.8421;
    double s = 20.4894;	    // scale factor in ppm

    // Convert rotations from seconds of arc to radians
    rx = rx / 3600.0 * DEG_TO_RAD;
    ry = ry / 3600.0 * DEG_TO_RAD;
    rz = rz / 3600.0 * DEG_TO_RAD;

    s /= 1000000;

//    cout << "(1+s) = " << (1+s) << '\n';

    OSGB36.setx(tx + (1+s)*WGS84.x() - rz*WGS84.y() + ry*WGS84.z());
    OSGB36.sety(ty + rz*WGS84.x() + (1+s)*WGS84.y() - rx*WGS84.z());
    OSGB36.setz(tz - ry*WGS84.x() + rx*WGS84.y() + (1+s)*WGS84.z());

    return(OSGB36);
}


//Reverse transformation achieved by reversing the signs of coefficients for the above.
//Only valid for small angles, and has same accuracy limitations
Point3D ConvertOSGB36ToWGS84(Point3D OSGB36)
{
    Point3D WGS84;

    double tx = 446.448;   // (m)
    double ty = -125.157;
    double tz = 542.060;
    double rx = 0.1502;    // (sec)
    double ry = 0.2470;
    double rz = 0.8421;
    double s = -20.4894;	    // scale factor in ppm

    // Convert rotations from seconds to radians
    rx = rx / 3600.0 * DEG_TO_RAD;
    ry = ry / 3600.0 * DEG_TO_RAD;
    rz = rz / 3600.0 * DEG_TO_RAD;

    s /= 1000000;

    WGS84.setx(tx + (1+s)*OSGB36.x() - rz*OSGB36.y() + ry*OSGB36.z());
    WGS84.sety(ty + rz*OSGB36.x() + (1+s)*OSGB36.y() - rx*OSGB36.z());
    WGS84.setz(tz - ry*OSGB36.x() + rx*OSGB36.y() + (1+s)*OSGB36.z());

    return(WGS84);
}







