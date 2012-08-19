/**
 * \file point3d.hxx
 * A 3d point class (depricated).  This class is depricated and we are
 * in the process of removing all usage of it in favor of plib's "sg"
 * library of point, vector, and math routines.  Plib's sg lib is less
 * object oriented, but integrates more seamlessly with opengl.
 *
 * Adapted from algebra3 by Jean-Francois Doue, started October 1998.
 */

// Copyright (C) 1998  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id$


#ifndef _POINT3D_HXX
#define _POINT3D_HXX


#ifndef __cplusplus
# error This library requires C++
#endif

#include <simgear/compiler.h>

#include <stdio.h>
#include <cassert>
#include <cmath>
#include <istream>
#include <ostream>
#include <vector>

#include <simgear/math/SGMath.hxx>


//const double fgPoint3_Epsilon = 0.0000001;
const double fgPoint3_Epsilon = 0.000001;

enum {PX, PY, PZ};		    // axes

// Kludge for msvc++ 6.0 - requires forward decls of friend functions.
class Point3D;
std::istream& operator>> ( std::istream&, Point3D& );
std::ostream& operator<< ( std::ostream&, const Point3D& );
Point3D operator- (const Point3D& p);	            // -p1
bool operator== (const Point3D& a, const Point3D& b);  // p1 == p2?


/**
 * 3D Point class.
 */

class Point3D {

protected:

    double n[3];

public:

    /** Default constructor */
    Point3D();
    Point3D(const double x, const double y, const double z);
    explicit Point3D(const double d);
    Point3D(const Point3D &p);

    static Point3D fromSGGeod(const SGGeod& geod);
    static Point3D fromSGGeoc(const SGGeoc& geoc);
    static Point3D fromSGVec3(const SGVec3<double>& cart);
    static Point3D fromSGVec3(const SGVec3<float>& cart);
    static Point3D fromSGVec2(const SGVec2<double>& cart);
    static Point3D fromSGVec2(const SGVec2<float>& cart);

    // Assignment operators

    Point3D& operator = ( const Point3D& p );	 // assignment of a Point3D
    Point3D& operator += ( const Point3D& p );	 // incrementation by a Point3D
    Point3D& operator -= ( const Point3D& p );	 // decrementation by a Point3D
    Point3D& operator *= ( const double d );     // multiplication by a constant
    Point3D& operator /= ( const double d );	 // division by a constant

    bool operator > ( const Point3D& p );
    bool operator < ( const Point3D& p );
    
    void setx(const double x);
    void sety(const double y);
    void setz(const double z);
    void setlon(const double x);
    void setlat(const double y);
    void setradius(const double z);
    void setelev(const double z);

    void snap( double grid );

    // Queries 

    double& operator [] ( int i);		 // indexing
    double operator[] (int i) const;		 // read-only indexing

    inline const double *get_n() const { return n; };
    double x() const;      // cartesian x
    double y() const;      // cartesian y
    double z() const;      // cartesian z

    double lon() const;    // polar longitude
    double lat() const;    // polar latitude
    double radius() const; // polar radius
    double elev() const;   // geodetic elevation (if specifying a surface point)

    SGGeod toSGGeod(void) const;
    SGGeoc toSGGeoc(void) const;

    SGVec3d toSGVec3d(void) const;
    SGVec3f toSGVec3f(void) const;
    SGVec2f toSGVec2f(void) const;

    // friends
    friend Point3D operator - (const Point3D& p);	            // -p1
    friend bool operator == (const Point3D& a, const Point3D& b);  // p1 == p2?
    friend std::istream& operator>> ( std::istream&, Point3D& );
    friend std::ostream& operator<< ( std::ostream&, const Point3D& );

    // Special functions
    double distance3D(const Point3D& a) const;        // distance between
    double distance3Dsquared(const Point3D& a) const; // distance between ^ 2

    bool   IsEqual2D(const Point3D& a) const;         // equality check in X,Y only
    bool   HasElevation() const;                      // does point have elevation data?

    bool   IsWithin( Point3D min, Point3D max ) const;
    bool   IsWithin( double xmin, double xmax, double ymin, double ymax ) const;
    
#ifdef _MSC_VER
    double round(double d)
    {
        return floor(d + 0.5);
    }
#endif

};


// input from stream
inline std::istream&
operator >> ( std::istream& in, Point3D& p)
{
    char c;

    in >> p.n[PX];

    // read past optional comma
    while ( in.get(c) ) {
	if ( (c != ' ') && (c != ',') ) {
	    // push back on the stream
	    in.putback(c);
	    break;
	}
    }
	
    in >> p.n[PY];

    // read past optional comma
    while ( in.get(c) ) {
	if ( (c != ' ') && (c != ',') ) {
	    // push back on the stream
	    in.putback(c);
	    break;
	}
    }
	
    in >> p.n[PZ];

    return in;
}

inline std::ostream&
operator<< ( std::ostream& out, const Point3D& p )
{
    char buff[128];
    sprintf( buff, "(%3.10lf, %3.10lf, %3.10lf)", p.n[PX], p.n[PY], p.n[PZ]);
    return out << buff;
}

///////////////////////////
//
// Point3D Member functions
//
///////////////////////////

// CONSTRUCTORS

inline Point3D::Point3D()
{
   n[PX] = n[PY] = 0.0;
   n[PZ] = -9999.0;
}

inline Point3D::Point3D(const double x, const double y, const double z)
{
    n[PX] = x; n[PY] = y; n[PZ] = z;
}

inline Point3D::Point3D(const double d)
{
    n[PX] = n[PY] = n[PZ] = d;
}

inline Point3D::Point3D(const Point3D& p)
{
    n[PX] = p.n[PX]; n[PY] = p.n[PY]; n[PZ] = p.n[PZ];
}

inline Point3D Point3D::fromSGGeod(const SGGeod& geod)
{
  Point3D pt;
  pt.setlon(geod.getLongitudeRad());
  pt.setlat(geod.getLatitudeRad());
  pt.setelev(geod.getElevationM());

  return pt;
}

inline Point3D Point3D::fromSGGeoc(const SGGeoc& geoc)
{
  Point3D pt;
  pt.setlon(geoc.getLongitudeRad());
  pt.setlat(geoc.getLatitudeRad());
  pt.setradius(geoc.getRadiusM());

  return pt;
}

inline Point3D Point3D::fromSGVec3(const SGVec3<double>& cart)
{
  Point3D pt;
  pt.setx(cart.x());
  pt.sety(cart.y());
  pt.setz(cart.z());

  return pt;
}

inline Point3D Point3D::fromSGVec3(const SGVec3<float>& cart)
{
  Point3D pt;
  pt.setx(cart.x());
  pt.sety(cart.y());
  pt.setz(cart.z());

  return pt;
}

inline Point3D Point3D::fromSGVec2(const SGVec2<double>& cart)
{
  Point3D pt;
  pt.setx(cart.x());
  pt.sety(cart.y());
  pt.setz(0);

  return pt;
}

inline Point3D Point3D::fromSGVec2(const SGVec2<float>& cart)
{
  Point3D pt;
  pt.setx(cart.x());
  pt.sety(cart.y());
  pt.setz(0);

  return pt;
}


// ASSIGNMENT OPERATORS

inline Point3D& Point3D::operator = (const Point3D& p)
{
    n[PX] = p.n[PX]; n[PY] = p.n[PY]; n[PZ] = p.n[PZ]; 

    return *this;
}

inline Point3D& Point3D::operator += ( const Point3D& p )
{
    n[PX] += p.n[PX]; n[PY] += p.n[PY]; n[PZ] += p.n[PZ]; 
    
    return *this;
}

inline Point3D& Point3D::operator -= ( const Point3D& p )
{
    n[PX] -= p.n[PX]; n[PY] -= p.n[PY]; n[PZ] -= p.n[PZ]; 

    return *this;
}

inline Point3D& Point3D::operator *= ( const double d )
{
    n[PX] *= d; n[PY] *= d; n[PZ] *= d; 

    return *this;
}

inline Point3D& Point3D::operator /= ( const double d )
{
    double d_inv = 1./d; n[PX] *= d_inv; n[PY] *= d_inv; n[PZ] *= d_inv;

    return *this;
}

inline bool Point3D::operator < ( const Point3D& p )
{
    return ( (n[PX] < p.n[PX]) && (n[PY] < p.n[PY]) );
}

inline bool Point3D::operator > ( const Point3D& p )
{
    return ( (n[PX] > p.n[PX]) && (n[PY] > p.n[PY]) );
}

inline void Point3D::setx(const double x) {
    n[PX] = x;
}

inline void Point3D::sety(const double y) {
    n[PY] = y;
}

inline void Point3D::setz(const double z) {
    n[PZ] = z;
}

inline void Point3D::setlon(const double x) {
    n[PX] = x;
}

inline void Point3D::setlat(const double y) {
    n[PY] = y;
}

inline void Point3D::setradius(const double z) {
    n[PZ] = z;
}

inline void Point3D::setelev(const double z) {
    n[PZ] = z;
}

inline void Point3D::snap( double grid )
{
    n[PX] =  grid * round( n[PX]/grid );
    n[PY] =  grid * round( n[PY]/grid );
    n[PZ] =  grid * round( n[PZ]/grid );
}

// QUERIES

inline double& Point3D::operator [] ( int i)
{
    assert(! (i < PX || i > PZ));
    return n[i];
}

inline double Point3D::operator [] ( int i) const {
    assert(! (i < PX || i > PZ));
    return n[i];
}


inline double Point3D::x() const { return n[PX]; }

inline double Point3D::y() const { return n[PY]; }

inline double Point3D::z() const { return n[PZ]; }

inline double Point3D::lon() const { return n[PX]; }

inline double Point3D::lat() const { return n[PY]; }

inline double Point3D::radius() const { return n[PZ]; }

inline double Point3D::elev() const { return n[PZ]; }

inline SGGeod Point3D::toSGGeod(void) const
{
  SGGeod geod;
  geod.setLongitudeRad(lon());
  geod.setLatitudeRad(lat());
  geod.setElevationM(elev());
  return geod;
}

inline SGGeoc Point3D::toSGGeoc(void) const
{
  SGGeoc geoc;
  geoc.setLongitudeRad(lon());
  geoc.setLatitudeRad(lat());
  geoc.setRadiusM(radius());
  return geoc;
}

inline SGVec3d Point3D::toSGVec3d(void) const
{
  return SGVec3d(x(), y(), z());
}

inline SGVec3f Point3D::toSGVec3f(void) const
{
  return SGVec3f(x(), y(), z());
}

inline SGVec2f Point3D::toSGVec2f(void) const
{
  return SGVec2f(x(), y());
}

inline bool Point3D::IsEqual2D(const Point3D& a) const
{
    return
	fabs(a.n[PX] - n[PX]) < fgPoint3_Epsilon &&
	fabs(a.n[PY] - n[PY]) < fgPoint3_Epsilon;
}

inline bool Point3D::HasElevation() const
{
    return ( fabs( n[PZ] + 9999.0 ) > fgPoint3_Epsilon );
}

inline bool Point3D::IsWithin( Point3D min, Point3D max ) const
{
    return ( (min.n[PX] <= n[PX]) && (min.n[PY] <= n[PY]) &&
             (max.n[PX] >= n[PX]) && (max.n[PY] >= n[PY]) );
}

inline bool Point3D::IsWithin( double xmin, double xmax, double ymin, double ymax ) const
{
    return ( (xmin <= n[PX]) && (ymin <= n[PY]) &&
             (xmax >= n[PX]) && (ymax >= n[PY]) );
}

// FRIENDS

inline Point3D operator - (const Point3D& a)
{
    return Point3D(-a.n[PX],-a.n[PY],-a.n[PZ]);
}

inline Point3D operator + (const Point3D& a, const Point3D& b)
{
    return Point3D(a) += b;
}

inline Point3D operator - (const Point3D& a, const Point3D& b)
{
    return Point3D(a) -= b;
}

inline Point3D operator * (const Point3D& a, const double d)
{
    return Point3D(a) *= d;
}

inline Point3D operator * (const double d, const Point3D& a)
{
    Point3D pt = a*d;

    return pt;
}

inline Point3D operator / (const Point3D& a, const double d)
{
    Point3D pt = Point3D(a) *= (1.0 / d );

    return pt;
}

inline bool operator == (const Point3D& a, const Point3D& b)
{
    return
	fabs(a.n[PX] - b.n[PX]) < fgPoint3_Epsilon &&
	fabs(a.n[PY] - b.n[PY]) < fgPoint3_Epsilon &&
	fabs(a.n[PZ] - b.n[PZ]) < fgPoint3_Epsilon;
}

inline bool operator != (const Point3D& a, const Point3D& b)
{
    return !(a == b);
}

// Special functions

inline double
Point3D::distance3D(const Point3D& a ) const
{
    double x, y, z;

    x = n[PX] - a.n[PX];
    y = n[PY] - a.n[PY];
    z = n[PZ] - a.n[PZ];

    return sqrt(x*x + y*y + z*z);
}


inline double
Point3D::distance3Dsquared(const Point3D& a ) const
{
    double x, y, z;

    x = n[PX] - a.n[PX];
    y = n[PY] - a.n[PY];
    z = n[PZ] - a.n[PZ];

    return(x*x + y*y + z*z);
}


typedef std::vector< Point3D > point_list;

typedef point_list::iterator point_list_iterator;

typedef point_list::const_iterator const_point_list_iterator;

inline Point3D sgCartToGeod( const Point3D& p )
{
        SGGeod geod;
        SGGeodesy::SGCartToGeod(SGVec3d(p.x(), p.y(), p.z()), geod);
        return Point3D::fromSGGeod(geod);
}

inline Point3D sgGeodToCart(const Point3D& geod)
{
  SGVec3<double> cart;
  SGGeodesy::SGGeodToCart(SGGeod::fromRadM(geod.lon(), geod.lat(), geod.elev()), cart);
  return Point3D::fromSGVec3(cart);
}

#endif // _POINT3D_HXX


