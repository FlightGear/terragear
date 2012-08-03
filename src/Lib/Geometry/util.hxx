// util.hxx - a collection of simple WGS84 utility functions.
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#ifndef __UTIL_HXX
#define __UTIL_HXX 1

#ifndef __cplusplus                                                          
# error This library requires C++
#endif  

#include <simgear/compiler.h>
#include <Geometry/point3d.hxx>

#include <string>

#include <Polygon/polygon.hxx>
#include <Polygon/superpoly.hxx>
#include <Polygon/texparams.hxx>

#include "line.hxx"

namespace tg {


/**
 * Inline function to clamp an angle between 0 and 360 degrees.
 *
 * @param a The angle to clamp, in degrees.
 * @return The clamped angle.
 */
inline double 
CLAMP_ANGLE (double a)
{
  while (a < 0.0)
    a += 360.0;
  while (a >= 360.0)
    a -= 360.0;
  return a;
}


/**
 * Calculate the intersection of two line segments.
 *
 * @param p0 First point on the first segment.
 * @param p1 A second point on the second segment.
 * @param p2 First point on the second segment.
 * @param p3 A second point on the second segment.
 * @param intersection A variable to hold the calculated intersection.
 * @return true if there was an intersection, false if the segments.
 *         are parallel or coincident.
 */
bool getIntersection (const Point3D &p0, const Point3D &p1,
		      const Point3D &p2, const Point3D &p3,
		      Point3D &intersection);


/**
 * Create a polygon out of a point.
 *
 * Note that simple geometry doesn't work here, because the scale is
 * not even -- the points on the x-axis (longitude) become closer and
 * closer as the y-axis (latitude) approaches the poles, meeting in
 * a single point at y=90 and y=-90.  As a result, this function
 * uses the WGS80 functions, rather than simple Pythagorean stuff.
 *
 * @param p The point at the centre of the new polygon.
 * @param width The width in standard units (meters for FlightGear).
 * @param polygon The object that will hold the new polygon.
 */
void makePolygon (const Point3D &p, double width, TGPolygon &polygon);


/**
 * Create a polygon out of a line.
 *
 * Note that simple geometry doesn't work here, because the scale is
 * not even -- the points on the x-axis (longitude) become closer and
 * closer as the y-axis (latitude) approaches the poles, meeting in
 * a single point at y=90 and y=-90.  As a result, this function
 * uses the WGS80 functions, rather than simple Pythagorean stuff.
 *
 * Update CLO 7 Mar 2003: Function rewritten to carefully preserve
 * specifed polygon width through any kind of point to point angle
 * change.
 *
 * @param line The multi-segment line inside the new polygon.
 * @param width The width in standard units (meters for FlightGear).
 * @param polygon The object that will hold the new polygon.
 */
void makePolygon (const Line &line, double width, TGPolygon &polygon);

void makePolygons   (const Line &line, double width, poly_list &segments);
void makePolygonsTP (const Line &line, double width, poly_list &segments, texparams_list &tps);

/**
 * Make a 2D bounding rectangle for a polygon.
 *
 * @param polygon The polygon to make the rectangle for.
 * @return The bounding rectangle.
 */
Rectangle makeBounds (const TGPolygon &polygon);


/**
 * Parse a chunk string (like "w080n40") into a rectangle.
 * Defaults to 10 x 10 degrees.
 *
 * @param s The string.
 * @return A rectangle containing the bounds.
 */
Rectangle parseChunk (const std::string &s, double delta);


/**
 * Parse a tile string (like "w080n41") into a 1x1 degree rectangle.
 *
 * @param s The string.
 * @return A rectangle containing the bounds.
 */
Rectangle parseTile (const std::string &s);


};  // namespace tg

#endif // __UTIL_HXX
