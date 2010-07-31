// line.hxx - a simple multi-segment line class.
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#ifndef __LINE_HXX
#define __LINE_HXX

#ifndef __cplusplus                                                          
# error This library requires C++
#endif  

#include <simgear/compiler.h>
#include <Geometry/point3d.hxx>

#include <vector>

#include "rectangle.hxx"

namespace tg {

/**
 * A simple multi-segment line class.
 *
 * A line segment is a growable list of points.  I will add more 
 * functionality if or when it is needed.
 */
class Line
{
public:
  /**
   * Create a new line with no points.
   */
  Line ();

  /**
   * Copy an existing line.
   *
   * @param l The line to copy.
   */
  Line (const Line &l);

  /**
   * Destructor.
   */
  virtual ~Line ();

  /**
   * Get the number of points currently in the line.
   *
   * @return The point count.
   */
  virtual int getPointCount () const;

  /**
   * Get a point in the line (const).
   *
   * @param index The index of the point, zero-based.
   * @return The point at the index specified.
   */
  virtual const Point3D &getPoint (int index) const;

  /**
   * Get a point in the line (non-const).
   *
   * @param index The index of the point, zero-based.
   * @return The point at the index specified.
   */
  virtual Point3D &getPoint (int index);

  /**
   * Add a new point to the end of the line.
   *
   * @param point The point to add.
   */
  virtual void addPoint (const Point3D &point);

  /**
   * Get the bounding rectangle for the line.
   *
   * @return The bounding rectangle.
   */
  virtual Rectangle getBounds () const;

private:
  std::vector<Point3D> _points;
};

};

#endif // __LINE_HXX
