// vpfbase.hxx - declaration of basic structures and classes for the library.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_VPFBASE_HXX
#define __VPF_VPFBASE_HXX 1

#include <string>


/**
 * A double-precision point in 3D space.
 */
struct VpfPoint
{
  VpfPoint ()
    : x(0), y(0), z(0)
  {}
  VpfPoint (double xin, double yin, double zin)
    : x(xin), y(yin), z(zin)
  {}
  double x;
  double y;
  double z;
};


/**
 * A rectangle.
 */
struct VpfRectangle
{
  double minX;
  double minY;
  double maxX;
  double maxY;
};


/**
 * A cross-tile id triplet.
 */
struct VpfCrossRef {
  int current_tile_key;
  int next_tile_id;
  int next_tile_key;
  int unused_key;
};


/**
 * Test whether a point is inside a rectangle.
 *
 * <p>On the edge counts as inside.</p>
 *
 * @param p The point to test.
 * @param r The rectangle to test.
 * @return true if the point is inside the rectangle or on its edge,
 * false otherwise.
 */
extern bool inside (const VpfPoint &p, const VpfRectangle &r);


/**
 * Test whether two rectangles overlap.
 *
 * <p>Coincident edges count as overlap.</p>
 *
 * @param r1 The first rectangle to test.
 * @param r2 The second rectangle to test.
 * @return true if the rectangles overlap, false otherwise.
 */
extern bool overlap (const VpfRectangle &r1, const VpfRectangle &r2);


/**
 * A VPF-related exception.
 */
class VpfException
{
public:
  VpfException ();
  VpfException (const std::string &message);
  virtual ~VpfException ();
  virtual const std::string &getMessage () const;
private:
  std::string _message;
};

#endif

// end of vpfbase.hxx
