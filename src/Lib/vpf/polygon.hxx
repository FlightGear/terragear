// polygon.hxx - declaration of VpfPolygon
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_POLYGON_HXX
#define __VPF_POLYGON_HXX 1

#include "vpfbase.hxx"

#include <string>

class VpfTable;
class VpfContour;


/**
 * Polygon topology.
 *
 * <p>This class represents an actual polygon, a collection of lines
 * each of which completely encloses an area.  Each of the lines is
 * called a contour: the first contour is always the outside ring of
 * the polygon, and the remaining contours always represent holes inside
 * it (such as a park in a city or an island in a lake).</p>
 *
 * <p>The implementation is lazy: getting a polygon from a feature doesn't
 * actually cause the (very large) FBR, RNG, or FAC tables to be read
 * until you actually need them.</p>
 *
 * <p>In VPF terminology, a polygon is called a "face", abbreviated
 * "fac".</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision: 1.1 $
 */
class VpfPolygon : public VpfComponent
{
public:


  /**
   * Copy constructor.
   *
   * @param polygon The polygon to copy.
   */
  VpfPolygon (const VpfPolygon &polygon);


  /**
   * Destructor.
   */
  virtual ~VpfPolygon ();


  /**
   * Get the minimum bounding rectangle for the polygon.
   *
   * <p>The bounding rectangle is useful for determining whether any
   * of the polygon falls inside a region of interest.</p>
   *
   * @return A rectangle containing the points for the top-left and
   * bottom-right corners of the smallest rectangle that can fit around
   * the polygon.
   */
  virtual const VpfRectangle getBoundingRectangle () const;


  /**
   * Count the contours in the polygon.
   *
   * <p>Every polygon contains at least one contour for its outer
   * boundary (the contour is a line where the last point joins with
   * the first to enclose an area).  Any additional contours are holes
   * inside the outer boundary.</p>
   *
   * @return The number of contours in the polygon (at least one).
   * @see #getContour
   */
  virtual int getContourCount () const;


  /**
   * Get a contour from the polygon.
   *
   * @param index The zero-based index of the contour to retrieve.
   * @return The contour as a line.
   * @exception VpfException If the index is out of range.
   * @see #getContourCount
   */
  virtual const VpfContour getContour (int index) const;


protected:

  friend class VpfTileRef;
  friend class VpfFeature;


  /**
   * Protected constructor.
   *
   * <p>This is the only mechanism for creating a new polygon from
   * scratch.  Library users will obtain a polygon from the VpfFeature
   * class rather than constructing one directly.
   *
   * @param polygonId The identifier of the polygon (foreign key into
   * the FBR and RNG tables).
   * @param path The path to the directory containing the fbr and rng
   * files for the polygon.
   */
  VpfPolygon (int polygonId, const std::string &path,
	      VpfTableManager &tableManager);


  /**
   * Get the raw FBR table.
   *
   * <p>This table contains minimum bounding rectangles for each polygon,
   * referenced by the polygon id.  This is a lazy implementation: the
   * table won't be loaded unless it's actually needed.</p>
   *
   * @return The FBR table.
   */
  const VpfTable &getFBR () const;


  /**
   * Get the raw RNG tables.
   *
   * <p>This table contains pointers, referenced by polygon id, to the
   * lines in the EDG table representing each of the polygon's
   * contours.  This is a lazy implementation: the table won't be
   * loaded unless it's actually needed.</p>
   *
   * @return The RNG tables.
   */
  const VpfTable &getRNG () const;

private:

  int _polygon_id;
  mutable const VpfTable * _fbr;
  mutable const VpfTable * _rng;
};

#endif

// end of polygon.hxx
