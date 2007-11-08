// line.hxx - declaration of VpfLine
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_LINE_HXX
#define __VPF_LINE_HXX 1

#include "vpfbase.hxx"
#include "component.hxx"

#include <string>

class VpfValue;


/**
 * Line topology.
 *
 * <p>This class represents an actual line, a series of connected
 * points.  The implementation is lazy: getting a line from a
 * polygon or feature doesn't actually cause the (very large) EBR
 * or EDG tables to be read until you actually need them.</p>
 *
 * <p>In VPF terminology, a line is called an "edge", abbreviated
 * "edg".</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision: 1.1 $
 */
class VpfLine : public VpfComponent
{
public:


  /**
   * Copy constructor.
   *
   * @param line The line to copy.
   */
  VpfLine (const VpfLine &line);


  /**
   * Destructor.
   */
  virtual ~VpfLine ();


  /**
   * Get the minimum bounding rectangle for the line.
   *
   * <p>The bounding rectangle is useful for determining whether any
   * of the line falls inside a region of interest.</p>
   *
   * @return A rectangle containing the points for the top-left and
   * bottom-right corners of the smallest rectangle that can fit
   * around the line.
   */
  virtual const VpfRectangle getBoundingRectangle () const;


  /**
   * Count the points in this line.
   *
   * @return The number of points in the line (at least two).
   * @see #getPoint
   */
  virtual int getPointCount () const;


  /**
   * Get a point from the line.
   *
   * @param index The zero-based index of the point to retrieve.
   * @return The point.
   * @exception VpfException If the index is out of range.
   * @see #getPointCount
   */
  virtual const VpfPoint getPoint (int index) const;


protected:

  friend class VpfFeature;
  friend class VpfContour;


  /**
   * Protected constructor.
   *
   * <p>This is the only mechanism for creating a new line from
   * scratch.  Library users will obtain a line from the VpfFeature
   * or VpfPolygon classes, rather than constructing one directly.
   *
   * @param lineId The identifier of the line (foreign key into
   * the ebr and edg tables).
   * @param path The path to the directory containing the EBR and
   * EDG files for the line.  The directory may be a tile directory
   * or the main coverage directory.
   */
  VpfLine (int lineId, const std::string &path, VpfTableManager &tableManager);


  /**
   * Get the raw EBR table.
   *
   * <p>This table contains minimum bounding rectangles for each line,
   * referenced by the line id.  This is a lazy implementation: the
   * table won't be loaded unless it's actually needed.</p>
   *
   * @return The EBR table.
   */
  const VpfTable &getEBR () const;


  /**
   * Get the raw EDG table.
   *
   * <p>This table contains the actual points for each line.  This
   * is a lazy implementation: the table won't be loaded unless
   * it's actually needed.</p>
   *
   * @return The EDG table.
   */
  const VpfTable &getEDG () const;


  /**
   * Get the value from the EDG table containing the line's points.
   *
   * @return The value from the EDG table.
   */
  const VpfValue &getValue () const;


private:
  int _line_id;
  mutable const VpfTable * _ebr;
  mutable const VpfTable * _edg;
  mutable const VpfValue * _value; // no need to delete
};

#endif

// end of line.hxx
