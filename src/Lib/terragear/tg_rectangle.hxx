// rectangle.hxx - a simple rectangle class (for bounds, etc.)
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#ifndef __RECTANGLE_HXX
#define __RECTANGLE_HXX 1

#ifndef __cplusplus                                                          
# error This library requires C++
#endif  

#include <simgear/compiler.h>
#include <simgear/math/SGMath.hxx>

/**
 * A simple rectangle class for bounding rectangles.
 *
 * The class defines a rectangle in by the vertices of its minimum and
 * maximum corners, ignoring any z coordinates.  There are methods to
 * sanify the rectangle (to make certain that each point is correct)
 * and to test whether another point lies inside it.
 */
class tgRectangle
{
public:

    /**
     * Create a new empty rectangle with both points at 0,0.
     */
    tgRectangle();

    /**
     * Copy an existing rectangle.
     *
     * @param r The rectangle to copy.
     */
    tgRectangle (const tgRectangle &r);

    /**
     * Convenience constructor.
     */
    tgRectangle (const SGGeod& min, const SGGeod& max);

    /**
     * Destructor.
     */
    virtual ~tgRectangle();

    /**
     * Get the minimum (top left) corner of the rectangle.
     *
     * @return The top-left vertex.
     */
    virtual const SGGeod &getMin () const { return _min; }

    /**
     * Get the maximum (bottom right) corner of the rectangle.
     *
     * @return The bottom-right vertex.
     */
    virtual const SGGeod &getMax () const { return _max; }

    /**
     * Get the minimum (top left) corner of the rectangle.
     *
     * @return The top-left vertex.
     */
    virtual SGGeod &getMin () { return _min; }

    /**
     * Get the maximum (bottom right) corner of the rectangle.
     *
     * @return The bottom-right vertex.
     */
    virtual SGGeod &getMax () { return _max; }

    /**
     * Set the minimum (top-left) corner of the rectangle.
     *
     * @param p The top-left vertex.
     */
    virtual void setMin (const SGGeod& p);

    /**
     * Set the maximum (bottom-right) corner of the rectangle.
     *
     * @param p The bottom-right vertex.
     */
    virtual void setMax (const SGGeod& p);

    /**
     * Expand the rectangle to encompass a given rectangle
     *
     * @param r The rectangle to expand by
     */
    virtual void expandBy(const tgRectangle& r);

    /**
     * Make the rectangle sane.
     *
     * Ensure that the min vertex is less than the max vertex.
     */
    virtual void sanify ();

    /**
     * Test whether a point lies inside the rectangle.
     *
     * The z-coordinates are ignored.
     *
     * @param p The point to test.
     * @return true if the point is inside or on the boundary of the
     *  rectangle, false if it is outside.
     */
    virtual bool isInside (const SGGeod& p) const;

    /**
     * Test whether this rectangle overlaps with another one.
     *
     * @param r The rectangle to test.
     * @return true if the rectangle is touching or overlapping, false
     * otherwise.
     */
    virtual bool intersects (const tgRectangle &r) const;

private:
    SGGeod _min;
    SGGeod _max;
};

#endif // __TGRECTANGLE_HXX