// value.hxx - class representing a VPF leaf value.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_VALUE_HXX
#define __VPF_VALUE_HXX 1

#include <iostream>

#include "vpfbase.hxx"


/**
 * Leaf data in a VPF table.
 *
 * <p>This class represents the actual VPF data primitives: strings,
 * numbers, dates, and arrays of coordinates.  Users need to test
 * the value first to find its data type -- any attempt to access
 * the wrong data type for the value will raise an exception
 * (which can be very helpful in avoiding bugs).</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 */
class VpfValue
{
public:


  /**
   * High-level, user visible data types.
   *
   * <p>The VPF table actually stores more types than these, but these
   * provide a useful simplification for library users.</p>
   */
  enum Type {
    EMPTY,
    TEXT,
    INT,
    REAL,
    POINTS,
    DATE,
    CROSSREF
  };


  /**
   * Default constructor.
   */
  VpfValue ();


  /**
   * Copy constructor.
   *
   * <p>Create a fresh copy of a value.</p>
   *
   * @param value The value to copy.
   */
  VpfValue (const VpfValue &value);


  /**
   * Destructor.
   */
  virtual ~VpfValue ();


  /**
   * Get the user-visible type of the value.
   *
   * @return A user-visible type.
   */
  virtual Type getType () const;


  /**
   * Get the number of elements in the value.
   *
   * <p>Note: for character arrays, this number may be less than the
   * element count provided by the {@link VpfColumnDecl}, because
   * trailing whitespace is trimmed automatically.</p>
   *
   * @return The number of elements in the value.
   * @see VpfColumnDecl#getElementCount
   */
  virtual int getElementCount () const;


  /**
   * Get the contents of a text value.
   *
   * <p>The contents are returned as a character array with a '\0'
   * appended, so that they can be treated as a C string if desired.</p>
   *
   * @return The textual contents of the value.
   * @exception VpfException If the type of the value is not TEXT.
   * @see #getType
   * @see #getElementCount
   */
  virtual const char * getText () const;


  /**
   * Get the contents of an integer value.
   *
   * @return The integer contents of the value.
   * @exception VpfException If the type of the value is not INT.
   * @see #getType
   */
  virtual int getInt () const;


  /**
   * Get the contents of a real-number value.
   *
   * @return The real-number contents of the value.
   * @exception VpfException If the type of the value is not REAL.
   */
  virtual double getReal () const;


  /**
   * Get a 3D point from a coordinate array.
   *
   * <p>VPF stores single- and double-precision 2D and 3D points; this
   * function normalizes all of them the double-precision 3D points,
   * with the z value set to 0 when only 2D information was
   * available.</p>
   *
   * @param index The index of the point in the coordinate array.
   * @return A double-precision 3D point.
   * @exception VpfException If the index is out of range or the type
   * of the value is not POINTS.
   * @see #getType
   * @see #getElementCount
   */
  virtual const VpfPoint getPoint (int index) const;


  /**
   * Get the contents of a date value.
   *
   * @return The date as a character string.
   * @exception VpfException If the type of the value is not DATE.
   * @see #getType
   */
  virtual const char * getDate () const;


  /**
   * Get a cross-tile reference.
   */
  virtual const VpfCrossRef getCrossRef () const;


protected:

  friend class VpfTable;
  friend class VpfColumnDecl;


  /**
   * A single-precision, two-dimensional coordinate.
   */
  struct float_xy {
    float x;
    float y;
  };


  /**
   * A double-precision, two-dimensional coordinate.
   */
  struct double_xy {
    double x;
    double y;
  };


  /**
   * A single-precision, three-dimensional coordinate.
   */
  struct float_xyz {
    float x;
    float y;
    float z;
  };


  /**
   * A double-precision, three-dimensional coordinate.
   */
  struct double_xyz {
    double x;
    double y;
    double z;
  };


  /**
   * Get the raw data type of the value.
   *
   * @return A constant representing the type.
   */
  virtual char getRawType () const;


  /**
   * Assert the raw data type of this value.
   *
   * <p>This method provides a simple way to force an exception if
   * the value is not of the expected type.</p>
   *
   * @param The type expected.
   * @exception VpfException If the expected type does not match the
   * actual type.
   */
  virtual void assertRawType (char type) const;


  /**
   * Get the character data for the value.
   *
   * <p>Note that the return value is <em>unsigned</em>, to allow
   * for various eight-bit character encodings.</p>
   *
   * @return An array of characters (see {@link #getElementCount for
   * the length) with a terminating '\0' (not counted in the length).
   * @exception VpfException If the data type is not one of the
   * *_TEXT types.
   */
  virtual const char * getRawCharArray () const;


  /**
   * Get the short integer data for the value.
   *
   * @return A short integer in system byte order.
   * @exception VpfException If the data type is not 'S'.
   */
  virtual short getRawShort () const;


  /**
   * Get the long integer data for the value.
   *
   * @return A long integer in system byte order.
   * @exception VpfException If the data type is not 'I'.
   */
  virtual int getRawInt () const;

  
  /**
   * Get the single-precision floating-point data for the value.
   *
   * @return A single-precision floating-point number in system byte
   * order.
   * @exception VpfException If the data type is not 'F'.
   */
  virtual float getRawFloat () const;


  /**
   * Get the double-precision floating-point data for the value.
   *
   * @return A short integer in system byte order.
   * @exception VpfException If the data type is not 'R'.
   */
  virtual double getRawDouble () const;


  /**
   * Get an array of single-precision 2D coordinates.
   *
   * @return An array of single-precision, two-dimensional
   * coordinates (see {@link #getElementCount} for the array
   * length).
   * @exception VpfException If the data type is not 'C'.
   */
  virtual const float_xy * getRawFloatXYArray () const;


  /**
   * Get an array of double-precision 2D coordinates.
   *
   * @return An array of double-precision, two-dimensional
   * coordinates (see {@link #getElementCount} for the array
   * length).
   * @exception VpfException If the data type is not 'B'.
   */
  virtual const double_xy * getRawDoubleXYArray () const;


  /**
   * Get an array of single-precision 3D coordinates.
   *
   * @return An array of single-precision, three-dimensional
   * coordinates (see {@link #getElementCount} for the array
   * length).
   * @exception VpfException If the data type is not 'Z'.
   */
  virtual const float_xyz * getRawFloatXYZArray () const;


  /**
   * Get an array of double-precision 3D coordinates.
   *
   * @return An array of double-precision, three-dimensional
   * coordinates (see {@link #getElementCount} for the array
   * length).
   * @exception VpfException If the data type is not 'Y'.
   */
  virtual const double_xyz * getRawDoubleXYZArray () const;


  /**
   * Get the date and time.
   *
   * <p>Note that {@link #getElementCount} will return only 1
   * (because there is one date), but the date is represented
   * by 20 characters.</p>
   *
   * @return An array of 20 characters representing the date.
   * @exception VpfException If the data type is not 'D'.
   */
  virtual const char * getRawDateTime () const;


  /**
   * Get a cross-tile id.
   *
   * @return The cross-tile identifier.
   * @exception VpfException if the data type is not 'K'.
   */
  virtual const VpfCrossRef &getRawCrossTileId () const;


  /**
   * Set a null value.
   *
   * <p>Free any previous value and set the type to 'X'.</p>
   */
  virtual void setNull ();


  /**
   * Set a text value.
   *
   * <p>This method applies to all of the *_TEXT types.  If no
   * explicit type is provided, it defaults to 'T'.</p>
   *
   * <p>This method will automatically strip trailing whitespace
   * during copying, so the array length in the value may be different
   * than the length provided.  The method will free any previous
   * value and set the new type appropriately.  This method will
   * automatically add a trailing '\0', which will not be counted
   * in the length.</p>
   *
   * @param array The array of characters (will be copied).
   * @param size The number of characters in the array.
   * @param type The type of text (defaults to 'T').
   * @exception VpfException If the type provided is not one
   * of the *_TEXT types.
   */
  virtual void setRawCharArray (const char * array, int size,
				char type='T');


  /**
   * Set a short integer value.
   *
   * <p>This method will free any previous value and set the type
   * to 'S'.</p>
   *
   * @param value The new value in system byte order.
   */
  virtual void setRawShort (short value);


  /**
   * Set a long integer value.
   *
   * <p>This method will free any previous value and set the type
   * to LONG.</p>
   *
   * @param value The new value in system byte order.
   */
  virtual void setRawInt (int value);


  /**
   * Set a single-precision floating-point value.
   *
   * <p>This method will free any previous value and set the type
   * to 'F'.</p>
   *
   * @param value The new value.
   */
  virtual void setRawFloat (float value);


  /**
   * Set a double-precision floating-point value.
   *
   * <p>This method will free any previous value and set the type
   * to 'R'.</p>
   *
   * @param value The new value.
   */
  virtual void setRawDouble (double value);


  /**
   * Set a single-precision 2D coordinate array.
   *
   * <p>This method will free any previous value and set the type
   * to 'C'.</p>
   *
   * @param array The coordinate array.
   * @param size The number of elements in the array.
   */
  virtual void setRawFloatXYArray (const float_xy * array, int size);


  /**
   * Set a double-precision 2D coordinate array.
   *
   * <p>This method will free any previous value and set the type
   * to 'B'.</p>
   *
   * @param array The coordinate array.
   * @param size The number of elements in the array.
   */
  virtual void setRawDoubleXYArray (const double_xy * array, int size);


  /**
   * Set a single-precision 3D coordinate array.
   *
   * <p>This method will free any previous value and set the type
   * to 'Z'.</p>
   *
   * @param array The coordinate array.
   * @param size The number of elements in the array.
   */
  virtual void setRawFloatXYZArray (const float_xyz * array, int size);


  /**
   * Set a double-precision 3D coordinate array.
   *
   * <p>This method will free any previous value and set the type
   * to 'Y'.</p>
   *
   * @param array The coordinate array.
   * @param size The number of elements in the array.
   */
  virtual void setRawDoubleXYZArray (const double_xyz * array, int size);


  /**
   * Set a date/time field.
   *
   * <p>Currently, the date and time is represented as an array of
   * 20 characters.</p>
   *
   * <p>This method will free any previous value and set the type
   * to 'D'.</p>
   *
   * @param date An array of 20 characters.
   */
  virtual void setRawDateTime (const char * date);


  /**
   * Set a cross-tile id.
   *
   * @param id The cross-tile-id (will be copied).
   */
  virtual void setRawCrossTileId (const VpfCrossRef &id);


  /**
   * Covert a raw type to a high-level one.
   *
   * @param rawType The raw type to convert.
   * @return The high-level type.
   */
  static Type convertType (char rawType);


private:

  void clear ();

  char _raw_type;

  int _element_count;

  union {
    char * char_array;
    short short_value;
    long int_value;
    float float_value;
    double double_value;
    float_xy * float_xy_array;
    double_xy * double_xy_array;
    float_xyz * float_xyz_array;
    double_xyz * double_xyz_array;
    VpfCrossRef * cross_tile_value;
  } _raw_value;

};


/**
 * Provide a text representation of a value for debugging.
 *
 * @param output The output stream.
 * @param value The value to write.
 * @return The output stream provided.
 */
std::ostream &operator<< (std::ostream &output, const VpfValue &value);


#endif

// end of value.hxx
