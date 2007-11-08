// property.hxx - declaration of VpfPropertyDecl
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_PROPERTY_HXX
#define __VPF_PROPERTY_HXX 1

#include "vpfbase.hxx"
#include "component.hxx"
#include "value.hxx"

#include <string>

class VpfFeature;
class VpfTable;


/**
 * Declaration information for a feature property.
 *
 * <p>This declaration contains extra information about a feature
 * property, useful mainly for generating schemas or summaries (such
 * as a prose description of the property and a list of allowed values
 * and their meanings).</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision: 1.1 $
 */
class VpfPropertyDecl : public VpfComponent
{
public:


  /**
   * Copy constructor.
   *
   * @param decl The feature property declaration to copy.
   */
  VpfPropertyDecl (const VpfPropertyDecl &decl);


  /**
   * Destructor.
   */
  virtual ~VpfPropertyDecl ();


  /**
   * Get the name of the feature property.
   *
   * @return The feature property's name.
   */
  virtual const char * getName () const;


  /**
   * Get a prose description of the feature property.
   *
   * @return The feature property's description.
   */
  virtual const char * getDescription () const;


  /**
   * Get the value type of the feature property.
   *
   * @return The feature property's value type.
   */
  virtual VpfValue::Type getValueType () const;


  /**
   * Count the allowed values for the feature property.
   *
   * <p>A count of 0 means that the property does not have a fixed
   * list of enumerated values (i.e. an elevation, date, or
   * distance).</p>
   *
   * @return The number of allowed values, or 0 for unrestricted
   * values.
   */
  virtual int getValueCount () const;


  /**
   * Get a prose description of one of the allowed values.
   *
   * @param index The zero-based index of the value.
   * @return The prose description, or the empty string ""
   * if none was provided in the VPF feature table.
   * @exception VpfException If the index is out of range.
   */
  virtual const char * getValueDescription (int index) const;


  /**
   * Get an allowed value.
   *
   * @param index The zero-based index of the value.
   * @return The allowed value.
   * @exception VpfException If the index is out of range.
   */
  virtual const VpfValue &getValue (int index) const;


protected:

  friend class VpfFeature;


  /**
   * Protected constructor.
   *
   * <p>This is the only mechanism for creating a new feature property
   * from scratch.  Library users will obtain a feature property from
   * the VpfFeature class, rather than constructing one directly.</p>
   *
   * @param path The path containing the feature table and value
   * description table for the feature.
   * @param xft_col The number of the column in the feature table
   * containing this property.
   * @param feature The parent feature of this property.
   */
  VpfPropertyDecl (const std::string &path, int xft_col,
		   const VpfFeature &feature);
  

  /**
   * Test whether this property is linked to a value description table.
   *
   * @return true if the property is associated with a VDT, false
   * otherwise.
   */
  virtual bool hasVDT () const;


  /**
   * Get the name of the value description table.
   *
   * @return The value description table's name as a string.
   * @exception VpfException If there is no VDT present.
   * @see #hasVDT
   */
  virtual const std::string getVDTName () const;


  /**
   * Get the feature table for this property.
   *
   * <p>The feature table may be for points, lines, polygons, or text
   * labels.  It is already loaded by the parent feature.  The table
   * contains the actual values for each property.</p>
   *
   * @return The feature table for this property.
   */
  virtual const VpfTable &getXFT () const;


  /**
   * Get the value description table for this property.
   *
   * <p>The value description table contains information about
   * each property, such as a prose description and the allowed
   * values.  This is a lazy implementation: the VDT will not be
   * loaded unless it is actually needed.</p>
   *
   * @return The VDT.
   * @exception VpfException If there is no VDT present.
   * @see #hasVDT
   */
  virtual const VpfTable &getVDT () const;

private:
  std::string _xft_name;
  int _xft_col;
  const VpfTable * _xft;
  mutable const VpfTable * _vdt;
};

#endif

// end of property.hxx
