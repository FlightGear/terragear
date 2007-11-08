// coverage.hxx - declaration of VpfCoverage.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_COVERAGE_HXX
#define __VPF_COVERAGE_HXX 1

#include "vpfbase.hxx"
#include "component.hxx"

#include <string>
#include <vector>

class VpfTable;
class VpfLibrary;
class VpfFeature;
class VpfLine;
class VpfPolygon;
class VpfTileRef;

/**
 * A coverage in a library in a VPF database.
 *
 * <p>This is the third level of the VPF hierarchy: every database
 * contains one or move libraries, and every library contains one or
 * more coverages, collections of GIS data belonging to the same
 * general class (i.e. transportation) and sharing the same attribute
 * dictionary.  This class has a copy constructor, so it can
 * safely be assigned and passed around by value.</p>
 *
 * <p>All of the points, lines, and polygons for the coverage are
 * available through this class; however, if the coverage contains
 * multiple features, it will often be simpler to access the points,
 * lines, and polygons through each feature individually.</p>
 *
 * <p>Users should obtain a copy of the coverage object through the
 * library's getCoverage methods; new coverage objects cannot be
 * created directly (except by copying an existing one).</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision: 1.2 $
 * @see VpfDataBase
 * @see VpfLibrary
 */
class VpfCoverage : public VpfComponent
{
public:


  /**
   * Copy constructor.
   *
   * @param coverage The coverage to copy.
   */
  VpfCoverage (const VpfCoverage &coverage);


  /**
   * Destructor.
   */
  virtual ~VpfCoverage ();


  /**
   * Get the name of the coverage.
   *
   * @return The coverage's name as a character string.
   */
  virtual const char * getName () const;


  /**
   * Get a description of the coverage.
   *
   * @return The coverage's description as a character string.
   */
  virtual const char * getDescription () const;


  /**
   * Get the topographical level of the coverage.
   *
   * <p>According to MIL-STD-2407, the meaning of this integer is as
   * follows:</p>
   *
   * <dl>
   * <dt>Level 0</dt>
   * <dd>Purely geometric aspects of spatial data, with no 
   * topology.</dd>
   * <dt>Level 1</dt>
   * <dd>Non-planar graph.</dd>
   * <dt>Level 2</dt>
   * <dd>Planar graph.</dd>
   * <dt>Level 3</dt>
   * <dd>Faces defined by the planar graph.</dd>
   * </dl>
   *
   * <p>Make of that what you will.</p>
   *
   * @return The coverage's topographical level.
   */
  virtual int getLevel () const;


  /**
   * Count the features.
   *
   * <p>A feature is a category of data within the coverage, such as
   * roads, or forest.</p>
   *
   * @return The number of features present in the coverage.
   * @see #getFeature
   */
  virtual int getFeatureCount () const;


  /**
   * Get a feature by index.
   *
   * @param index The zero-based index of the feature.
   * @return A copy of an object representing the feature.
   * @exception VpfException If the index is out of bounds.
   * @see #getFeatureCount
   */
  virtual const VpfFeature getFeature (int index) const;


  /**
   * Test whether a feature is present.
   *
   * @param name The feature name.
   * @return true if the feature is present, false if it is not.
   */
  virtual bool hasFeature (const std::string &name) const;


  /**
   * Get a feature by name.
   *
   * @param name The name of the feature.
   * @return A copy of the object representing the feature.
   * @exception VpfException If no feature exists with the name
   * provided.
   * @see #hasFeature
   */
  virtual const VpfFeature getFeature (const std::string &name) const;


protected:

  friend class VpfLibrary;
  friend class VpfFeature;


  /**
   * Protected constructor.
   *
   * <p>This is the only mechanism for creating a new coverage from
   * scratch.  Library users will obtain a coverage object from
   * the VpfLibrary class, rather than constructing one directly.</p>
   *
   * @param path The path to the directory containing the FCS table.
   * @param library The parent library object.
   * @param cat_row The row in the parent library's CAT table.
   */
  VpfCoverage (const std::string &path, const VpfLibrary &library,
	       int cat_row);


  /**
   * Get a feature schema table.
   *
   * <p>The feature schema table declares all of the features used
   * in the coverage.  This is a lazy implementation: the FCS will not
   * be loaded unless it is actually needed.</p>
   *
   * @return The feature schema table for this coverage.
   */
  const VpfTable &getFCS () const;


  /**
   * Get a vector of feature names for this coverage.
   *
   * <p>The names are collected from the FCS.  This is a lazy
   * implementation: the vector will not be built unless it is
   * actually needed.</p>
   *
   * @return A vector containing one copy of each feature name.
   */
  const std::vector<std::string> &getFeatureNames () const;


  /**
   * Get the path of the parent library.
   *
   * @return The parent library's path.
   */
  const std::string &getLibraryPath () const;


private:
  std::string _library_path;
  std::string _name;
  std::string _description;
  int _level;

  mutable std::vector<std::string> * _feature_names;
  mutable const VpfTable * _fcs;

};

#endif

// end of coverage.hxx
