// feature.hxx - declaration of VpfFeature class.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_FEATURE_HXX
#define __VPF_FEATURE_HXX 1

#include "vpfbase.hxx"
#include "component.hxx"

class VpfTable;
class VpfValue;
class VpfCoverage;
class VpfLine;
class VpfPolygon;
class VpfLabel;
class VpfPropertyDecl;
class VpfTile;


/**
 * A feature in a coverage in a library in a VPF database.
 *
 * <p>This is the optional fourth level of the VPF hierarchy: a
 * library (such as transportation) may or may not be divided into
 * separate features (such as roads, railroads, etc.).  This class
 * provides information about a feature, and also provides a
 * convenient way to access all of the shape information for a
 * specific feature in a single place.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 */
class VpfFeature : public VpfComponent
{
public:


  /**
   * The possible topology types.
   */
  enum TopologyType {
    UNKNOWN,
    POINT,
    LINE,
    POLYGON,
    LABEL
  };


  /**
   * Copy constructor.
   */
  VpfFeature (const VpfFeature &feature);


  /**
   * Destructor.
   */
  virtual ~VpfFeature ();


  /**
   * Get the name of this feature.
   *
   * @return The feature's short name as a character string.
   */
  virtual const char * getName () const;


  /**
   * Get the full description of this feature.
   *
   * @return The feature's description as a character string.
   */
  virtual const char * getDescription () const;


  /**
   * Test whether this feature's topology is tiled.
   *
   * @return true if the feature's topology is tiled, false otherwise.
   */
  virtual bool isTiled () const;


  /**
   * Count the feature properties.
   *
   * @return The number of properties for the feature.
   */
  virtual int getPropertyDeclCount () const;


  /**
   * Get a property declaration.
   *
   * @param index The index of the property declaration.
   * @return The property declaration.
   */
  virtual const VpfPropertyDecl getPropertyDecl (int index) const;


  /**
   * Test whether a property is present.
   *
   * @param name The property name to test.
   * @return true if the property is present, false otherwise.
   */
  virtual bool hasProperty (const std::string &name) const;


  /**
   * Get a property declaration by name.
   *
   * @param name The name of the feature property.
   * @return The declaration for the property.
   * @exception VpfException If the specified feature property does not
   * exist.
   */
  virtual const VpfPropertyDecl
  getPropertyDecl (const std::string &name) const;


  /**
   * Get a property value for a specific piece of topology.
   *
   * @param name The property name to look up.
   * @param index The index of the topology.
   * @return The property value for the specified topology.
   * @exception VpfException If the specified feature property does
   * not exist or if the index is out of range.
   * @see #getTopologyCount
   */
  virtual const VpfValue &getPropertyValue (const std::string &name,
					    int index) const;


  /**
   * Get the type of topology covered by the feature.
   *
   * @return 
   */
  virtual TopologyType getTopologyType () const;


  /**
   * Count the topology items in the feature.
   */
  virtual int getTopologyCount () const;


  /**
   * Get a point from this feature.
   *
   * @param index The index of the point to retrieve.
   * @return The point requested.
   * @exception VpfException If the index is out of range.
   */
  virtual const VpfPoint getPoint (int index) const;


  /**
   * Get a line from this feature.
   *
   * @param index The index of the line to retrieve.
   * @return The line requested.
   * @exception VpfException If the index is out of range.
   */
  virtual const VpfLine getLine (int index) const;


  /**
   * Get a polygon from this feature.
   *
   * @param index The index of the polygon to retrieve.
   * @return The polygon requested.
   * @exception VpfException If the index is out of range.
   */
  virtual const VpfPolygon getPolygon (int index) const;


  /**
   * Text a label from this feature.
   *
   * @param index The index of the label to retrieve.
   * @return The label requested.
   * @exception VpfException If the index is out of range.
   */
  virtual const VpfLabel getLabel (int index) const;


  /**
   * Get the tile where a specific topology occurs.
   *
   * @param index The index of the topology.
   * @return The tile where the topology occurs.
   * @exception VpfException If the index is out of range.
   * @see #getTopologyCount
   */
  virtual const VpfTile getTile (int index) const;


protected:

  friend class VpfCoverage;
  friend class VpfPropertyDecl;


  /**
   * Protected constructor.
   *
   * <p>This is the only way to build a new feature from scratch.
   * Library users should obtain a feature object from the VpfCoverage
   * class.</p>
   *
   * @param path The path to the coverage directory.
   * @param name The name of the feature.
   * @param coverage The parent coverage.
   */
  VpfFeature (const std::string &path, const std::string &name,
	      const VpfCoverage &coverage);


  /**
   * Get the feature table for the feature.
   *
   * <p>The feature table (*.pft, *.lft, *.aft, or *.tft, depending
   * on the coverage type) contains information about the feature,
   * including a list of topologies.  This is a lazy implementation:
   * the feature table will not be loaded unless it is actually
   * needed.</p>
   *
   * @return The feature table for this feature.
   */
  virtual const VpfTable &getXFT () const;


  /**
   * Get the feature class attribute table.
   *
   * <p>The FCA contains the description of each feature.  This is
   * a lazy implementation: the FCA will not be loaded unless it is
   * actually needed.</p>
   *
   * @return The FCA table.
   */
  virtual const VpfTable &getFCA () const;


  /**
   * Get the name of the feature table.
   *
   * @return The name of the feature table, with the appropriate
   * extension based on the feature type.
   */
  virtual const std::string getFeatureTableName () const;

private:

  bool isProperty (const std::string &name) const;

  std::string _name;
  TopologyType _topology_type;
  std::string _tileref_path;
  mutable const VpfTable * _xft;
  mutable const VpfTable * _fca;
};

#endif

// end of feature.hxx
