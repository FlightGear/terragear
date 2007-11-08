// label.hxx - declaration of VpfLabel
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_LABEL_HXX
#define __VPF_LABEL_HXX 1

#include "vpfbase.hxx"
#include "component.hxx"

class VpfTable;


/**
 * Label topology.
 *
 * <p>This class represents a textual label attached to a point.
 * In vmap0, most labels (i.e. for geographical locations) are
 * actually contained in point topology with a txt feature property,
 * so this class is not needed that often.  This implementation is
 * lazy: getting a label from a feature doesn't actually cause
 * the TXT table to be read until you actually need it.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision: 1.1 $
 */
class VpfLabel : public VpfComponent
{
public:


  /**
   * Copy constructor.
   *
   * @param label The label to copy.
   */
  VpfLabel (const VpfLabel &label);


  /**
   * Destructor.
   */
  virtual ~VpfLabel ();


  /**
   * Get the text associated with the label.
   *
   * @return The label's text.
   */
  virtual const char * getText () const;


  /**
   * Get the point associated with the label.
   *
   * @return The label's point.
   */
  virtual const VpfPoint getPoint () const;

protected:

  friend class VpfFeature;


  /**
   * Protected constructor.
   *
   * <p>This is the only way to build a new label from scratch.
   * Library users should obtain label objects from the VpfFeature
   * class.</p>
   *
   * @param labelId The label identifier.
   * @param path The path to the directory containing the txt file.
   */
  VpfLabel (int labelId, const std::string &path,
	    VpfTableManager &tableManager);


  /**
   * Get the raw TXT table.
   *
   * <p>This table contains the text and point for each label.
   * This is a lazy implementation: the table won't be loaded unless
   * it's actually needed.</p>
   */
  const VpfTable &getTXT () const;

private:
  int _label_id;
  mutable const VpfTable * _txt;

};

#endif

// end of label.hxx
