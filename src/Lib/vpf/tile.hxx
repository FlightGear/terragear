// tile.hxx - declaration of VpfTile
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_TILE_HXX
#define __VPF_TILE_HXX 1

#include "vpfbase.hxx"
#include "component.hxx"

#include <string>

using std::string;


class VpfTable;


/**
 * Information about a tile.
 *
 * <p>This class applies only to tiled coverages.  The library uses
 * it to find the path to a tile, and the end-user can get a copy
 * from the VpfFeature class to do a quick check on a topology's
 * tile bounds before getting the topology's own bounding rectangle.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 */
class VpfTile : public VpfComponent
{
public:


  /**
   * Copy constructor.
   *
   * @param tile The tile to copy.
   */
  VpfTile (const VpfTile &tile);


  /**
   * Destructor.
   */
  virtual ~VpfTile ();


  /**
   * Get the subdirectory for a tile.
   *
   * <p>The subdirectory returned is relative to a coverage.</p>
   *
   * @return The subdirectory for a tile.
   */
  virtual string getTileSubdir () const;


  /**
   * Get the bounding rectangle for a tile.
   *
   * <p>This is a particularly valuable method for pruning out
   * topologies outside an area of interest -- it is much cheaper
   * to test the bounding rectangle for a tile than it is to check
   * the bounding rectangle for an individual line or polygon, so
   * check this first.</p>
   *
   * @return The bounding rectangle for the tile.
   */
  virtual const VpfRectangle getBoundingRectangle () const;

protected:

  friend class VpfFeature;


  /**
   * Protected constructor.
   *
   * <p>This constructor is the only mechanism for building a new tile
   * object from scratch.  Library users should obtain a tile object
   * from the VpfFeature class.</p>
   *
   * @param path The path to the tileref directory.
   * @param tile_id The identifier of the tile.
   */
  VpfTile (VpfTableManager &tableManager,
	   const std::string &path, int tile_id);


  /**
   * Get the area feature table for the tileref.
   *
   * <p>The AFT contains the subdirectory information and face id
   * for each tile.  This is a lazy implementation: the AFT will not
   * be loaded unless it is actually needed.</p>
   *
   * @return The AFT table.
   */
  const VpfTable &getAFT () const;


  /**
   * Get the face bounding rectangle table for the tileref.
   *
   * <p>The FBR contains the bounding rectangle for each tile, indexed
   * by face id.  This is a lazy implementation: the FBR will not
   * be loaded unless it is actually needed.</p>
   *
   * @return The FBR table.
   */
  const VpfTable &getFBR () const;


private:
  int _tile_id;
  mutable int _face_id;
  mutable const VpfTable * _aft;
  mutable const VpfTable * _fbr;
};

#endif

// end of tile.hxx
