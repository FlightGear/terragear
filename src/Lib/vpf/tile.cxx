// tile.cxx - implementation of VpfTile
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "tile.hxx"
#include "table.hxx"
#include "value.hxx"

#include <string>

using std::string;

VpfTile::VpfTile (VpfTableManager &tableManager,
		  const string &path, int tile_id)
  : VpfComponent(tableManager, path),
    _tile_id(tile_id),
    _aft(0),
    _fbr(0)
{
  const VpfTable &aft = getAFT();
  int row = aft.findMatch("id", _tile_id);
  _face_id = aft.getValue(row, "fac_id").getInt();
}

VpfTile::VpfTile (const VpfTile &tile)
  : VpfComponent(tile.getTableManager(), tile.getPath()),
    _tile_id(tile._tile_id),
    _face_id(tile._face_id),
    _aft(copyTable(tile._aft)),
    _fbr(copyTable(tile._fbr))
{
}

VpfTile::~VpfTile ()
{
  freeTable(_aft);
  freeTable(_fbr);
}

string
VpfTile::getTileSubdir () const
{
  const VpfTable &aft = getAFT();
  int row = aft.findMatch("id", _tile_id);
  string subdir = aft.getValue(row, "tile_name").getText();
  for (unsigned int i = 0; i < subdir.size(); i++) {
    if (subdir[i] == '\\')
      subdir[i] = PATHSEP;
    else
      subdir[i] = tolower(subdir[i]);
  }
  return subdir;
}

const VpfRectangle
VpfTile::getBoundingRectangle () const
{
  VpfRectangle rect;
  const VpfTable &fbr = getFBR();

  int row = fbr.findMatch("id", _face_id);
  rect.minX = fbr.getValue(row, "xmin").getReal();
  rect.minY = fbr.getValue(row, "ymin").getReal();
  rect.maxX = fbr.getValue(row, "xmax").getReal();
  rect.maxY = fbr.getValue(row, "ymax").getReal();
  return rect;
}

const VpfTable &
VpfTile::getAFT () const
{
  if (_aft == 0)
    _aft = getChildTable("tileref.aft");
  return *_aft;
}

const VpfTable &
VpfTile::getFBR () const
{
  if (_fbr == 0)
    _fbr = getChildTable("fbr");
  return *_fbr;
}

// end of tile.cxx
