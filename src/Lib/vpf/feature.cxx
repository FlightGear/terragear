// feature.cxx - implementation of VpfFeature class.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include <string>

#include "vpfbase.hxx"
#include "value.hxx"
#include "feature.hxx"
#include "coverage.hxx"
#include "table.hxx"
#include "line.hxx"
#include "polygon.hxx"
#include "label.hxx"
#include "property.hxx"
#include "tile.hxx"

using std::string;


VpfFeature::VpfFeature (const string &path, const string &name,
			const VpfCoverage &coverage)
  : VpfComponent(coverage.getTableManager(), path),
    _name(name),
    _topology_type(UNKNOWN),
    _tileref_path(getFileName(coverage.getLibraryPath(), "tileref")),
    _xft(0),
    _fca(0)
{
  if (hasChildFile(_name + ".pft"))
    _topology_type = POINT;
  else if (hasChildFile(_name + ".lft"))
    _topology_type = LINE;
  else if (hasChildFile(_name + ".aft"))
    _topology_type = POLYGON;
  else if (hasChildFile(_name + ".tft"))
    _topology_type = LABEL;
}

VpfFeature::VpfFeature (const VpfFeature &feature)
  : VpfComponent(feature.getTableManager(), feature.getPath()),
    _name(feature._name),
    _topology_type(feature._topology_type),
    _tileref_path(feature._tileref_path),
    _xft(copyTable(feature._xft)),
    _fca(copyTable(feature._fca))
{
}

VpfFeature::~VpfFeature ()
{
  freeTable(_xft);
  freeTable(_fca);
}

const char *
VpfFeature::getName () const
{
  return _name.c_str();
}

const char *
VpfFeature::getDescription () const
{
  if (hasChildFile("fca")) {
    const VpfTable &fca = getFCA();
    int row = fca.findMatch("fclass", getName());
    return fca.getValue(row, "descr").getText();
  } else {
    return "";
  }
}

bool
VpfFeature::isTiled () const
{
  return getXFT().hasColumn("tile_id");
}

int
VpfFeature::getPropertyDeclCount () const
{
  const VpfTable &xft = getXFT();
  int nCols = xft.getColumnCount();
  int result = 0;
  for (int i = 0; i < nCols; i++) {
    const VpfColumnDecl &decl = xft.getColumnDecl(i);
    if (isProperty(decl.getName()))
      result++;
  }
  return result;
}

const VpfPropertyDecl
VpfFeature::getPropertyDecl (int index) const
{
  const VpfTable &xft = getXFT();
  int nCols = xft.getColumnCount();
  for (int i = 0; i < nCols; i++) {
    const VpfColumnDecl &decl = xft.getColumnDecl(i);
    if (isProperty(decl.getName())) {
      if (index == 0)
	return VpfPropertyDecl(getPath(), i, *this);
      else
	index--;
    }
  }
  throw VpfException("property declaration index out of range");
}

bool
VpfFeature::hasProperty (const string &name) const
{
  return (isProperty(name) && getXFT().hasColumn(name));
}

const VpfPropertyDecl
VpfFeature::getPropertyDecl (const string &name) const
{
  if (!hasProperty(name))
    throw VpfException(string("No feature property named ") + name);
  int col = getXFT().findColumn(name);
  return VpfPropertyDecl(getPath(), col, *this);
}

const VpfValue &
VpfFeature::getPropertyValue (const string &name, int index) const
{
  if (!hasProperty(name))
    throw VpfException(string("No feature property named ") + name);
  return getXFT().getValue(index, name);
}

VpfFeature::TopologyType
VpfFeature::getTopologyType () const
{
  return _topology_type;
}

int
VpfFeature::getTopologyCount () const
{
  return getXFT().getRowCount();
}

const VpfPoint
VpfFeature::getPoint (int index) const
{
  if (_topology_type != POINT)
    throw VpfException("Not point topology");
  
  const VpfTable &pft = getXFT();
  int pointId = pft.getValue(index, "end_id").getInt();
  if (isTiled()) {
    
    string end_path =
      getFileName(getChildFileName(getTile(index).getTileSubdir()), "end");
    const VpfTable * end = getTable(end_path);
    int row = end->findMatch("id", pointId);
    const VpfPoint p = end->getValue(row, "coordinate").getPoint(0);
    freeTable(end);
    return p;
  } else {
    const VpfTable * end = getChildTable("end");
    int row = end->findMatch("id", pointId);
    const VpfPoint p = end->getValue(row, "coordinate").getPoint(0);
    freeTable(end);
    return p;
  }
}

const VpfLine
VpfFeature::getLine (int index) const
{
  if (_topology_type != LINE)
    throw VpfException("Not line topology");
  const VpfTable &lft = getXFT();
  int lineId = lft.getValue(index, "edg_id").getInt();
  if (isTiled()) {
    string path = getChildFileName(getTile(index).getTileSubdir());
    return VpfLine(lineId, path, getTableManager());
  } else {
    return VpfLine(lineId, getPath(), getTableManager());
  }
}

const VpfPolygon
VpfFeature::getPolygon (int index) const
{
  if (_topology_type != POLYGON)
    throw VpfException("Not polygon topology");
  const VpfTable &aft = getXFT();
  int polygonId = aft.getValue(index, "fac_id").getInt();
				// Polygon #1 is the global polygon;
				// no one should ever be trying to
				// fetch it from a feature.
  if (polygonId == 1)
    throw VpfException("Getting polygon #1!!!!!");
  if (isTiled()) {
    string path = getChildFileName(getTile(index).getTileSubdir());
    return VpfPolygon(polygonId, path, getTableManager());
  } else {
    return VpfPolygon(polygonId, getPath(), getTableManager());
  }
}

const VpfLabel
VpfFeature::getLabel (int index) const
{
  if (_topology_type != LABEL)
    throw VpfException("Not label topology");
  const VpfTable &tft = getXFT();
  int labelId = tft.getValue(index, "txt_id").getInt();

  if (isTiled()) {
    string path = getChildFileName(getTile(index).getTileSubdir());
    return VpfLabel(labelId, path, getTableManager());
  } else {
    return VpfLabel(labelId, getPath(), getTableManager());
  }
}

const VpfTile
VpfFeature::getTile (int index) const
{
  if (!isTiled())
    throw VpfException("Not a tiled feature");
  int tile_id = getXFT().getValue(index, "tile_id").getInt();
  return VpfTile(getTableManager(), _tileref_path, tile_id);
}

const VpfTable &
VpfFeature::getXFT () const
{
  if (_xft == 0)
    _xft = getChildTable(getFeatureTableName());
  return *_xft;
}

const VpfTable &
VpfFeature::getFCA () const
{
  if (_fca == 0)
    _fca = getChildTable("fca");
  return *_fca;
}

bool
VpfFeature::isProperty (const string &name) const
{
  return (name != "id" &&
	  name != "tile_id" &&
	  name != "end_id" &&
	  name != "edg_id" &&
	  name != "fac_id" &&
	  name != "txt_id");
}

const string
VpfFeature::getFeatureTableName () const
{
    switch (_topology_type) {
    case POINT:
      return _name + ".pft";
    case LINE:
      return _name + ".lft";
    case POLYGON:
      return _name + ".aft";
    case LABEL:
      return _name + ".tft";
    default:
      throw VpfException("Unsupported feature topology type");
    }
}


// end of feature.cxx
