// library.cxx - implementation of VpfLibrary
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "vpfbase.hxx"
#include "table.hxx"
#include "database.hxx"
#include "library.hxx"
#include "coverage.hxx"

using std::string;

VpfLibrary::VpfLibrary (const std::string &path, const VpfDataBase &database)
  : VpfComponent(database.getTableManager(), path),
    _lat(copyTable(&(database.getLAT()))),
    _cat(0),
    _lht(0),
    _tileref_aft(0),
    _tileref_fbr(0)
{
  string path = getChildFileName("tileref");
  if (hasFile(path, "tileref.aft"))
    _tileref_aft = getTable(getFileName(path, "tileref.aft"));
  if (hasFile(path, "fbr"))
    _tileref_fbr = getTable(getFileName(path, "fbr"));
}

VpfLibrary::VpfLibrary (const VpfLibrary &library)
  : VpfComponent(library.getTableManager(), library.getPath()),
    _lat(copyTable(library._lat)),
    _cat(copyTable(library._cat)),
    _lht(copyTable(library._lht)),
    _tileref_aft(copyTable(library._tileref_aft)),
    _tileref_fbr(copyTable(library._tileref_fbr))
{
}

VpfLibrary::~VpfLibrary ()
{
  freeTable(_lat);
  freeTable(_cat);
  freeTable(_lht);
  freeTable(_tileref_aft);
  freeTable(_tileref_fbr);
}

const char *
VpfLibrary::getName () const
{
  return getLHT().getValue(0, "library_name").getText();
}

const char *
VpfLibrary::getDescription () const
{
  return getLHT().getValue(0, "description").getText();
}

const VpfRectangle
VpfLibrary::getBoundingRectangle () const
{
  const VpfTable &lat = getLAT();
  VpfRectangle bounds;
  int row = lat.findMatch("library_name", getName());
  bounds.minX = lat.getValue(row, "xmin").getReal();
  bounds.minY = lat.getValue(row, "ymin").getReal();
  bounds.maxX = lat.getValue(row, "xmax").getReal();
  bounds.maxY = lat.getValue(row, "ymax").getReal();
  return bounds;
}

int
VpfLibrary::getCoverageCount () const
{
  return getCAT().getRowCount();
}

const VpfCoverage
VpfLibrary::getCoverage (int index) const
{
  const char * name = getCAT().getValue(index, "coverage_name").getText();
  return VpfCoverage(getChildFileName(name), *this, index);
}

bool
VpfLibrary::hasCoverage (const std::string &name) const
{
  return (getCAT().findMatch("coverage_name", name.c_str()) != -1);
}

const VpfCoverage
VpfLibrary::getCoverage (const string &name) const
{
  if (!hasCoverage(name))
    throw VpfException(string("No coverage named " + name));
  int cat_row = getCAT().findMatch("coverage_name", name.c_str());
  return VpfCoverage(getChildFileName(name), *this, cat_row);
}

const VpfTable &
VpfLibrary::getLAT () const
{
  return *_lat;
}

const VpfTable &
VpfLibrary::getCAT () const
{
  if (_cat == 0)
    _cat = getChildTable("cat");
  return *_cat;
}

const VpfTable &
VpfLibrary::getLHT () const
{
  if (_lht == 0)
    _lht = getChildTable("lht");
  return *_lht;
}

// end of library.cxx
