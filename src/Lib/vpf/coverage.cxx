// coverage.cxx - implementation of VpfCoverage
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "coverage.hxx"
#include "library.hxx"
#include "table.hxx"
#include "feature.hxx"

#include <string>
#include <vector>
#include <algorithm>

using std::string;
using std::vector;



////////////////////////////////////////////////////////////////////////
// Implementation of VpfCoverage
////////////////////////////////////////////////////////////////////////


VpfCoverage::VpfCoverage (const string &path, const VpfLibrary &lib,
			  int cat_row)
  : VpfComponent(lib.getTableManager(), path),
    _library_path(lib.getPath()),
    _name(lib.getCAT().getValue(cat_row, "coverage_name").getText()),
    _description(lib.getCAT().getValue(cat_row, "description").getText()),
    _level(lib.getCAT().getValue(cat_row, "level").getInt()),
    _feature_names(0),
    _fcs(0)
{
}

VpfCoverage::VpfCoverage (const VpfCoverage &coverage)
  : VpfComponent(coverage.getTableManager(), coverage.getPath()),
    _library_path(coverage._library_path),
    _name(coverage._name),
    _description(coverage._description),
    _level(coverage._level),
    _feature_names(0),
    _fcs(copyTable(coverage._fcs))
{
}

VpfCoverage::~VpfCoverage ()
{
  delete _feature_names;
  freeTable(_fcs);
}

const char *
VpfCoverage::getName () const
{
  return _name.c_str();
}

const char *
VpfCoverage::getDescription () const
{
  return _description.c_str();
}

int
VpfCoverage::getLevel () const
{
  return _level;
}

int
VpfCoverage::getFeatureCount () const
{
  return getFeatureNames().size();
}

const VpfFeature
VpfCoverage::getFeature (int index) const
{
  return VpfFeature(getPath(), getFeatureNames()[index], *this);
}

bool
VpfCoverage::hasFeature (const string &name) const
{
  if (_feature_names == 0)
    getFeatureNames();
  return (find(_feature_names->begin(), _feature_names->end(), name)
	  != _feature_names->end());
}

const VpfFeature
VpfCoverage::getFeature (const string &name) const
{
  if (!hasFeature(name))
    throw VpfException(string("No feature named ") + name);
  return VpfFeature(getPath(), name, *this);
}

const vector<string> &
VpfCoverage::getFeatureNames () const
{
  if (_feature_names == 0) {
    _feature_names = new vector<string>;
    const VpfTable &fcs = getFCS();
    int nRows = fcs.getRowCount();
    for (int i = 0; i < nRows; i++) {
      const string &name = fcs.getValue(i, "feature_class").getText();
      if (!hasFeature(name))
	_feature_names->push_back(name);
    }
  }
  return *_feature_names;
}

const string &
VpfCoverage::getLibraryPath () const
{
  return _library_path;
}

const VpfTable &
VpfCoverage::getFCS () const
{
  if (_fcs == 0)
    _fcs = getChildTable("fcs");
  return *_fcs;
}

// end of coverage.cxx
