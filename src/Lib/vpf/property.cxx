// property.cxx - implementation of VpfProperty
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "property.hxx"
#include "table.hxx"
#include "feature.hxx"

#include <string>
using std::string;

VpfPropertyDecl::VpfPropertyDecl (const string &path, int xft_col,
				  const VpfFeature &feature)
  : VpfComponent(feature.getTableManager(), path),
    _xft_name(feature.getFeatureTableName()),
    _xft_col(xft_col),
    _xft(copyTable(&(feature.getXFT()))),
    _vdt(0)
{
}

VpfPropertyDecl::VpfPropertyDecl (const VpfPropertyDecl &decl)
  : VpfComponent(decl.getTableManager(), decl.getPath()),
    _xft_name(decl._xft_name),
    _xft_col(decl._xft_col),
    _xft(copyTable(decl._xft)),
    _vdt(copyTable(decl._vdt))
{
}

VpfPropertyDecl::~VpfPropertyDecl ()
{
  freeTable(_xft);
  freeTable(_vdt);
}

const char *
VpfPropertyDecl::getName () const
{
  return getXFT().getColumnDecl(_xft_col).getName().c_str();
}

const char *
VpfPropertyDecl::getDescription () const
{
  return getXFT().getColumnDecl(_xft_col).getDescription().c_str();
}

VpfValue::Type
VpfPropertyDecl::getValueType () const
{
  return getXFT().getColumnDecl(_xft_col).getValueType();
}

int
VpfPropertyDecl::getValueCount () const
{
  if (hasVDT()) {
    string name = getName();
    const VpfTable &vdt = getVDT();
    int nRows = vdt.getRowCount();
    int result = 0;
    for (int i = 0; i < nRows; i++) {
      if (name == vdt.getValue(i, "attribute").getText() &&
	  _xft_name == vdt.getValue(i, "table").getText())
	result++;
    }
    return result;
  } else {
    return 0;
  }
}

const char *
VpfPropertyDecl::getValueDescription (int index) const
{
  if (hasVDT()) {
    string name = getName();
    const VpfTable &vdt = getVDT();
    int nRows = vdt.getRowCount();
    for (int i = 0; i < nRows; i++) {
      if (name == vdt.getValue(i, "attribute").getText() &&
	  _xft_name == vdt.getValue(i, "table").getText()) {
	if (index == 0)
	  return vdt.getValue(i, "description").getText();
	else
	  index--;
      }
    }
    throw VpfException("value not found");
  } else {
    throw VpfException("index out of range");
  }
}

const VpfValue &
VpfPropertyDecl::getValue (int index) const
{
  if (hasVDT()) {
    int row = getVDT().findMatch("attribute", getName(), index);
    return getVDT().getValue(row, "value");
  } else {
    throw VpfException("index out of range");
  }
}

const string
VpfPropertyDecl::getVDTName () const
{
  return getXFT().getColumnDecl(_xft_col).getValueDescriptionTable();
}

bool
VpfPropertyDecl::hasVDT () const
{
  return (getVDTName() != "-");
}

const VpfTable &
VpfPropertyDecl::getXFT () const
{
  return *_xft;			// initialized in constructor
}

const VpfTable &
VpfPropertyDecl::getVDT () const
{
  if (_vdt == 0) {
    string name = getVDTName();
    if (name == "-")
      throw VpfException("No VDT");
    else
      _vdt = getChildTable(name);
  }
  return *_vdt;
}


// end of property.cxx
