// component.cxx - implementation of VpfComponent
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "component.hxx"
#include "tablemgr.hxx"

#include <string>
#include <fstream>

using std::string;
using std::ifstream;


VpfComponent::VpfComponent (VpfTableManager &tableManager,
			    const string &path)
  :  _path(path),
     _table_manager(tableManager)
{
}

VpfComponent::~VpfComponent ()
{
}

const string &
VpfComponent::getPath () const
{
  return _path;
}

bool
VpfComponent::hasChildFile (const string &child) const
{
  return hasFile(getPath(), child);
}

string
VpfComponent::getChildFileName (const string &child) const
{
  return getFileName (getPath(), child);
}

bool
VpfComponent::hasFile (const string &path, const string &file) const
{
				// FIXME: portable, but not that efficient
  bool result;
  string fullpath = getFileName(path, file);
  ifstream input(fullpath.c_str());
  if (!input) {
    input.clear();
    fullpath += '.';
    input.open(fullpath.c_str());
  }
  result = input;
  input.close();
  return result;
}

string
VpfComponent::getFileName (const string &path, const string &file) const
{
  string result = path;
  if (result[result.length()-1] != PATHSEP)
    result += PATHSEP;
  result += file;
  return result;
}

const VpfTable *
VpfComponent::getChildTable (const string &child) const
{
  return getTable(getChildFileName(child));
}

const VpfTable *
VpfComponent::getTable (const string &path) const
{
  return _table_manager.getTable(path);
}

const VpfTable *
VpfComponent::copyTable (const VpfTable * table) const
{
  return _table_manager.copyTable(table);
}

void
VpfComponent::freeTable (const VpfTable * table) const
{
  _table_manager.freeTable(table);
}

VpfTableManager &
VpfComponent::getTableManager () const
{
  return _table_manager;
}


// end of component.cxx
