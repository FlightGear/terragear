// database.cxx - implementation of VpfDatabase
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "vpfbase.hxx"
#include "database.hxx"
#include "library.hxx"
#include "table.hxx"

#include <string>

using std::string;


VpfDataBase::VpfDataBase (const string &path)
  : VpfComponent(_table_manager, path),
    _dht(0),
    _lat(0)
{
}

VpfDataBase::VpfDataBase (const VpfDataBase &database)
  : VpfComponent(database._table_manager, database.getPath()),
    _dht(copyTable(database._dht)),
    _lat(copyTable(database._lat))
{
}

VpfDataBase::~VpfDataBase ()
{
  freeTable(_dht);
  freeTable(_lat);
}

const char *
VpfDataBase::getName () const
{
  return getDHT().getValue(0, "database_name").getText();
}

const char *
VpfDataBase::getDescription () const
{
  return getDHT().getValue(0, "database_desc").getText();
}

int
VpfDataBase::getLibraryCount () const
{
  return getLAT().getRowCount();
}

const VpfLibrary
VpfDataBase::getLibrary (int index) const
{
  const char * name = getLAT().getValue(index, "library_name").getText();
  return getLibrary(name);
}

bool
VpfDataBase::hasLibrary (const std::string &name) const
{
  return (getLAT().findMatch("library_name", name.c_str()) != -1);
}

const VpfLibrary
VpfDataBase::getLibrary (const string &name) const
{
  if (!hasLibrary(name))
    throw VpfException(string("No library named ") + name);
  return VpfLibrary(getChildFileName(name), *this);
}


const VpfTable &
VpfDataBase::getDHT () const
{
  if (_dht == 0)
    _dht = getChildTable("dht");
  return *_dht;
}

const VpfTable &
VpfDataBase::getLAT () const
{
  if (_lat == 0)
    _lat = getChildTable("lat");
  return *_lat;
}


// end of database.cxx
