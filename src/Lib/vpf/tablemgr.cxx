// tablemgr.cxx - implementation of VpfTableManager
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "tablemgr.hxx"
#include "table.hxx"

//  #include <iostream>
//  using std::cerr;
//  using std::endl;

#include <string>
#include <map>
#include <list>

using std::string;
using std::map;
using std::list;


VpfTableManager::VpfTableManager ()
  : _table_count(0)
{
}

VpfTableManager::~VpfTableManager ()
{
  cull_queue(0);
//    cerr << "Queue size at end: " << _empty_queue.size() << endl;
//    cerr << "Total tables remaining: " << _table_count << endl;
  if (_table_count != 0)
    throw VpfTable("Table manager deleted while some tables still in use");
}

const VpfTable *
VpfTableManager::getTable (const string &path)
{
  const VpfTable * result = _table_map[path];
  if (result == 0) {
    result = new VpfTable(path);
    _table_map[path] = result;
    _table_references[path] = 1;
    _table_count++;
  } else {
    if (_table_references[path] < 1)
      remove_from_queue(result);
    _table_references[path]++;
  }
  return result;
}

const VpfTable *
VpfTableManager::copyTable (const VpfTable * table)
{
  if (table != 0) {
    _table_references[table->getPath()]++;
  }
  return table;
}

void
VpfTableManager::freeTable (const VpfTable *table)
{
  if (table != 0) {
    const string &path = table->getPath();
    _table_references[path]--;
    if (_table_references[path] == 0)
      add_to_queue(table);
    cull_queue(MAX_QUEUE_SIZE);
  }
}

void
VpfTableManager::add_to_queue (const VpfTable * table)
{
  _empty_queue.push_front(table);
}

void
VpfTableManager::remove_from_queue (const VpfTable * table)
{
  _empty_queue.remove(table);
}

void
VpfTableManager::cull_queue (int max_size)
{
  while (int(_empty_queue.size()) > max_size) {
    const VpfTable * table = _empty_queue.back();
//      cerr << "Culling " << table->getPath() << endl;
    _empty_queue.pop_back();
    _table_map[table->getPath()] = 0; // FIXME: remove key completely
    _table_references[table->getPath()] = 0; // FIXME: ditto
    delete table;
    _table_count--;
  }
}

// end of tablemgr.cxx
