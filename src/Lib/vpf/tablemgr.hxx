// tablemgr.hxx - declaration of VpfTableManager
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_TABLEMGR_HXX
#define __VPF_TABLEMGR_HXX 1

#include <string>
#include <map>
#include <list>

class VpfTable;


/**
 * Manage VPF tables in memory with reference counting.
 *
 * <p>This is an extremely simplistic management class, but it's
 * useful for ensuring that tables are kept around for a little
 * while in case they're needed, and that the same table is
 * never loaded twice.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 */
class VpfTableManager
{
public:
  VpfTableManager ();
  virtual ~VpfTableManager ();

  virtual const VpfTable * getTable (const std::string &path);
  virtual const VpfTable * copyTable (const VpfTable * table);
  virtual void freeTable (const VpfTable * table);

private:

  static const int MAX_QUEUE_SIZE = 3; // keep up to 3 unused tables around

  void add_to_queue (const VpfTable * table);
  void remove_from_queue (const VpfTable * table);
  void cull_queue (int max_size);

  int _table_count;
  std::map<std::string,const VpfTable *> _table_map;
  std::map<std::string,int> _table_references;
  std::list<const VpfTable *> _empty_queue;
};

#endif

// end of tablemgr.hxx
