// database.hxx - declaration for VpfDataBase
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_DATABASE_HXX
#define __VPF_DATABASE_HXX 1

#include <string>
#include <map>
#include <list>

#include "vpfbase.hxx"
#include "component.hxx"
#include "tablemgr.hxx"


class VpfTableManager;
class VpfLibrary;

/**
 * A Vector Product Format (VPF) database.
 *
 * <p>This is the top level of the VPF hierarchy: the database contains
 * one or more libraries, which in turn contain coverages of various
 * sorts.  This class has a copy constructor, so it can safely be
 * assigned and passed around by value.</p>
 *
 * <p>This is the only class that users should create directly using a
 * public constructor, providing the path to the root of the database
 * in the file system.  This class can be used to create library
 * objects, which, in turn, create coverage objects, and so on.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 * @see VpfDataBase
 * @see VpfCoverage
 */
class VpfDataBase : public VpfComponent
{
public:


  /**
   * Public constructor.
   *
   * <p>This constructor is the main mechanism for creating a VPF
   * database.  The user provides it with a path to the root of the
   * VPF database, the name of a directory containing "dht" and "lat"
   * files, such as "/cdrom/vmaplv0/" or "d:\\vmaplv0\\".</p>
   *
   * @param path The directory at the root of the VPF database.
   */
  VpfDataBase (const std::string &path);


  /**
   * Destructor.
   */
  virtual ~VpfDataBase ();


  /**
   * Copy constructor.
   */
  VpfDataBase (const VpfDataBase &database);


  /**
   * Get the name of the database.
   *
   * @return The database's name as a character string.
   */
  virtual const char * getName () const;


  /**
   * Get a description of the database.
   *
   * @return The database's description as a character string.
   */
  virtual const char * getDescription () const;


  /**
   * Count the libraries in the database.
   *
   * @return The number of libraries in the database.
   */
  virtual int getLibraryCount () const;


  /**
   * Get a library by index.
   *
   * @param index The zero-based index of the library.
   * @return The library.
   * @exception VpfException If the library is out of range.
   * @see #getLibraryCount
   */
  virtual const VpfLibrary getLibrary (int index) const;


  /**
   * Test whether a library is present.
   *
   * @param name The library name to test.
   * @return true if a library exists with the name specified, false
   * otherwise.
   */
  virtual bool hasLibrary (const std::string &name) const;


  /**
   * Get a library by name.
   *
   * @param name The library's name.
   * @return The library.
   * @exception VpfException If there is no library with the
   * name provided.
   * @see #hasLibrary
   */
  virtual const VpfLibrary getLibrary (const std::string &name) const;


protected:

  friend class VpfLibrary;


  /**
   * Get a copy of the database header table.
   *
   * <p>The DHT contains information about the database, including
   * its name and description.  This is a lazy implementation: it will
   * not load the DHT table unless it is actually needed.</p>
   *
   * @return The DHT table.
   */
  virtual const VpfTable &getDHT () const;


  /**
   * Get a copy of the library attribute table.
   *
   * <p>The LAT contains the name and bounding rectangle for every
   * library in the database.  This is a lazy implementation: it will
   * not load the LAT table unless it is actually needed.</p>
   *
   * @return the LAT table.
   */
  virtual const VpfTable &getLAT () const;

private:

  mutable VpfTableManager _table_manager;
  mutable const VpfTable * _dht;
  mutable const VpfTable * _lat;
};


#endif // __VPF_DATABASE_HXX

// end of database.hxx
