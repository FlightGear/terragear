// library.hxx - declaration for VpfLibrary class.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_LIBRARY_HXX
#define __VPF_LIBRARY_HXX 1

#include <string>

#include "vpfbase.hxx"

class VpfDataBase;
class VpfTable;
class VpfCoverage;


/**
 * A library in a VPF database.
 *
 * <p>This is the second level of the VPF hierarchy: every
 * database consists of one or more libraries, collections of related
 * coverages in the same area (i.e. North America, or southern
 * Ontario).  This class has a copy constructor, so it can safely be
 * assigned and passed around by value.</p>
 *
 * <p>Users should obtain a copy of the library object through the
 * database's getLibrary methods; new library objects cannot be
 * created directly (except by copying an existing one).</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision: 1.1 $
 * @see VpfDataBase
 * @see VpfCoverage
 */
class VpfLibrary : public VpfComponent
{
public:

  /**
   * Copy constructor.
   */
  VpfLibrary (const VpfLibrary &library);


  /**
   * Destructor.
   */
  virtual ~VpfLibrary ();


  /**
   * Get the name of the library.
   *
   * @return The library's name as a character string.
   */
  virtual const char * getName () const;


  /**
   * Get a description of the library.
   *
   * @return The library's description as a character string.
   */
  virtual const char * getDescription () const;


  /**
   * Get the minimum bounding rectangle of the library's coverages.
   *
   * <p>This is the smallest rectangle that can fit around all of
   * the topology in the library's coverages.</p>
   *
   * @return The minimum bounding rectangle.
   */
  virtual const VpfRectangle getBoundingRectangle () const;


  /**
   * Count the coverages available in the library.
   *
   * @return The number of coverages.
   */
  virtual int getCoverageCount () const;


  /**
   * Get a coverage by index.
   *
   * @param index The zero-based index of the coverage.
   * @return The coverage requested.
   * @exception VpfException If the index is out of range.
   * @see #getCoverageCount
   */
  virtual const VpfCoverage getCoverage (int index) const;


  /**
   * Test whether a named coverage is present.
   *
   * @param name The name of the coverage to test.
   * @return true if the coverage is present, false otherwise.
   */
  virtual bool hasCoverage (const std::string &name) const;


  /**
   * Get a coverage by name.
   *
   * @param name The name of the coverage.
   * @return The coverage requested.
   * @exception VpfException If no coverage exists with the name
   * provided.
   * @see #hasCoverage
   */
  virtual const VpfCoverage getCoverage (const std::string &name) const;


protected:

  friend class VpfDataBase;
  friend class VpfCoverage;


  /**
   * Protected constructor.
   *
   * <p>This is the only mechanism for constructing a new library
   * from scratch.  Library users should obtain a library from the
   * database class.</p>
   *
   * @param path The path to the directory containing the CAT, LHT, and
   * CRT files.
   * @param database The parent database.
   */
  VpfLibrary (const std::string &path, const VpfDataBase &database);


  /**
   * Get the library attribute table for the parent database.
   *
   * <p>The LAT contains the bounding rectangle for the library.
   *
   * @return The LAT.
   */
  const VpfTable &getLAT () const;


  /**
   * Get the coverage attribute table for this library.
   *
   * <p>The CAT lists all of the coverages available and provides
   * information about them.  This is a lazy implementation: the
   * CAT will not be loaded unless it is actually needed.</p>
   *
   * @return The CAT.
   */
  virtual const VpfTable &getCAT () const;


  /**
   * Get the library header table for this library.
   *
   * <p>The LHT contains information about the library itself, including
   * its name and description.  This is a lazy implementation: the
   * LHT will not be loaded unless it is actually needed.</p>
   *
   * @return The LHT.
   */
  virtual const VpfTable &getLHT () const;


private:

  const VpfTable * _lat;
  mutable const VpfTable * _cat;
  mutable const VpfTable * _lht;
  mutable const VpfTable * _tileref_aft;
  mutable const VpfTable * _tileref_fbr;

};

#endif

// end of library.hxx
