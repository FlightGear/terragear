// component.hxx - declaration of VpfComponent base class
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_COMPONENT_HXX
#define __VPF_COMPONENT_HXX 1

#include "vpfbase.hxx"

#include <string>

class VpfTable;
class VpfTableManager;


/**
 * Base class for hierarchical VPF components.
 *
 * <p>This class provides simple, base functionality for all components,
 * especially path-related operations such as opening and testing
 * for tables.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 */
class VpfComponent
{
public:

  /**
   * Get the base directory for this component.
   *
   * @return The base directory as a string.
   */
  virtual const std::string &getPath () const;


protected:


  /**
   * Protected constructor.
   *
   * <p>Only subclasses should invoke this constructor.</p>
   *
   * @param table_manager The underlying table manager.
   * @param path The base path for the component.
   */
  VpfComponent (VpfTableManager &table_manager,
		const std::string &path);


  /**
   * Destructor.
   */
  virtual ~VpfComponent ();

#ifdef __MSDOS__
  static const char PATHSEP = '\\';
#else
  static const char PATHSEP = '/';	// FIXME: make more robust
#endif


  /**
   * Test if this component has a child file.
   *
   * <p>This method tests for the file relative to the component's
   * path.  The current implementation is very crude, but
   * portable; note, however, that it will fail if the child is
   * a directory rather than a file.</p>
   *
   * @param child The relative file name, which must not contain
   * any path separators.
   * @return true if the file exists and can be read, false otherwise.
   * @see #hasFile
   */
  virtual bool hasChildFile (const std::string &child) const;


  /**
   * Get the absolute name of a child file.
   *
   * <p>This method can be used with a file or directory.  The name
   * returned is relative to the component's path.</p>
   *
   * @param child The relative file or directory name, which must not
   * contain any path separators.
   * @return The absolute file or directory name.
   * @see #getFileName
   */
  virtual std::string getChildFileName (const std::string &child) const;


  /**
   * Test for the existence of a file.
   *
   * <p>This method tests for the existence of a file inside a
   * directory and creates a new absolute file name using the
   * appropriate path separator.  Note that this method will fail if
   * the relative path points to a directory rather than a file.</p>
   *
   * @param path The partial path leading up to the file or directory.
   * @param file The relative name of the file or directory, which
   * must not contain any path separators.
   * @return true if the file exists, false otherwise.
   */
  virtual bool hasFile (const std::string &path,
			const std::string &file) const;


  /**
   * Get the absolute name for a file.
   *
   * <p>This method builds the absolute name of a file or directory
   * inside any arbitrary directory, using the appropriate path
   * separator.  Invocations can be nested to get a complex path.</p>
   *
   * @param path The partial path leading up to the component.
   * @param file The relative name of the file or directory, which
   * must not contain any path separators.
   * @return The absolute file name.
   */
  virtual std::string getFileName (const std::string &path,
				   const std::string &file) const;


  /**
   * Get a VPF table based on an absolute pathname.
   *
   * <p>The application must call {@link #freeTable} to release the
   * table once it is finished with it.  It must <em>not</em>
   * delete the table.</p>
   *
   * @param path The absolute pathname to the VPF table.
   * @return A pointer to a table from the table manager.
   * @exception VpfException If the table cannot be read.
   */
  virtual const VpfTable * getTable (const std::string &path) const;


  /**
   * Get a VPF table based on a relative filename.
   *
   * <p>The table will be loaded from the component's base directory.</p>
   *
   * <p>The application must call {@link #freeTable} to release the
   * table once it is finished with it.  It must <em>not</em>
   * delete the table.</p>
   *
   * @param file The table's relative file name, which must not
   * contain any path separators.
   * @return A pointer to a table from the table manager.
   * @exception VpfException If the table cannot be read.
   */
  virtual const VpfTable * getChildTable (const std::string &file) const;


  /**
   * Create a new copy of an existing table.
   *
   * <p>This method currently increases the reference counter in the
   * table manager.</p>
   *
   * <p>The application must call {@link #freeTable} to release the
   * table once it is finished with it.  It must <em>not</em>
   * delete the table.</p>
   *
   * @param table The table to copy.
   * @return A new copy of the table.
   */
  virtual const VpfTable * copyTable (const VpfTable * table) const;


  /**
   * Release a table.
   *
   * <p>This method currently decreases the reference counter in the
   * table manager, which may cause the table to be freed.  The
   * application must not attempt to use this copy of the table
   * after freeing it.</p>
   *
   * @param table The table to copy.
   */
  virtual void freeTable (const VpfTable * table) const;

  virtual VpfTableManager &getTableManager () const;


private:
  std::string _path;
  mutable VpfTableManager &_table_manager;

};


#endif

// end of component.hxx
