// table.hxx - declarations for VPF low-level table structures.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#ifndef __VPF_TABLE_HXX
#define __VPF_TABLE_HXX 1

#include <iostream>
#include <string>
#include <vector>

#include "vpfbase.hxx"
#include "value.hxx"



////////////////////////////////////////////////////////////////////////
// VpfTable class declaration.
////////////////////////////////////////////////////////////////////////

class VpfColumnDecl;


/**
 * A low-level view of a VPF data table.
 *
 * <p>This class provides basic row/column access to the data in a
 * VPF table.  Higher-level classes implement the logic behind the
 * different table types.  Each column in the table contains
 * declaration information available though a {@link VpfColumnDecl}
 * object, and every field contains a leaf value available through
 * a {@link VpfValue} object.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 */
class VpfTable {

public:

  /**
   * Construct a new table from a file name.
   *
   * @param fileName The path to the table file.
   * @exception VpfException If there is an error reading the table.
   */
  VpfTable (const std::string &fileName);


  /**
   * Copy a table.
   *
   * @param table The table to copy.
   */
  VpfTable (const VpfTable &table);


  /**
   * Destructor.
   */
  virtual ~VpfTable ();


  /**
   * Get the path/filename of this table.
   *
   * @return The table's filename.
   */
  virtual const std::string &getPath () const;


  /**
   * Get the table description.
   *
   * <p>This is the human-readable title provided by the table
   * creator.</p>
   *
   * @return The description provided in the table.
   */
  virtual const std::string &getDescription () const;


  /**
   * Get the name of the documentation file for this table.
   *
   * @return The documentation file name, or "-" if there is no
   * documentation file.
   */
  virtual const std::string &getDocFileName () const;


  /**
   * Test whether a column exists in a table.
   *
   * @param name The column name to test.
   * @return true if the column is present, false otherwise.
   */
  virtual bool hasColumn (const std::string &name) const;


  /**
   * Count the columns in the table.
   *
   * @return The number of columns in each row of the table.
   */
  virtual int getColumnCount () const;


  /**
   * Count the rows in the table.
   *
   * @return The number of rows in the table.
   */
  virtual int getRowCount () const;


  /**
   * Get the declaration information for a column.
   *
   * <p>This information includes the column name and type and
   * optional associated files.</p>
   *
   * @param index The zero-based index of the row.
   * @return The column's declaration information.
   * @exception VpfException If the index is out of range.
   */
  virtual const VpfColumnDecl &getColumnDecl (int index) const;


  /**
   * Get a value by row and column.
   *
   * @param row The zero-based index of the row.
   * @param column The zero-based index of the column.
   * @return The value at the specified position.
   * @exception VpfException If the row or column is out of range.
   */
  virtual const VpfValue &getValue (int row, int column) const;


  /**
   * Get a value by row and column name.
   *
   * @param row The zero-based index of the row.
   * @param columnName The name of the column.
   * @return The value at the specified position.
   * @exception VpfException If the row is out of range or there
   * is no column with the name provided.
   */
  virtual const VpfValue &getValue (int row,
				    const std::string &columName) const;


  /**
   * Given a column name, look up the column number.
   *
   * @param columnName The name of the column to look up.
   * @return The column number, or -1 if not found.
   */
  virtual int findColumn (const std::string &columnName) const;


  /**
   * Count the matches for a character array value in a column.
   *
   * @param columnName The name of the column to search.
   * @param value The value to count.
   * @return The number of times the value appears.
   * @exception VpfException If the column does not exist or does
   * not contain a character-array data type.
   */
  virtual int countMatches (const std::string &columnName,
			    const char * value) const;


  /**
   * Find a row with a column value matching a character array.
   *
   * @param columnName The name of the column to search.
   * @param value The value to search for.
   * @param index The index of the match to return (defaults to
   * 0 for the first match).
   * @return The row number containing the match, or -1 if none was
   * found.
   * @exception VpfException If the column does not exist or does
   * not contain a character-array data type.
   */
  virtual int findMatch (const std::string &columnName,
			 const char * value,
			 int index = 0) const;


  /**
   * Count the matches for an integer value in a column.
   *
   * @param columnName The name of the column to search.
   * @param value The value to count.
   * @return The number of times the value appears.
   * @exception VpfException If the column does not exist or does
   * not contain an integer data type.
   */
  virtual int countMatches (const std::string &columnName, int value) const;


  /**
   * Find a row with a column value matching an integer.
   *
   * @param columnName The name of the column to search.
   * @param value The value to search for.
   * @param index The index of the match to return (defaults to
   * 0 for the first match).
   * @return The row number containing the match, or -1 if none was
   * found.
   * @exception VpfException If the column does not exist or does
   * not contain an integer type.
   */
  virtual int findMatch (const std::string &columnName,
			 int value,
			 int index = 0) const;


protected:

  friend class VpfColumnDecl;
  friend class VpfTableManager;
  friend class VpfComponent;


  enum ByteOrder {
    MSB,
    LSB
  };

  virtual void init ();
  virtual ByteOrder get_system_byte_order () const;
  virtual void read (const std::string &fileName);
  virtual bool read_row (std::istream &input, VpfValue row[]);
//   virtual void seek_to_row (std::istream &input, int row);
  virtual short read_short (std::istream &input) const;
  virtual int read_int (std::istream &input) const;
  virtual float read_float (std::istream &input) const;
  virtual double read_double (std::istream &input) const;
  virtual int read_variable_int (std::istream &input, int type) const;

  virtual short make_short (char buf[2]) const;
  virtual int make_int (char buf[4]) const;
  virtual float make_float (char buf[4]) const;
  virtual double make_double (char buf[8]) const;


private:

  std::string _path;
  ByteOrder _system_byte_order;
  ByteOrder _file_byte_order;
  int _header_byte_size;
  std::string _description;
  std::string _doc_file_name;
  std::vector<VpfColumnDecl *> _columns;
  std::vector<VpfValue *> _rows;
};



////////////////////////////////////////////////////////////////////////
// VpfColumnDecl class declaration.
////////////////////////////////////////////////////////////////////////


/**
 * Declaration for a column in a VPF table.
 *
 * <p>This declaration applies to the same column position in all rows
 * of the table.  It specified the column name and data type and
 * whether the column contains primary or unique keys, among other
 * information.</p>
 *
 * <p>Instances of this class are managed only by the {@link VpfTable}
 * class, so there are no public constructors or destructors.</p>
 *
 * @author David Megginson, david@megginson.com
 * @version $Revision$
 */
class VpfColumnDecl {

public:

				// FIXME: there may also be an 'F'
				// type for foreign key (at least,
				// openmap thinks so).
  /**
   * The key type of the column.
   *
   * <p>A column may contain a primary key, a unique key, or a
   * non-unique key (i.e. just general data).  The primary key
   * can be used to look up a specific row.</p>
   */
  enum KeyType {
    PRIMARY_KEY,
    UNIQUE,
    NON_UNIQUE
  };


  /**
   * Get the column name.
   *
   * <p>This is the equivalent of a variable name for all
   * entries in this column.</p>
   *
   * @return The column name as a string.
   */
  virtual const std::string &getName () const;



  /**
   * Get the key type of this column (PRIMARY_KEY, UNIQUE, or NON_UNIQUE).
   *
   * @return The key type for this column.
   */
  virtual KeyType getKeyType () const;


  /**
   * Get the user-visible value type.
   *
   * @return The value type for the column.
   */
  virtual VpfValue::Type getValueType () const;



  /**
   * Get a textual description of this column.
   *
   * @return Text describing this column.
   */
  virtual const std::string &getDescription () const;


  /**
   * Get the name of the table describing this column.
   *
   * <p>Columns may optionally have an external table providing
   * a more detailed description.</p>
   *
   * @return The name of the value description table, or "-" if there
   * is none.
   */
  virtual const std::string &getValueDescriptionTable () const;


  /**
   * Get the name of the thematic index for this column.
   *
   * <p>Columns may optionally have an external table providing
   * a thematic index.</p>
   *
   * @return The name of the table containing the thematic index, or
   * "-" if there is none.
   */
  virtual const std::string &getThematicIndexName () const;


  /**
   * Get the name of the narrative table for this column.
   *
   * <p>Columns may optionally have an external table providing
   * narrative.</p>
   * 
   * @return The name of the narrative table, or "-" if there is none.
   */
  virtual const std::string &getNarrativeTable () const;


  /**
   * Get the declared size of the column
   *
   * @return The declared size of the column, or -1 for variable size.
   */
  virtual int getElementCount () const;


protected:

  friend class VpfTable;


  /**
   * Protected constructor.
   *
   * <p>This is the only way to make a column from scratch.  Users
   * will obtain references to column objects through VpfTable.</p>
   *
   * @param table The table to which the column declaration applies.
   */
  VpfColumnDecl (const VpfTable * table);


  /**
   * Protected destructor.
   *
   * <p>Only VpfTable may delete a column declaration.
   */
  virtual ~VpfColumnDecl ();


  /**
   * Read the actual header from input.
   *
   * @param input The input stream.
   * @return true if the header was read successfully.
   */
  bool read_header (std::istream &input);


  /**
   * Read a value from input.
   *
   * @param input The input stream.
   * @param value The object to hold the value.
   * @return true if a value was read successfully.
   */
  bool read_value (std::istream &input, VpfValue * value);


  /**
   * Get the column data type.
   *
   * <p>This is the only type of data that will appear in this column.
   * The types are declared in the {@link VpfValue} class.</p>
   *
   * @return The column data type.
   */
  virtual char getRawType () const;


private:

  const VpfTable * _table;
  std::string _name;
  char _raw_type;
  int _element_count;
  KeyType _key_type;
  std::string _description;
  std::string _value_description_table;
  std::string _thematic_index_name;
  std::string _narrative_table;
};


#endif

// end of table.hxx
