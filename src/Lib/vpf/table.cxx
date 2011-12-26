// table.cxx - implementation of VpfTable
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include <simgear/compiler.h>

#include <string>
#include <fstream>

#include <simgear/misc/zfstream.hxx>	// ios_binary

#include <stdlib.h>

#include "vpfbase.hxx"
#include "table.hxx"

using std::string;
using std::ifstream;
using std::istream;



////////////////////////////////////////////////////////////////////////
// Static I/O methods.
////////////////////////////////////////////////////////////////////////


/**
 * Test whether the local system uses MSB or LSB encoding.
 */
VpfTable::ByteOrder
VpfTable::get_system_byte_order () const
{
  int test = 0x01020304;
  char * b = (char *)(&test);
  if (b[0] == 0x01 && b[1] == 0x02 && b[2] == 0x03 && b[3] == 0x04)
    return VpfTable::MSB;
  else if (b[0] == 0x04 && b[1] == 0x03 && b[2] == 0x02 && b[3] == 0x01)
    return VpfTable::LSB;
  else
    throw VpfException("Unsupported byte encoding in local system");
}


static void
swap2 (char in[2], char out[2])
{
  out[0] = in[1] ;
  out[1] = in[0] ;
}

static void
swap4 (char in[4], char out[4])
{
  out[0] = in[3] ;
  out[1] = in[2] ;
  out[2] = in[1] ;
  out[3] = in[0] ;
}

static void
swap8 (char in[8], char out[8])
{
  out[0] = in[7] ;
  out[1] = in[6] ;
  out[2] = in[5] ;
  out[3] = in[4] ;
  out[4] = in[3] ;
  out[5] = in[2] ;
  out[6] = in[1] ;
  out[7] = in[0] ;
}


/**
 * Read a fixed number of characters into a buffer.
 *
 * @param input The input stream.
 * @param buffer The buffer to receive the characters.
 * @param length The number of characters to read.
 * @exception VpfException if the specified number of characters
 *             cannot be read from the stream.
 */
static void
read_chars (istream &input, char * buffer, int length)
{
  if (!input.read(buffer, length))
    throw VpfException("Failure reading from file");
}


/**
 * Read a single character.
 *
 * @param input The input stream.
 * @return A single character.
 * @exception VpfException if one character cannot be read from the 
 * stream.
 */
static char
read_char (istream &input)
{
  char c;
  read_chars(input, &c, 1);
  return c;
}


/**
 * Read up to a delimiter.
 *
 * @param input The input stream.
 * @param delim The delimiter character.
 * @exception VpfException if the delimiter character cannot be found
 *            or if there is an error reading from the stream.
 */
static string
read_to_delim (istream &input, char delim)
{
  string result;
  char c = read_char(input);
  while (c != delim) {
    result += c;
    c = read_char(input);
  }
  return result;
}


/**
 * Read text from a column description.
 *
 * @param input The input stream.
 * @exception VpfException if there is an error reading the text.
 */
static string
read_column_text (istream &input)
{
  string result;
  char c = read_char(input);
  while (c != ',') {
    if (c == ':')		// FIXME: what if there was some text first?
      throw true;
    if (c == '\\')
      c = read_char(input);
    result += c;
    c = read_char(input);
  }
  return result;
}


/**
 * Test that an expected character appears.
 *
 * @param actual The character that appeared.
 * @param expected The character that was expected.
 * @exception VpfException If the actual character is not the same as
 *            the expected one.
 */
static void
assert_char (char actual, char expected)
{
  if (actual != expected) {
    string message = "Expected '";
    message += expected;
    message += "' but found '";
    message += actual;
    message += '\'';
    throw VpfException(message);
  }
}


/**
 * Read a number of varying length.
 *
 * @param input The input stream.
 * @param type The number type (0=none, 1=byte, 2=short, 3=int).
 * @return The number, or -1 for an empty field.
 */
int
VpfTable::read_variable_int (istream &input, int type) const
{
  switch (type) {
  case 0:
    return -1;
  case 1:
    return (unsigned char)read_char(input);
  case 2:
    return (unsigned short)read_short(input);
  case 3:
    return read_int(input);
  default:
    throw VpfException("Internal error reading variable integer");
  }
}


////////////////////////////////////////////////////////////////////////
// Implementation of VpfTable
////////////////////////////////////////////////////////////////////////

VpfTable::VpfTable (const string &fileName)
  : _path(fileName),
    _system_byte_order(LSB),
    _file_byte_order(LSB),
    _header_byte_size(-1),
    _description(""),
    _doc_file_name("")
{
  init();
  read(_path);
}

VpfTable::VpfTable (const VpfTable &table)
  : _path(table._path),
    _system_byte_order(table._system_byte_order),
    _file_byte_order(table._file_byte_order),
    _header_byte_size(table._header_byte_size),
    _description(table._description),
    _doc_file_name(table._doc_file_name)
{
  init();
  read(_path);
}


void
VpfTable::init ()
{
  _system_byte_order = get_system_byte_order();
}

VpfTable::~VpfTable ()
{
  int len = _columns.size();
  int i;
  for (i = 0; i < len; i++) {
    VpfColumnDecl * tmp = _columns[i];
    _columns[i] = 0;
    delete tmp;
  }
  len = _rows.size();
  for (i = 0; i < len; i++) {
    VpfValue * tmp = _rows[i];
    _rows[i] = 0;
    delete[] tmp;
  }
}

const string &
VpfTable::getPath () const
{
  return _path;
}

const string &
VpfTable::getDescription () const
{
  return _description;
}

const string &
VpfTable::getDocFileName () const
{
  return _doc_file_name;
}

bool
VpfTable::hasColumn (const string &name) const
{
  return (findColumn(name) != -1);
}

int
VpfTable::getColumnCount () const
{
  return _columns.size();
}

int
VpfTable::getRowCount () const
{
  return _rows.size();
}

const VpfColumnDecl &
VpfTable::getColumnDecl (int index) const
{
  return *(_columns[index]);
}

void
VpfTable::read (const string &fileName)
{
  _path = fileName;
  ifstream input;

  input.open(fileName.c_str(), ios_binary);
  if (!input) {
    input.clear();
    string fileName2 = fileName + '.';
    input.open(fileName2.c_str(), ios_binary);
  }
  if (!input)
    throw VpfException(string("Failed to open VPF table file ") + fileName);

				// Read the length as raw bytes
				// (we don't know byte-order yet).
  char rawLen[4];
  read_chars(input, rawLen, 4);

				// Now, check the byte order
  char c = read_char(input);
  switch (c) {
  case 'L':
  case 'l':
    c = read_char(input);
				// fall through...
  case ';':
    _file_byte_order = LSB;
    break;
  case 'M':
  case 'm':
    c = read_char(input);
    _file_byte_order = MSB;
    break;
  default:
    throw VpfException("Unknown byte-order marker");
  }

  assert_char(c, ';');

  _header_byte_size = make_int(rawLen) + 4; // for initial integer

  _description = read_to_delim(input, ';');

  _doc_file_name = read_to_delim(input, ';');

				// Read all of the column defs
  while (true) {
    VpfColumnDecl * col = new VpfColumnDecl(this);
    if (!col->read_header(input))
      break;
    _columns.push_back(col);
  }

  if (!input.seekg(_header_byte_size))
    throw VpfException("Failed to seek past header");

				// Ignore variable-length indices for
				// now, since we don't need random
				// access.
  VpfValue * row = new VpfValue[getColumnCount()];
  while (read_row(input, row)) {
    _rows.push_back(row);
    row = new VpfValue[getColumnCount()];
  }
  delete[] row;

  input.close();
}

bool
VpfTable::read_row (istream &input, VpfValue * row)
{
				// FIXME: excessively crude test for EOF
  char c;
  if (!input.get(c))
    return false;
  else
#if !defined(SG_HAVE_NATIVE_SGI_COMPILERS)
    input.unget();
#else
    input.putback(c);
#endif
				// OK, continue
  int nCols = _columns.size();
  for (int i = 0; i < nCols; i++) {
    _columns[i]->read_value(input, &(row[i]));
  }
  return true;
}

const VpfValue &
VpfTable::getValue (int row, int column) const
{
  if (row < 0 || row >= getRowCount())
    throw VpfException("Row index out of range");
  else if (column < 0 || column >= getColumnCount())
    throw VpfException("Column index out of range");
  else
    return _rows[row][column];
}

const VpfValue &
VpfTable::getValue (int row, const string &columnName) const
{
  int column = findColumn(columnName);
  if (column == -1)
    throw VpfException(string("Column name not found: ") + columnName);
  else
    return getValue(row, column);
}

int
VpfTable::findColumn (const string &columnName) const
{
  int nColumns = getColumnCount();
  for (int i = 0; i < nColumns; i++) {
    if (columnName == getColumnDecl(i).getName())
      return i;
  }
  return -1;
}

int
VpfTable::countMatches (const string &columnName, const char * value) const
{
  int column = findColumn(columnName);
  if (column == -1)
    throw VpfException("Column does not exist");
  int result = 0;
  int nRows = getRowCount();
  for (int i = 0; i < nRows; i++) {
    if (string(value) == getValue(i, column).getText())
      result++;
  }
  return result;
}

int
VpfTable::findMatch (const string &columnName, const char * value,
		     int index) const
{
  int column = findColumn(columnName);
  if (column == -1)
    throw VpfException("Column does not exist");
  int result = -1;

  int nRows = getRowCount();
  for (int i = 0; i < nRows; i++) {
    if (string(value) == getValue(i, column).getText()) {
      if (index == 0)
	return i;
      else
	index--;
    }
  }
  return result;
}

int
VpfTable::countMatches (const string &columnName, int value) const
{
  int column = findColumn(columnName);
  if (column == -1)
    throw VpfException("Column does not exist");
  int result = 0;
  int nRows = getRowCount();
  for (int i = 0; i < nRows; i++) {
    if (value == getValue(i, column).getInt())
      result++;
  }
  return result;
}

int
VpfTable::findMatch (const string &columnName, int value, int index) const
{
				// START KLUDGE
  if (columnName == "id" &&
      hasColumn("id") && 
      index == 0 &&
      value >= 0 &&
      value < getRowCount() &&
      value == getValue(value-1, "id").getInt())
    return value - 1;
				// END KLUDGE
  int column = findColumn(columnName);
  if (column == -1)
    throw VpfException("Column does not exist");
  int result = -1;
  int nRows = getRowCount();

  for (int i = 0; i < nRows; i++) {
    if (value == getValue(i, column).getRawInt()) {
      if (index == 0)
	return i;
      else
	index--;
    }
  }
  return result;
}

short
VpfTable::read_short (istream &input) const
{
  char buf[2];
  read_chars(input, buf, 2);
  return make_short(buf);
}

int
VpfTable::read_int (istream &input) const
{
  char buf[4];
  read_chars(input, buf, 4);
  return make_int(buf);
}

float
VpfTable::read_float (istream &input) const
{
  char buf[4];
  read_chars(input, buf, 4);
  return make_float(buf);
}

double
VpfTable::read_double (istream &input) const
{
  char buf[8];
  read_chars(input, buf, 8);
  return make_double(buf);
}

short
VpfTable::make_short (char buf[2]) const
{
    if (_system_byte_order == _file_byte_order) {
        return *((short *)buf);
    } else {
        char out[2];
        short *out_short = (short *)out;

        swap2(buf, out);
        return *out_short;
    }
}

int
VpfTable::make_int (char buf[4]) const
{
    if (_system_byte_order == _file_byte_order) {
        return *((int *)buf);
    } else {
        char out[4];
        int *int_out = (int *)out;

        swap4(buf, out);
        return *int_out;
    }
}

float
VpfTable::make_float (char buf[4]) const
{
    if (_system_byte_order == _file_byte_order) {
        return *((float *)buf);
    } else {
        char out[4];
        float *float_out = (float *)out;

        swap4(buf, out);
        return *float_out;
    }
}

double
VpfTable::make_double (char buf[8]) const
{
    if (_system_byte_order == _file_byte_order) {
        return *((double *)buf);
    } else {
        char out[8];
        double *double_out = (double *)out;

        swap8(buf, out);
        return *double_out;
    }
}



////////////////////////////////////////////////////////////////////////
// Implementation of VpfColumnDecl
////////////////////////////////////////////////////////////////////////

VpfColumnDecl::VpfColumnDecl (const VpfTable * table)
  : _table(table),
    _description(""),
    _value_description_table("-"),
    _thematic_index_name("-"),
    _narrative_table("-")
{
}

VpfColumnDecl::~VpfColumnDecl ()
{
}

const string &
VpfColumnDecl::getName () const
{
  return _name;
}

VpfColumnDecl::KeyType
VpfColumnDecl::getKeyType () const
{
  return _key_type;
}

VpfValue::Type
VpfColumnDecl::getValueType () const
{
  return VpfValue::convertType(_raw_type);
}

const string &
VpfColumnDecl::getDescription () const
{
  return _description;
}

const string &
VpfColumnDecl::getValueDescriptionTable () const
{
  return _value_description_table;
}

const string &
VpfColumnDecl::getThematicIndexName () const
{
  return _thematic_index_name;
}

const string &
VpfColumnDecl::getNarrativeTable () const
{
  return _narrative_table;
}


bool
VpfColumnDecl::read_header (istream &input)
{
  char c = read_char(input);
  if (c == ';')
    return false;
  _name = "";
  _name += c;
  _name += read_to_delim(input, '=');

  _raw_type = read_char(input);
  c = read_char(input);
  assert_char(c, ',');

  string length_string = read_to_delim(input, ',');
  if (length_string == "*")
    _element_count = -1;
  else
    _element_count = atoi(length_string.c_str());

				// Not allowed in an array...
  if (_element_count != 1) {
    switch (_raw_type) {
    case 'F': 
    case 'R': 
    case 'S': 
    case 'I': 
    case 'D': 
    case 'X': 
    case 'K': 
      throw VpfException("Illegal array type");
    }
  }

  string keyType;
  try {
    keyType = read_column_text(input);
  } catch (bool end) {		// FIXME: what is this is only field???
    throw VpfException("required key type field missing");
  }
  if (keyType.size() != 1)
    throw VpfException("Key type should be length 1");
  switch (keyType[0]) {
  case 'P':
    _key_type = PRIMARY_KEY;
    break;
  case 'U':
    _key_type = UNIQUE;
    break;
  case 'N':
    _key_type = NON_UNIQUE;
    break;
  default:
    throw VpfException("Unrecognized key type");
  }

				// FIXME: test for end of record
  try {
    _description = read_column_text(input);
  } catch (bool end) {
    return true;
  }

  try {
    _value_description_table = read_column_text(input);
  } catch (bool end) {
    return true;
  }
  // fixme: convert to lower case

  try {
    _thematic_index_name = read_column_text(input);
  } catch (bool end) {
    return true;
  }
  // fixme: convert to lower case

  try {
    _narrative_table = read_column_text(input);
  } catch (bool end) {
    return true;
  }
  // fixme: convert to lower case

  c = read_char(input);
  assert_char(c, ':');

  return true;
}


bool
VpfColumnDecl::read_value (istream &input, VpfValue * value)
{
  switch (_raw_type) {
  case 'T':
  case 'L':
  case 'N':
  case 'M': {
    int length = getElementCount();
    if (length == -1)
      length = _table->read_int(input);
    char *buf = new char[length]; // FIXME: inefficient
    read_chars(input, buf, length);
    value->setRawCharArray(buf, length, _raw_type);
    delete[] buf;
    break;
  };
  case 'F':
    value->setRawFloat(_table->read_float(input));
    break;
  case 'R':
    value->setRawDouble(_table->read_double(input));
    break;
  case 'S':
    value->setRawShort(_table->read_short(input));
    break;
  case 'I':
    value->setRawInt(_table->read_int(input));
    break;
  case 'C': {
    int length = getElementCount();
    if (length == -1)
      length = _table->read_int(input);
    VpfValue::float_xy * buf = new VpfValue::float_xy[length];
    for (int i = 0; i < length; i++) {
      buf[i].x = _table->read_float(input);
      buf[i].y = _table->read_float(input);
    }
    value->setRawFloatXYArray(buf, length);
    delete[] buf;
    break;
  };
  case 'B': {
    int length = getElementCount();
    if (length == -1)
      length = _table->read_int(input);
    VpfValue::double_xy * buf = new VpfValue::double_xy[length];
    for (int i = 0; i < length; i++) {
      buf[i].x = _table->read_double(input);
      buf[i].y = _table->read_double(input);
    }
    value->setRawDoubleXYArray(buf, length);
    delete[] buf;
    break;
  };
  case 'Z': {
    int length = getElementCount();
    if (length == -1)
      length = _table->read_int(input);
    VpfValue::float_xyz * buf = new VpfValue::float_xyz[length];
    for (int i = 0; i < length; i++) {
      buf[i].x = _table->read_float(input);
      buf[i].y = _table->read_float(input);
      buf[i].z = _table->read_float(input);
    }
    value->setRawFloatXYZArray(buf, length);
    delete[] buf;
    break;
  };
  case 'Y': {
    int length = getElementCount();
    if (length == -1)
      length = _table->read_int(input);
    VpfValue::double_xyz * buf = new VpfValue::double_xyz[length];
    for (int i = 0; i < length; i++) {
      buf[i].x = _table->read_double(input);
      buf[i].y = _table->read_double(input);
      buf[i].z = _table->read_double(input);
    }
    value->setRawDoubleXYZArray(buf, length);
    delete[] buf;
    break;
  };
  case 'D': {
    char *buf = new char[20]; // FIXME: inefficient
    read_chars(input, buf, 20);
    value->setRawDateTime(buf);
    delete[] buf;
    break;
  };
  case 'X':
    value->setNull();
    break;
  case 'K': {
    VpfCrossRef id;
    unsigned char length_info = (unsigned char)read_char(input);
    id.current_tile_key =
      _table->read_variable_int(input, (length_info&0xC0)>>6);
    id.next_tile_id =
      _table->read_variable_int(input, (length_info&0x30)>>4);
    id.next_tile_key =
      _table->read_variable_int(input, (length_info&0x0C)>>2);
    id.unused_key = _table->read_variable_int(input, length_info&0x03);
    value->setRawCrossTileId(id);
    break;
  };
  default:
    throw VpfException("Internal error: bad type");
  }
  return true;
}

char
VpfColumnDecl::getRawType () const
{
  return _raw_type;
}

int
VpfColumnDecl::getElementCount () const
{
  return _element_count;
}


// end of table.cxx
