// value.cxx - implementation of VpfValue
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include <iostream>
#include <string>

using std::ostream;
using std::string;

#include "vpfbase.hxx"
#include "value.hxx"

VpfValue::VpfValue ()
  : _raw_type('X'),
    _element_count(0)
{
}

VpfValue::VpfValue (const VpfValue &value)
  : _raw_type('X'),
    _element_count(0)
{
  switch (value._raw_type) {
  case 'T':
  case 'L':
  case 'N':
  case 'M':
    setRawCharArray(value.getRawCharArray(),
		    value._element_count, value._raw_type);
    break;
  case 'F':
    setRawFloat(value.getRawFloat());
    break;
  case 'R':
    setRawDouble(value.getRawDouble());
    break;
  case 'S':
    setRawShort(value.getRawShort());
    break;
  case 'I':
    setRawInt(value.getRawInt());
    break;
  case 'C':
    setRawFloatXYArray(value.getRawFloatXYArray(), value._element_count);
    break;
  case 'B':
    setRawDoubleXYArray(value.getRawDoubleXYArray(), value._element_count);
    break;
  case 'Z':
    setRawFloatXYZArray(value.getRawFloatXYZArray(), value._element_count);
    break;
  case 'Y':
    setRawDoubleXYZArray(value.getRawDoubleXYZArray(), value._element_count);
    break;
  case 'D':
    setRawDateTime(value.getRawDateTime());
    break;
  case 'X':
    break;
  case 'K':
    setRawCrossTileId(value.getRawCrossTileId());
    break;
  }
}

VpfValue::~VpfValue ()
{
  clear();
}

VpfValue::Type
VpfValue::getType () const
{
  return convertType(_raw_type);
}

int
VpfValue::getElementCount () const
{
  return _element_count;
}

const char *
VpfValue::getText () const
{
  return getRawCharArray();
}

int
VpfValue::getInt () const
{
  switch(_raw_type) {
  case 'S':
    return getRawShort();
  case 'I':
    return getRawInt();
  default:
    throw VpfException("Type not 'S' or 'I'");
  }
}

double
VpfValue::getReal () const
{
  switch(_raw_type) {
  case 'F':
    return getRawFloat();
  case 'R':
    return getRawDouble();
  default:
    throw VpfException("Type not 'F' or 'R'");
  }
}

const VpfPoint
VpfValue::getPoint (int index) const
{
  VpfPoint result;
  if (index < 0 || index >= _element_count)
    throw VpfException("Index out of range");
  switch (_raw_type) {
  case 'C':
    result.x = _raw_value.float_xy_array[index].x;
    result.y = _raw_value.float_xy_array[index].y;
    result.z = 0.0;
    break;
  case 'B':
    result.x = _raw_value.double_xy_array[index].x;
    result.y = _raw_value.double_xy_array[index].y;
    result.z = 0.0;
    break;
  case 'Z':
    result.x = _raw_value.float_xyz_array[index].x;
    result.y = _raw_value.float_xyz_array[index].y;
    result.z = _raw_value.float_xyz_array[index].z;
    break;
  case 'Y':
    result.x = _raw_value.double_xyz_array[index].x;
    result.y = _raw_value.double_xyz_array[index].y;
    result.z = _raw_value.double_xyz_array[index].z;
    break;
  default:
    throw VpfException("Not an array of coordinates");
  }
  return result;
}

const char *
VpfValue::getDate () const
{
  return getRawDateTime();
}

const VpfCrossRef
VpfValue::getCrossRef () const
{
  return getRawCrossTileId();
}

char
VpfValue::getRawType () const
{
  return _raw_type;
}

void
VpfValue::assertRawType (char type) const
{
  if (_raw_type != type)
    throw VpfException("Type mismatch");
}

const char *
VpfValue::getRawCharArray () const
{
  switch (_raw_type) {
  case 'T':
  case 'L':
  case 'N':
  case 'M':
    return _raw_value.char_array;
  default:
    throw VpfException("Type mismatch");
  }
}

short
VpfValue::getRawShort () const
{
  assertRawType('S');
  return _raw_value.short_value;
}

int
VpfValue::getRawInt () const
{
  assertRawType('I');
  return _raw_value.int_value;
}

float
VpfValue::getRawFloat () const
{
  assertRawType('F');
  return _raw_value.float_value;
}

double
VpfValue::getRawDouble () const
{
  assertRawType('R');
  return _raw_value.double_value;
}

const VpfValue::float_xy *
VpfValue::getRawFloatXYArray () const
{
  assertRawType('C');
  return _raw_value.float_xy_array;
}

const VpfValue::double_xy *
VpfValue::getRawDoubleXYArray () const
{
  assertRawType('B');
  return _raw_value.double_xy_array;
}

const VpfValue::float_xyz *
VpfValue::getRawFloatXYZArray () const
{
  assertRawType('Z');
  return _raw_value.float_xyz_array;
}

const VpfValue::double_xyz *
VpfValue::getRawDoubleXYZArray () const
{
  assertRawType('Y');
  return _raw_value.double_xyz_array;
}

const char *
VpfValue::getRawDateTime () const
{
  assertRawType('D');
  return _raw_value.char_array;
}

const VpfCrossRef &
VpfValue::getRawCrossTileId () const
{
  assertRawType('K');
  return *(_raw_value.cross_tile_value);
}

void
VpfValue::setNull ()
{
  clear();
  _raw_type = 'X';
}

void
VpfValue::setRawCharArray (const char * array, int size, char type)
{
  clear();
  switch (type) {
  case 'T':
  case 'L':
  case 'N':
  case 'M':
    _raw_type = type;
    break;
  default:
    throw VpfException("Illegal type for character array");
  }
  _element_count = size;
  int i;
				// strip trailing spaces
  for (i = size - 1; i > 0; i--) {
    if (array[i] == ' ')
      _element_count--;
    else
      break;
  }
				// Add an extra byte for final null
  _raw_value.char_array = new char[_element_count+1];
  for (i = 0; i < _element_count; i++)
    _raw_value.char_array[i] = array[i];
  _raw_value.char_array[_element_count] = '\0';
}

void
VpfValue::setRawShort (short value)
{
  clear();
  _raw_type = 'S';
  _element_count = 1;
  _raw_value.short_value = value;
}

void
VpfValue::setRawInt (int value)
{
  clear();
  _raw_type = 'I';
  _element_count = 1;
  _raw_value.int_value = value;
}

void
VpfValue::setRawFloat (float value)
{
  clear();
  _raw_type = 'F';
  _element_count = 1;
  _raw_value.float_value = value;
}

void
VpfValue::setRawDouble (double value)
{
  clear();
  _raw_type = 'R';
  _element_count = 1;
  _raw_value.double_value = value;
}

void
VpfValue::setRawFloatXYArray (const float_xy * array, int size)
{
  clear();
  _raw_type = 'C';
  _element_count = size;
  _raw_value.float_xy_array = new float_xy[size];
  for (int i = 0; i < size; i++) {
    _raw_value.float_xy_array[i].x = array[i].x;
    _raw_value.float_xy_array[i].y = array[i].y;
  }
}

void
VpfValue::setRawDoubleXYArray (const double_xy * array, int size)
{
  clear();
  _raw_type='B';
  _element_count = size;
  _raw_value.double_xy_array = new double_xy[size];
  for (int i = 0; i < size; i++) {
    _raw_value.double_xy_array[i].x = array[i].x;
    _raw_value.double_xy_array[i].y = array[i].y;
  }
}

void
VpfValue::setRawFloatXYZArray (const float_xyz * array, int size)
{
  clear();
  _raw_type = 'Z';
  _element_count = size;
  _raw_value.float_xyz_array = new float_xyz[size];
  for (int i = 0; i < size; i++) {
    _raw_value.float_xyz_array[i].x = array[i].x;
    _raw_value.float_xyz_array[i].y = array[i].y;
    _raw_value.float_xyz_array[i].z = array[i].z;
  }
}

void
VpfValue::setRawDoubleXYZArray (const double_xyz * array, int size)
{
  clear();
  _raw_type = 'Y';
  _element_count = size;
  _raw_value.double_xyz_array = new double_xyz[size];
  for (int i = 0; i < size; i++) {
    _raw_value.double_xyz_array[i].x = array[i].x;
    _raw_value.double_xyz_array[i].y = array[i].y;
    _raw_value.double_xyz_array[i].z = array[i].z;
  }
}

void
VpfValue::setRawDateTime (const char * array)
{
  clear();
  _raw_type = 'D';
  _element_count = 1;
  _raw_value.char_array = new char[20];
  for (int i = 0; i < 20; i++)
    _raw_value.char_array[i] = array[i];
}

void
VpfValue::setRawCrossTileId (const VpfCrossRef &id)
{
  clear();
  _raw_type = 'K';
  _element_count = 1;
  _raw_value.cross_tile_value = new VpfCrossRef;
  _raw_value.cross_tile_value->current_tile_key = id.current_tile_key;
  _raw_value.cross_tile_value->next_tile_id = id.next_tile_id;
  _raw_value.cross_tile_value->next_tile_key = id.next_tile_key;
  _raw_value.cross_tile_value->unused_key = id.unused_key;
}

void
VpfValue::clear ()
{
  switch (_raw_type) {
  case 'T':
  case 'L':
  case 'N':
  case 'M':
    delete[] _raw_value.char_array;
    _raw_value.char_array = 0;
    break;
  case 'F':
  case 'R':
  case 'S':
  case 'I':
    break;
  case 'C':
    delete[] _raw_value.float_xy_array;
    _raw_value.float_xy_array = 0;
    break;
  case 'B':
    delete[] _raw_value.double_xy_array;
    _raw_value.double_xy_array = 0;
    break;
  case 'Z':
    delete[] _raw_value.float_xyz_array;
    _raw_value.float_xyz_array = 0;
    break;
  case 'Y':
    delete[] _raw_value.double_xyz_array;
    _raw_value.double_xyz_array = 0;
    break;
  case 'D':
    delete [] _raw_value.char_array;
    _raw_value.char_array = 0;
    break;
  case 'X':
    break;
  case 'K':
    delete _raw_value.cross_tile_value;
    _raw_value.cross_tile_value = 0;
    break;
  }
  _raw_type = 'X';
}

VpfValue::Type
VpfValue::convertType (char rawType)
{
  switch (rawType) {
  case 'T':
  case 'L':
  case 'N':
  case 'M':
    return TEXT;
  case 'F':
  case 'R':
    return REAL;
  case 'S':
  case 'I':
    return INT;
  case 'C':
  case 'B':
  case 'Z':
  case 'Y':
    return POINTS;
  case 'D':
    return DATE;
  case 'X':
    return EMPTY;
  case 'K':
    return CROSSREF;
  default:
    throw VpfException(string("Unknown raw value type: ") + rawType);
  }
}

ostream &
operator<< (ostream &output, const VpfValue &value)
{
  switch (value.getType()) {
  case VpfValue::TEXT:
    output << value.getText();
    break;
  case VpfValue::INT:
    output << value.getInt();
    break;
  case VpfValue::REAL:
    output << value.getReal();
    break;
  case VpfValue::POINTS: {
    int nPoints = value.getElementCount();
    for (int i = 0; i < nPoints; i++) {
      if (i > 0)
	output << ',';
      output << '{' << value.getPoint(i).x << ','
	     << value.getPoint(i).y << ',' << value.getPoint(i).z << '}';
    }
    break;
  }
  case VpfValue::DATE:
    output << value.getDate();	// FIXME
    break;
  case VpfValue::CROSSREF:
    output << value.getCrossRef().current_tile_key << ','
	   << value.getCrossRef().next_tile_id << ','
	   << value.getCrossRef().next_tile_key;
    break;
  }

  return output;
}

// end of value.cxx
