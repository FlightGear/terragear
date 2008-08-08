// landcover.cxx - implementation of the LandCover class
// Started 2000-10-25 by David Megginson, david@megginson.com

// This program is in the Public Domain and comes with NO WARRANTY.
// Use at your own risk.

#include <simgear/compiler.h>
#include <string>


#include "landcover.hxx"

using std::ifstream;
using std::string;

LandCover::LandCover( const string &filename )
{
    // MSVC chokes when these are defined and initialized as "static
    // const long" in the class declaration.u
    WIDTH = 43200;
    HEIGHT = 21600;

    _input = new ifstream(filename.c_str());
    if (!_input->good())  {
#ifdef _MSC_VER
	// there are no try or catch statements to support
	// the throw-expression except in test-landcover.cxx
	printf( "Failed to open %s\n", filename.c_str() );
	exit( 1 );
#else
	throw (string("Failed to open ") + filename);
#endif
    }
}

LandCover::~LandCover ()
{
  _input->close();
  delete _input;
}

int
LandCover::getValue (long x, long y) const
{
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
    return -1;			// TODO: exception

  long offset = x + (y * WIDTH);
  _input->seekg(offset);
  if (!_input->good())
    throw string("Failed to seek to position");
  int value = _input->get();
  if (!_input->good())
    throw string("Failed to read character");
  return value;
}

int
LandCover::getValue (double lon, double lat) const
{
  if (lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0)
    return -1;			// TODO: exception

  long x = long((lon + 180.0) * 120.0);
  long y = HEIGHT - long((lat + 90.0) * 120.0);
  return getValue(x, y);
}

const char *
LandCover::getDescUSGS (int value) const
{
  switch (value) {
  case 1:
    return "Urban and Built-Up Land";
  case 2:
    return "Dryland Cropland and Pasture";
  case 3:
    return "Irrigated Cropland and Pasture";
  case 4:
    return "Mixed Dryland/Irrigated Cropland and Pasture";
  case 5:
    return "Cropland/Grassland Mosaic";
  case 6:
    return "Cropland/Woodland Mosaic";
  case 7:
    return "Grassland";
  case 8:
    return "Shrubland";
  case 9:
    return "Mixed Shrubland/Grassland";
  case 10:
    return "Savanna";
  case 11:
    return "Deciduous Broadleaf Forest";
  case 12:
    return "Deciduous Needleleaf Forest";
  case 13:
    return "Evergreen Broadleaf Forest";
  case 14:
    return "Evergreen Needleleaf Forest";
  case 15:
    return "Mixed Forest";
  case 16:
    return "Water Bodies";
  case 17:
    return "Herbaceous Wetland";
  case 18:
    return "Wooded Wetland";
  case 19:
    return "Barren or Sparsely Vegetated";
  case 20:
    return "Herbaceous Tundra";
  case 21:
    return "Wooded Tundra";
  case 22:
    return "Mixed Tundra";
  case 23:
    return "Bare Ground Tundra";
  case 24:
    return "Snow or Ice";
  default:
    return "Unknown";
  }
}

// end of landcover.cxx
