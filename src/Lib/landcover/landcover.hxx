// landcover.hxx - query the USGS worldwide 30 arcsec land-cover image file
// Started 2000-10-25 by David Megginson, david@megginson.com

// This program is in the Public Domain and comes with NO WARRANTY.
// Use at your own risk.

#ifndef __LANDCOVER_HXX
#define __LANDCOVER_HXX 1

#include <simgear/compiler.h>

#include <string>
#include <fstream>

/**
 * Query class for the USGS worldwide 30 arcsec land-cover image.
 *
 * At the time of writing, the USGS worldwide 30 arc-second land-cover
 * image is available for free download at
 *
 *  http://edcwww.cr.usgs.gov/pub/data/glcc/globe/latlon/gusgs2_0ll.img.gz
 *
 * This image contains land-cover data for the entire world in 30
 * arc-second increments (about a square kilometer, depending on
 * latitude).  The class allows you to provide geographical co-ordinates
 * and discover the predominant land cover for that location.  This
 * class is intended for integration into the TerraGear terrain
 * construction program for the FlightGear flight simulator (see
 * www.terragear.org and www.flightgear.org).
 *
 * The image uncompresses to nearly a gigabyte, so this class does not
 * attempt to hold the entire image in memory; instead, it opens a
 * stream to the file and seeks to the appropriate position for each query.
 * The stream is closed automatically by the destructor.
 *
 * The image file is 43200 bytes wide and 21600 bytes high, and represents
 * 30 arc second increments from longitude -180.0 to 180.0 horizontally
 * and from latitude 90.0 to -90.0 vertically.
 *
 * To retrieve a value at any location, there are two getValue methods:
 *
 *  int getValue (long x, long y)
 *  int getValue (double lon, double lat)
 *
 * The first method returns the value at a location using native image
 * coordinates, where 0,0 is the top left corner and 43200,21600 is the
 * bottom right corner.  The second method returns the value at a
 * location using longitude and latitude, where -180.0,90.0 is the top
 * left corner and 180.0,-90.0 is the bottom right corner.
 * 
 * This class should work with any image file using the same coordinate
 * system and resolution.  For the USGS image, you can look up the
 * legend associated with any land-cover value using the getDescUSGS
 * method.
 *
 * @author David Megginson, david@megginson.com
 * @version 0.1
 */

class LandCover {

public:

  explicit LandCover( const std::string &filename );
  virtual ~LandCover ();

  virtual int getValue (long x, long y) const;
  virtual int getValue (double lon, double lat) const;
  virtual const char *getDescUSGS (int value) const;

private:
  mutable std::ifstream * _input;
  long WIDTH;
  long HEIGHT;
};

#endif // __LANDCOVER_HXX

// end of landcover.hxx
