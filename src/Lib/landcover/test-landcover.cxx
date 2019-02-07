// test-landcover.cxx - simple test program for the LandCover class.

// This program is in the Public Domain and comes with NO WARRANTY.
// Use at your own risk.

#include <simgear/compiler.h>

#include <stdlib.h>

#include <iostream>
#include <string>

#include "landcover.hxx"

using std::cerr;
using std::cout;
using std::endl;
using std::string;

int
main (int ac, const char * av[])
{
  if (ac != 4) {
    cerr << "Usage: " << av[0] << " <filename> <lon> <lat>" << endl;
    return 1;
  }

  const char * filename = av[1];
  double lon = atof(av[2]);
  double lat = atof(av[3]);

  cout << "File: " << filename << endl;
  cout << "Longitude: " << lon << endl;
  cout << "Latitude: " << lat << endl;

  try {

    LandCover lu(filename);

    int value = lu.getValue(lon, lat);
    cout << "Value is " << value 
	 << " \"" << lu.getDescUSGS(value) << '"' << endl;
  } catch (const string& e) {
    cerr << "Died with exception: " << e << endl;
    return 1;
  }

  return 0;
}

// end of test-landcover.cxx
