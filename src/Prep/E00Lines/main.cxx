// main.cxx -- generate lines from E00 files.
//
// Written by David Megginson, started October 2000.  Based on
// code by Curtis Olson.
//
// Copyright (C) 2000  David Megginson  - david@megginson.com
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
 

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/debug/logstream.hxx>

#include <string>
#include <fstream>

using std::string;
using std::ifstream;

#include <Polygon/index.hxx>
#include <Polygon/names.hxx>
#include <Polygon/polygon.hxx>
#include <Polygon/split.hxx>
#include <e00/e00.hxx>

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

static inline double ANGLE (double a)
{
  while (a < 0.0)
    a += 360.0;
  while (a >= 360.0)
    a -= 360.0;
  return a;
}


int
main (int argc, const char **argv)
{

				// Enable logging.
  fglog().setLogLevels( FG_ALL, FG_DEBUG );
  
				// Check usage.
  if ( argc != 5 ) {
    FG_LOG( FG_GENERAL, FG_ALERT, "Usage: " << argv[0] 
	    << " <e00_line_file> <width_meters> <area_type> <work_dir>" );
    exit(-1);
  }

				// Grab command-line arguments.
  ifstream input(argv[1]);
  if ( !input.good() ) {
    FG_LOG( FG_GENERAL, FG_ALERT, "Cannot open file: " << argv[1]);
    exit(-1);
  }

  int width = atoi(argv[2]);
  if (width <= 0) {
    FG_LOG(FG_GENERAL, FG_ALERT, "Bad width specified " << argv[2]);
    exit(-1);
  }

  AreaType area_type = get_area_type(argv[3]);

  string work_dir = argv[4];
#ifdef _MSC_VER
  fg_mkdir(work_dir.c_str());
#else
  string command = "mkdir -p " + work_dir;
  system(command.c_str());
#endif

				// Read the E00 file.
  E00 data;
  try {
    data.readE00(input);
  } catch (E00Exception &e) {
    FG_LOG(FG_GENERAL, FG_ALERT, "Reading " << argv[1]
	   << " failed with exception " << e.message);
    exit(1);
  }

				// Initialize the persistant polygon counter.
  string counter_file = work_dir + "/../poly_counter";
  poly_index_init( counter_file );

  FGPolygon shape;

				// Iterate through the lines.
  int nLines = data.nLines();
  for (int i = 0; i < nLines; i++) {
    const e00ARC * line = data.getLine(i);
    cout << "Line has " << line->numberOfCoordinates << " coordinates" << endl;
    for (int j = 0; j < line->numberOfCoordinates - 1; j++) {
      double lon1 = line->coordinates[j].x;
      double lat1 = line->coordinates[j].y;
      double lon2 = line->coordinates[j+1].x;
      double lat2 = line->coordinates[j+1].y;
      double angle1, angle2, dist;
      geo_inverse_wgs_84(0, lat1, lon1, lat2, lon2, &angle1, &angle2, &dist);
      cout << "angle1 = " << angle1 << endl;
      cout << "angle2 = " << angle2 << endl;
      cout << "dist = " << dist << endl;
      shape.erase();

      double x, y, az;

				// Corner 1
      geo_direct_wgs_84(0, lat1, lon1, ANGLE(angle1+90), width/2, &y, &x, &az);
      cout << x << '\t' << y << endl;
      shape.add_node(0, Point3D(x, y, 0));

				// Corner 2
      geo_direct_wgs_84(0, lat1, lon1, ANGLE(angle1-90), width/2, &y, &x, &az);
      cout << x << '\t' << y << endl;
      shape.add_node(0, Point3D(x, y, 0));

				// Corner 3
      geo_direct_wgs_84(0, lat2, lon2, ANGLE(angle2+90), width/2, &y, &x, &az);
      cout << x << '\t' << y << endl;
      shape.add_node(0, Point3D(x, y, 0));

				// Corner 4
      geo_direct_wgs_84(0, lat2, lon2, ANGLE(angle2-90), width/2, &y, &x, &az);
      cout << x << '\t' << y << endl;
      shape.add_node(0, Point3D(x, y, 0));

				// Corner 1, again
      geo_direct_wgs_84(0, lat1, lon1, ANGLE(angle1+90), width/2, &y, &x, &az);
      cout << x << '\t' << y << endl;
      shape.add_node(0, Point3D(x, y, 0));

				// Split into tiles
      split_polygon(work_dir, area_type, shape);
    }
  }

  return 0;
}


