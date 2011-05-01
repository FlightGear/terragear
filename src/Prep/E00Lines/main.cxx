// main.cxx -- generate lines from E00 files.
//
// Written by David Megginson, started October 2000.  Based on
// code by Curtis Olson.
//
// Copyright (C) 2000  David Megginson  - david@megginson.com
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: main.cxx,v 1.18 2004-11-19 22:25:51 curt Exp $
 

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sgstream.hxx>
#include <simgear/misc/sg_path.hxx>

#include <iostream>
#include <string>
#include <vector>

#include <Geometry/rectangle.hxx>
#include <Geometry/util.hxx>
#include <Polygon/chop.hxx>
#include <Polygon/index.hxx>
#include <Polygon/polygon.hxx>
#include <e00/e00.hxx>

#include <stdlib.h>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;



////////////////////////////////////////////////////////////////////////
// Areas.
////////////////////////////////////////////////////////////////////////

/**
 * Make a bounding box for a single ARC.
 */
static const tg::Rectangle
makeBounds (const E00::ARC &arc)
{
  Point3D min, max;
  for (int i = 0; i < arc.numberOfCoordinates; i++) {
    double x = arc.coordinates[i].x;
    double y = arc.coordinates[i].y;
    if (i == 0) {
      min = max = Point3D(x, y, 0);
    } else {
      if (x < min.x()) min.setx(x);
      if (y < min.y()) min.sety(y);
      if (x > max.x()) max.setx(x);
      if (y > max.y()) max.sety(y);
    }
  }
  return tg::Rectangle(min, max);
}

/**
 * Make a bounding box for a polygon.
 */
static const tg::Rectangle
makeBounds (const E00::PAL &pal)
{
  return tg::Rectangle(Point3D(pal.min.x, pal.min.y, 0),
                       Point3D(pal.max.x, pal.max.y, 0));
}



////////////////////////////////////////////////////////////////////////
// Processing methods.
////////////////////////////////////////////////////////////////////////

/**
 * An attribute pattern.
 */
struct Attribute
{
  string file;
  string item;
  string value;
};


/**
 * Check whether an attribute pattern matches a shape.
 */
static bool
checkAttribute (const E00 &data, int index, const Attribute &att)
{
  const string * type = data.getIFOItemType(att.file, att.item);
  if (type == 0)
    return false;
  const string * value = data.getIFOItem(att.file, index, att.item);
  if (value == 0)
    return false;

  if (*type == "10-1") {	// date
    return (*value == att.value);
  }
  
  else if (*type == "20-1") {	// string
    return (*value == att.value);
  }

  else if (*type == "30-1" || *type == "50-1") { // integer
    int val1 = atoi(value->c_str());
    int val2 = atoi(att.value.c_str());
    return (val1 == val2);
  }

  else if (*type == "40-1" || *type == "60-1") { // float
    float val1 = atof(value->c_str());
    float val2 = atof(att.value.c_str());
    return (val1 == val2);
  }

  else {
    cerr << "Unknown IFO field type " << *type << endl;
    exit(1);
  }

  return (*value == att.value);
}


/**
 * Create polygons out of points.
 *
 * Note that simple geometry doesn't work here, because the scale is
 * not even -- the points on the x-axis (longitude) become closer and
 * closer as the y-axis (latitude) approaches the poles, meeting in
 * a single point at y=90 and y=-90.  As a result, this function
 * uses the WGS80 functions, rather than simple Pythagorean stuff.
 */
static void
processPoints (const E00 &data, const tg::Rectangle &bounds,
	       const string& poly_type, const string &workDir, int width)
{
  // double x, y, az;

  int nPoints = data.nPoints();
  cout << "Processing " << nPoints << " points" << endl;
  for (int i = 1; i <= nPoints; i++) {
    TGPolygon shape;
    const E00::LAB &lab = data.getLAB(i);
    Point3D p(lab.coord.x, lab.coord.y, 0);

    if (!bounds.isInside(p)) {
      cout << "Skipping point " << i << " (out of bounds)" << endl;
      continue;
    }

    tg::makePolygon(p, width, shape);
    tgChopNormalPolygon(workDir, poly_type, shape, false);
  }
}


/**
 * Create polygons out of all loose line segments.
 *
 * Note that simple geometry doesn't work here, because the scale is
 * not even -- the points on the x-axis (longitude) become closer and
 * closer as the y-axis (latitude) approaches the poles, meeting in
 * a single point at y=90 and y=-90.  As a result, this function
 * uses the WGS80 functions, rather than simple Pythagorean stuff.
 */
static void
processLines (const E00 &data, const tg::Rectangle &bounds,
	      const string& poly_type, const string &workDir, int width,
	      const vector<Attribute> &aat_list)
{
  int nLines = data.nLines();
  cout << "Processing " << nLines << " lines." << endl;
  for (int i = 1; i <= nLines; i++) {
    TGPolygon shape;
    const E00::ARC &arc = data.getARC(i);
    tg::Rectangle arcBounds = makeBounds(arc);
    if (!bounds.isOverlapping(arcBounds)) {
      cout << "Arc " << i << " outside of area; skipping" << endl;
      continue;
    }

				// If any arc attributes were specified,
				// make sure at least one of them is
				// present.
    if (aat_list.size() > 0) {
      bool status = false;
      for (unsigned int j = 0; j < aat_list.size(); j++) {
	if (checkAttribute(data, i, aat_list[j])) {
	  status = true;
	  break;
	}
      }
      if (!status) {
	cout << "Skipping line " << i << " (failed attribute tests)" << endl;
	continue;
      }
    }

    tg::Line line;
    int j;
    for (j = 0; j < arc.numberOfCoordinates; j++) {
      line.addPoint(Point3D(arc.coordinates[j].x,
			    arc.coordinates[j].y,
			    0));
    }

    tg::makePolygon(line, width, shape);

    				// Split into tiles
    cout << "Splitting polygon..." << endl;
    cout << "  Total size: " << shape.total_size() << endl;
    cout << "  Minimum angle: "
	 << (shape.minangle_contour(0) * SGD_RADIANS_TO_DEGREES) << endl;
    
    tgChopNormalPolygon(workDir, poly_type, shape, false);
  }
  cout << "Done lines" << endl;
}


/**
 * Import all polygons.
 */
static void
processPolygons (const E00 &data, const tg::Rectangle &bounds,
		 const string& poly_type, const string &workDir,
		 const vector<Attribute> pat_list)
{
  int nPolygons = data.nPolygons();
  cout << "Processing " << nPolygons << " polygons" << endl;

  for (int i = 2; i <= nPolygons; i++) {
    TGPolygon shape;
				// Test whether the polygon matches
				// at least one of the attributes
				// provided.
    if (pat_list.size() > 0) {
      bool status = false;
      for (unsigned int j = 0; j < pat_list.size(); j++) {
	if (checkAttribute(data, i, pat_list[j])) {
	  status = true;
	  break;
	}
      }
      if (!status) {
	cout << "Skipping polygon " << i << " (failed attribute tests)"
	     << endl;
	continue;
      }
    }

    int contour = 0;
    const E00::PAL &pal = data.getPAL(i);
    tg::Rectangle palBounds = makeBounds(pal);
    if (!bounds.isOverlapping(palBounds)) {
      cout << "Polygon " << i << " outside of area, skipping" << endl;
      continue;
    }
    shape.erase();
    for (int j = 0; j < pal.numArcs; j++) {
      int arcNum = pal.arcs[j].arcNum;
				// Starting a hole
      if (arcNum == 0) {
	contour++;
	point_list contour;
	contour.clear();
	shape.add_contour(contour, 1);
      } else if (arcNum < 0) {
	const E00::ARC &arc = data.getARC(0-arcNum);
	for (int k = arc.numberOfCoordinates - 1; k >= 0; k--)
	  shape.add_node(contour, Point3D(arc.coordinates[k].x,
					  arc.coordinates[k].y,
					  0.0));
      } else {
	const E00::ARC &arc = data.getARC(arcNum);
	for (int k = 0; k < arc.numberOfCoordinates; k++)
	  shape.add_node(contour, Point3D(arc.coordinates[k].x,
					  arc.coordinates[k].y,
					  0.0));
      }
    }
    tgChopNormalPolygon(workDir, poly_type, shape, false);
  }
}




////////////////////////////////////////////////////////////////////////
// Main program.
////////////////////////////////////////////////////////////////////////


/**
 * Print the command-line usage and exit.
 */
static void
usage (const char * prog)
{
  cerr << "Usage: " << prog << " [opts] e00file ..." << endl;
  cerr << "Options:" << endl;
  cerr << "--points=yes|no (default: no)" << endl;
  cerr << "--lines=yes|no (default: yes)" << endl;
  cerr << "--polygons=yes|no (default: yes)" << endl;
  cerr << "--min-lon=<longitude> (default: -180.0)" << endl;
  cerr << "--min-lat=<latitude> (default: -90.0)" << endl;
  cerr << "--max-lon=<longitude> (default: 180.0)" << endl;
  cerr << "--max-lat=<latitude> (default: 90.0)" << endl;
  cerr << "--area=<area_type> (default: Default)" << endl;
  cerr << "--point-width=<meters> (default: 500)" << endl;
  cerr << "--line-width=<meters> (default: 50)" << endl;
  cerr << "--work-dir=<dir> (default: .)" << endl;
  cerr << "--aat=<infofile>:<item>:<value> (may be repeated)" << endl;
  cerr << "--pat=<infofile>:<item>:<value> (may be repeated)" << endl;
  exit(2);
}


/**
 * Parse an attribute value specification from the command line.
 */
static void
parseAttribute (const char * prog, string arg, Attribute &att)
{
  int pos1 = arg.find(':');
  int pos2 = arg.rfind(':');
  if (pos1 == -1 || pos2 == -1 || pos2 <= pos1) {
    cerr << "Bad attribute specification: " << arg << endl;
    usage(prog);
  }

  att.file = arg.substr(0, pos1);
  att.item = arg.substr(pos1 + 1, pos2 - (pos1 + 1));
  att.value = arg.substr(pos2+1);
}


/**
 * Main entry point.
 */
int
main (int argc, const char **argv)
{
  vector<Attribute> aat_list;
  vector<Attribute> pat_list;

				// Enable logging.
  sglog().setLogLevels( SG_ALL, SG_DEBUG );


				// Default values
  tg::Rectangle bounds(Point3D(-180.0, -90.0, 0),
		   Point3D(180.0, 90.0, 0));
  string poly_type = "Default";
  int pointWidth = 500;
  int lineWidth = 50;
  string workDir = ".";
  bool usePoints = false;
  bool useLines = true;
  bool usePolygons = true;

				// Command-line options
  int argPos = 1;
  while (argPos < argc) {
    string arg = argv[argPos];

    cout << "Trying argument " << arg << endl;

    if (arg.find("--points=") == 0) {
      if (arg.substr(9) == "yes")
	usePoints = true;
      else if (arg.substr(9) == "no")
	usePoints = false;
      else {
	cerr << "--points option needs 'yes' or 'no'" << endl;
	usage(argv[0]);
      }
      argPos++;
    }

    else if (arg.find("--lines=") == 0) {
      if (arg.substr(8) == "yes")
	useLines = true;
      else if (arg.substr(8) == "no")
	useLines = false;
      else {
	cerr << "--lines option needs 'yes' or 'no'" << endl;
	usage(argv[0]);
      }
      argPos++;
    }

    else if (arg.find("--polygons=") == 0) {
      if (arg.substr(11) == "yes")
	usePolygons = true;
      else if (arg.substr(11) == "no")
	usePolygons = false;
      else {
	cerr << "--polygons option needs 'yes' or 'no'" << endl;
	usage(argv[0]);
      }
      argPos++;
    }

    else if (arg.find("--min-lon=") == 0) {
      bounds.getMin().setx(atof(arg.substr(10).c_str()));
      argPos++;
    }

    else if (arg.find("--min-lat=") == 0) {
      bounds.getMin().sety(atof(arg.substr(10).c_str()));
      argPos++;
    }

    else if (arg.find("--max-lon=") == 0) {
      bounds.getMax().setx(atof(arg.substr(10).c_str()));
      argPos++;
    } 

    else if (arg.find("--max-lat=") == 0) {
      bounds.getMax().sety(atof(arg.substr(10).c_str()));
      argPos++;
    }

    else if (arg.find("--area=") == 0) {
      poly_type = arg.substr(7);
      argPos++;
    }

    else if (arg.find("--point-width=") == 0) {
      pointWidth = atoi(arg.substr(14).c_str());
      argPos++;
    }

    else if (arg.find("--line-width=") == 0) {
      lineWidth = atoi(arg.substr(13).c_str());
      argPos++;
    }

    else if (arg.find("--work-dir=") == 0) {
      workDir = arg.substr(11);
      argPos++;
    }

    else if (arg.find("--aat=") == 0) {
      Attribute att;
      parseAttribute(argv[0], arg.substr(6), att);
      aat_list.push_back(att);
      cout << "Added polygon constraint for " << att.file << ": "
	   << att.item << '=' << att.value << endl;
      argPos++;
    }

    else if (arg.find("--pat=") == 0) {
      Attribute att;
      parseAttribute(argv[0], arg.substr(6), att);
      pat_list.push_back(att);
      cout << "Added polygon constraint for " << att.file << ": "
	   << att.item << '=' << att.value << endl;
      argPos++;
    }

    else if (arg == "--") {
      argPos++;
      break;
    }

    else if (arg.find("-") == 0) {
      cerr << "Unrecognized option: " << arg << endl;
      usage(argv[0]);
    }

    else {
      break;
    }
  }

				// Check for files to process
  if (argPos >= argc) {
    cerr << "No e00 files specified!!" << endl;
    usage(argv[0]);
  }

  cout << "Bounds are " << bounds.getMin().x() << ','
       << bounds.getMin().y() << " and "
       << bounds.getMax().x() << ',' << bounds.getMax().y() << endl;
  cout << "Area type is " << poly_type << endl;;
  cout << "Working directory is " << workDir << endl;
  if (usePoints)
    cout << "Using points with width " << pointWidth << " meters" << endl;
  else
    cout << "Ignoring point coverage" << endl;
  if (useLines)
    cout << "Using lines with width " << lineWidth << " meters" << endl;
  else
    cout << "Ignoring line coverage" << endl;
  if (usePolygons)
    cout << "Using polygons" << endl;
  else
    cout << "Ignoring polygon coverage" << endl;
  if (useLines && aat_list.size() > 0) {
    cout << "Lines must match at least one of the following:" << endl;
    for (unsigned int i = 0; i < aat_list.size(); i++) {
      cout << aat_list[i].file << ' ' << aat_list[i].item << ' '
	   << aat_list[i].value << endl;
    }
  }
  if (usePolygons && pat_list.size() > 0) {
    cout << "Polygons must match at least one of the following:" << endl;
    for (unsigned int i = 0; i < pat_list.size(); i++) {
      cout << pat_list[i].file << ' ' << pat_list[i].item << ' '
	   << pat_list[i].value << endl;
    }
  }

				// Make sure the destination
				// directory exists.
  SGPath sgp( workDir );
  sgp.append( "dummy" );
  sgp.create_dir( 0755 );

				// Process all of the e00 files
				// on the command line.
  while (argPos < argc) {

    cout << "Processing " << argv[argPos] << endl;

				// Grab command-line arguments.
    sg_gzifstream input(argv[argPos]);
    if ( !input.good() ) {
      SG_LOG( SG_GENERAL, SG_ALERT, "Cannot open file: " << argv[argPos]);
      exit(-1);
    }

				// Read the E00 file.
    
    E00 data;
    try {
      data.readE00(input);
    } catch (E00Exception &e) {
      SG_LOG(SG_GENERAL, SG_ALERT, "Reading " << argv[argPos]
	     << " failed with exception " << e.getMessage());
      exit(1);
    }

				// Initialize the persistant polygon counter.
    string counter_file = workDir + "/../poly_counter";
    poly_index_init( counter_file );
    
    if (usePoints)
      processPoints(data, bounds, poly_type, workDir, pointWidth);

    if (useLines)
      processLines(data, bounds, poly_type, workDir, lineWidth, aat_list);

    if (usePolygons)
      processPolygons(data, bounds, poly_type, workDir, pat_list);
    
    cout << "Done processing " << argv[argPos] << endl;
    argPos++;
  }

  cout << "Done all processing." << endl;
  return 0;
}

// end of main.cxx
