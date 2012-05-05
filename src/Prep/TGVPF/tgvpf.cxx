// tgvpf.cxx -- generate polygons from VPF topologies.
//
// Written by David Megginson, started September 2001.   Based on
// code by Curtis Olson.
//
// Copyright (C) 2001  David Megginson  - david@megginson.com
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
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: tgvpf.cxx,v 1.20 2005-09-28 16:44:05 curt Exp $
 

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sgstream.hxx>
#include <simgear/timing/timestamp.hxx>
#include <simgear/misc/sg_path.hxx>

#include <iostream>
#include <string>
#include <vector>

#include <Geometry/line.hxx>
#include <Geometry/util.hxx>
#include <Polygon/chop.hxx>
#include <Polygon/index.hxx>
#include <Polygon/polygon.hxx>
#include <vpf/vpf.hxx>

#include <stdlib.h>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;



////////////////////////////////////////////////////////////////////////
// Program-wide variables.
////////////////////////////////////////////////////////////////////////

static const char * progname;



////////////////////////////////////////////////////////////////////////
// VPF conversion code.
////////////////////////////////////////////////////////////////////////

/**
 * Convert a VPF point to a regular TerraGear point.
 */
static inline const Point3D
vpf2tg (const VpfPoint &p)
{
  return Point3D(p.x, p.y, p.z);
}


/**
 * Convert a VPF line to a regular TerraGear line.
 */
static const tg::Line
vpf2tg (const VpfLine &l)
{
  tg::Line result;
  int nPoints = l.getPointCount();
  for (int i = 0; i < nPoints; i++)
    result.addPoint(vpf2tg(l.getPoint(i)));
  return result;
}


/**
 * Convert a VPF rectangle to a TerraGear rectangle.
 */
static inline const tg::Rectangle
vpf2tg (const VpfRectangle &rect)
{
  return tg::Rectangle(Point3D(rect.minX, rect.minY, 0),
		   Point3D(rect.maxX, rect.maxY, 0));
}


/**
 * Convert a VPF polygon to a TerraGear polygon.
 */
static const TGPolygon
vpf2tg (const VpfPolygon &polygon)
{
  TGPolygon shape;

  shape.erase();
  int nContours = polygon.getContourCount();
  int contour_num = 0;
  for (int i = 0; i < nContours; i++) {
    const VpfContour contour = polygon.getContour(i);
    int nPoints = contour.getPointCount();
    for (int j = 0; j < nPoints; j++) {
      const VpfPoint p = contour.getPoint(j);
      shape.add_node(contour_num, Point3D(p.x, p.y, p.z));
    }
    shape.set_hole_flag(contour_num, (i > 0));
    contour_num++;
  }
  return shape;
}



////////////////////////////////////////////////////////////////////////
// Processing methods.
////////////////////////////////////////////////////////////////////////

/**
 * An attribute pattern.
 */
struct Attribute
{
  Attribute ()
    : state(true), name(""), value("")
  {}
  bool state;
  string name;
  string value;
};


/**
 * Check whether an attribute pattern matches a shape.
 */
static bool
checkAttribute (const VpfFeature &feature, int index, const Attribute &att)
{
  bool result = false;
  const VpfValue &value = feature.getPropertyValue(att.name, index);
  switch (value.getType()) {
  case VpfValue::TEXT:
    result = (att.value == value.getText());
    break;
  case VpfValue::INT:
    result = (atoi(att.value.c_str()) == value.getInt());
    break;
  case VpfValue::REAL:
    result = (strtod(att.value.c_str(), 0) == value.getReal());
    break;
  default:
    result = false;
    break;
  }
  return (att.state ? result : !result);
}


/**
 * Get the area of a polygon's bounding rectangle.
 *
 * Since we have to convert from geodetic, it would be pretty
 * expensive to calculate the actual area, so we just do the bounding
 * rectangle.
 *
 * @param polygon
 * @return The area of the bounding rectangle in m^2.
 */
static double
getArea (const TGPolygon &polygon)
{
    tg::Rectangle bounds = tg::makeBounds(polygon);
    Point3D min =
        sgGeodToCart(Point3D(bounds.getMin().x() * SGD_DEGREES_TO_RADIANS,
                             bounds.getMin().y() * SGD_DEGREES_TO_RADIANS,
                             0));
    Point3D max =
        sgGeodToCart(Point3D(bounds.getMax().x() * SGD_DEGREES_TO_RADIANS,
                             bounds.getMax().y() * SGD_DEGREES_TO_RADIANS,
                             0));
    double width = fabs(max.x() - min.x());
    double height = fabs(max.y() - min.y());
    return (width * height);
}



////////////////////////////////////////////////////////////////////////
// Main program.
////////////////////////////////////////////////////////////////////////


/**
 * Print the command-line usage and exit.
 */
static void
usage ()
{
  cerr << "Usage: "
       << progname
       << " [opts] <db> <library> <coverage> <feature>"
       << endl;
  cerr << "Options:" << endl;
  cerr << "--tile=<tile>         (default: none, ex: e002n49)" << endl;
  cerr << "--chunk=<chunk>       (default: none, ex: w090n10)" << endl;
  cerr << "--min-lon=<longitude> (default: -180.0)" << endl;
  cerr << "--min-lat=<latitude>  (default: -90.0)" << endl;
  cerr << "--max-lon=<longitude> (default: 180.0)" << endl;
  cerr << "--max-lat=<latitude>  (default: 90.0)" << endl;
  cerr << "--min-area=<area>     (default: none)" << endl;
  cerr << "--max-area=<area>     (default: none)" << endl;
  cerr << "--material=<material_type> (default: Default)" << endl;
  cerr << "--width=<meters>      (default: 50 line, 500 point)" << endl;
  cerr << "--max-segment=<meters>(default: none)" << endl;
  cerr << "--work-dir=<dir>      (default: .)" << endl;
  cerr << "--att=<item>:<value>  (may be repeated)" << endl;
  cerr << "--att=!<item>:<value> (may be repeated)" << endl;
  cerr << "--start-index=<num>      (default: 0)" << endl;
  exit(2);
}


/**
 * Parse an attribute value specification from the command line.
 */
static const Attribute
parseAttribute (string arg)
{
  Attribute att;

  if (arg[0] == '!') {
    att.state = false;
    arg = arg.substr(1);
  } else {
    att.state = true;
  }

  int pos = arg.find(':');
  if (pos == -1) {
    cerr << "Bad attribute specification: " << arg << endl;
    usage();
  }

  att.name = arg.substr(0, pos);
  att.value = arg.substr(pos + 1);

  return att;
}


/**
 * Main entry point.
 */
int
main (int argc, const char **argv)
{
				// Store the program name for future
				// reference.
  progname = argv[0];

  vector<Attribute> attributes;

				// Enable logging.
  sglog().setLogLevels( SG_ALL, SG_WARN );


				// Default values
  tg::Rectangle bounds(Point3D(-180, -90, 0), Point3D(180, 90, 0));
  bool invert = false;
  string material_type = "Default";
  int width = -1;		// use default
  double max_segment = 0.0;     // zero = no segement splitting
  string work_dir = ".";
  double min_area = -1;
  double max_area = -1;
  int start_index = 0;


  //
  // Process command-line options.
  //
  int argPos = 1;
  while (argPos < argc) {
    string arg = argv[argPos];
    
    if (arg.find("--tile=") == 0) {
        bounds = tg::parseTile(arg.substr(7));
        argPos++;
    }

    else if (arg.find("--chunk=") == 0) {
        bounds = tg::parseChunk(arg.substr(8), 10.0);
        argPos++;
    }

    else if (arg.find("--min-lon=") == 0) {
        bounds.getMin().setx(strtod(arg.substr(10).c_str(), 0));
        argPos++;
    }

    else if (arg.find("--min-lat=") == 0) {
        bounds.getMin().sety(strtod(arg.substr(10).c_str(), 0));
        argPos++;
    }

    else if (arg.find("--max-lon=") == 0) {
        bounds.getMax().setx(strtod(arg.substr(10).c_str(), 0));
        argPos++;
    } 

    else if (arg.find("--max-lat=") == 0) {
        bounds.getMax().sety(strtod(arg.substr(10).c_str(), 0));
        argPos++;
    }

    else if (arg.find("--min-area=") == 0) {
        min_area = strtod(arg.substr(11).c_str(), 0);
        argPos++;
    }

    else if (arg.find("--max-area=") == 0) {
        max_area = strtod(arg.substr(11).c_str(), 0);
        argPos++;
    }

    else if (arg.find("--material=") == 0) {
        material_type = arg.substr(11);
        argPos++;
    }

    else if (arg.find("--width=") == 0) {
        width = atoi(arg.substr(8).c_str());
        argPos++;
    }

    else if (arg.find("--max-segment=") == 0) {
        max_segment = atoi(arg.substr(14).c_str());
        cout << "Maximum segment length set to " << max_segment << " meters."
             << endl;
        argPos++;
    }

    else if (arg.find("--work-dir=") == 0) {
        work_dir = arg.substr(11);
        argPos++;
    }

    else if (arg.find("--invert") == 0) {
        invert = true;
        argPos++;
    }

    else if (arg.find("--att=") == 0) {
        attributes.push_back(parseAttribute(arg.substr(6)));
        argPos++;
    }

    else if (arg.find("--start-index=") == 0) {
        start_index = atoi(arg.substr(14).c_str());
        argPos++;
    }

    else if (arg == "--") {
        argPos++;
        break;
    }

    else if (arg.find("-") == 0) {
        cerr << "Unrecognized option: " << arg << endl;
        usage();
    }

    else {
        break;
    }
  }

  //
  // Sanity check on bounds.
  //
  bounds.sanify();


  //
  // Process command-line arguments.
  //

  if (argPos != (argc - 4))
    usage();

  const char * database_name = argv[argPos++];
  const char * library_name = argv[argPos++];
  const char * coverage_name = argv[argPos++];
  const char * feature_name = argv[argPos++];

				// Make sure the destination
				// directory exists.
  SGPath sgp( work_dir );
  sgp.append( "dummy" );
  sgp.create_dir( 0755 );

  //
  // Make the TerraGear polygon for the bounds.
  //
  TGPolygon bounds_poly = bounds.toPoly();

  //
  // Show settings.
  //
  cout << "Database path: " << database_name << endl;
  cout << "Library name: " << library_name << endl;
  cout << "Coverage name: " << coverage_name << endl;
  cout << "Feature name: " << feature_name << endl;
  cout << "Working directory: " << work_dir << endl;
  if (min_area > -1)
      cout << "Minimum area: " << min_area << endl;
  if (max_area > -1)
      cout << "Maximum area: " << max_area << endl;
  cout << "Material type: " << material_type << endl;
  if (width > -1)
      cout << "Point and line width: " << width << endl;
  for (unsigned int x = 0; x < attributes.size(); x++) {
    cout << "Attribute " << attributes[x].name
	 << (attributes[x].state ? " = " : " != ")
	 << attributes[x].value << endl;
  }
  if ( start_index > 0 ) {
    cout << "Starting at index = " << start_index << endl;
  }


  //
  // Main processing loop.
  //

				// Initialize the persistant polygon counter.
  string counter_file = work_dir + "/../poly_counter";
  poly_index_init( counter_file );
    
  try {
    VpfDataBase db(database_name);
    VpfLibrary lib = db.getLibrary(library_name);
    VpfFeature feature = lib.getCoverage(coverage_name)
      .getFeature(feature_name);

    const VpfRectangle rect = lib.getBoundingRectangle();
    if (!bounds.isOverlapping(vpf2tg(rect))) {
      cerr << "Library coverage does not overlap with area" << endl;
      return 1;
    }

    int nTopologies = feature.getTopologyCount();
    int type = feature.getTopologyType();
    cerr << "Searching through " << nTopologies << " topologies" << endl;
    int nAttributes = attributes.size();
    SGTimeStamp start, now;
    start.stamp();
    long start_sec = start.get_seconds();

    TGPolygon mask;
    for (int i = start_index; i < nTopologies; i++) {
      if ((i % 10) == 0) {
	now.stamp();
        long now_sec = now.get_seconds();
	double elapsed_min = (now_sec - start_sec) / 60.0;
        double percent = (double)i / (double)nTopologies;
	cerr << i << "# ";
	cerr << percent*100.0 << "% done ...";
        cerr << " total time = " << elapsed_min / percent;
	cerr << " (min)  finished in " << elapsed_min / percent - elapsed_min
             << " minutes" << endl;
      }
      if (feature.isTiled()) {
	VpfRectangle rect = feature.getTile(i).getBoundingRectangle();
	if (!bounds.isOverlapping(vpf2tg(rect)))
	  continue;
      }

      bool skip = false;
      for (int j = 0; j < nAttributes; j++) {
	if (!checkAttribute(feature, i, attributes[j])) {
	  skip = true;
	  break;
	}
      }
      if (skip)
	continue;
	  
      TGPolygon shape;
      switch (type) {
	// FIXME: check for attributes as well
      case VpfFeature::POINT: {
	const Point3D p = vpf2tg(feature.getPoint(i));
	if (!bounds.isInside(p))
	  continue;
	tg::makePolygon(p, (width == -1 ? 500 : width), shape);
	break;
      }
      case VpfFeature::LINE: {
	const tg::Line line = vpf2tg(feature.getLine(i));
	if (!bounds.isOverlapping(line.getBounds()))
	  continue;
	tg::makePolygon(line, (width == -1 ? 50 : width), shape);
	break;
      }
      case VpfFeature::POLYGON: {
	const VpfPolygon polygon = feature.getPolygon(i);
	if (!bounds.isOverlapping(vpf2tg(polygon.getBoundingRectangle())))
	  continue;
	shape = vpf2tg(polygon);
                                // Filter by area if requested
        if (max_area > -1 || min_area > -1) {
            double area = getArea(shape);
            if (max_area > -1 && area > max_area)
                continue;
            if (min_area > -1 && area < min_area)
                continue;
        }
	break;
      }
      case VpfFeature::LABEL: {
	const Point3D p = vpf2tg(feature.getLabel(i).getPoint());
	if (!bounds.isInside(p))
	  continue;
	tg::makePolygon(p, (width == -1 ? 500 : width), shape);
	break;
      }
      default:
	throw VpfException("Unsupported topology type");
      }

      if (invert) {
	mask = tgPolygonUnion(mask, shape);
      } else {
	shape = tgPolygonInt(shape, bounds_poly);
	if (shape.total_size() >= 3) {
          // cout << "Polygon with " << shape.total_size() << " points in "
          //      << shape.contours() << " contour(s)" << endl;
          if ( max_segment > 1.0 ) {
              shape = tgPolygonSplitLongEdges( shape, max_segment );
          }
	  tgChopNormalPolygon(work_dir, material_type, shape, false);
	}
      }
    }

				// If we are inverting, we have to
				// wait until the end (and hope for
				// not too large a polygon)
    if (invert) {
      mask = tgPolygonDiff(bounds_poly, mask);
      if (mask.total_size() >= 3) {
	cout << "Inverse polygon with " << mask.total_size() << " points in "
	     << mask.contours() << " contour(s)" << endl;
        if ( max_segment > 1.0 ) {
            mask = tgPolygonSplitLongEdges( mask, max_segment );
        }
	tgChopNormalPolygon(work_dir, material_type, mask, false);
      } else {
	cout << "Inverse polygon is empty" << endl;
      }
    }
    
  } catch (VpfException &e) {
    cerr << "Processing failed with VPF exception: " << e.getMessage() << endl;
    return 1;
  }

  return 0;
}

// end of tgvpf.cxx
