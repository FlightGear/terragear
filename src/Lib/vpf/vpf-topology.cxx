// vpf-topology.cxx - program to dump a topology to output.
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include <simgear/compiler.h>

#include <iostream>
#include <string>

#include <stdlib.h>

#include "vpf.hxx"

using std::cout;
using std::cerr;
using std::endl;
using std::ostream;
using std::string;


ostream &
operator<< (ostream &output, const VpfRectangle &rect)
{
  output << rect.minX << ','
	 << rect.minY << ','
	 << rect.maxX << ','
	 << rect.maxY;
  return output;
}

static void
dump_point (const VpfPoint &p)
{
  cout << p.x << ' ' << p.y << endl;
}

static void
dump_point (const VpfPoint &p, const VpfRectangle &bounds)
{
  if (inside(p, bounds))
    dump_point(p);
  cout << endl;
}

static void
dump_line (const VpfLine &l)
{
  int nPoints = l.getPointCount();
  for (int i = 0; i < nPoints; i++)
    dump_point(l.getPoint(i));
}

static void
dump_line (const VpfLine &l, const VpfRectangle &bounds)
{
  if (overlap(l.getBoundingRectangle(), bounds))
    dump_line(l);
  cout << endl;
}

static void
dump_contour (const VpfContour &c)
{
  int nPoints = c.getPointCount();
  cerr << "dumping contour with " << nPoints << " points" << endl;
  for (int i = 0; i < nPoints; i++) {
    dump_point(c.getPoint(i));
  }
}

static void
dump_polygon (const VpfPolygon &poly)
{
  int nContours = poly.getContourCount();
  for (int i = 0; i < nContours; i++) {
    dump_contour(poly.getContour(i));
    cout << endl;
  }
  cout << endl;
}

static void
dump_polygon (const VpfPolygon &poly, const VpfRectangle &bounds)
{
  if (overlap(poly.getBoundingRectangle(), bounds))
    dump_polygon(poly);
}

static void
dump_label (const VpfLabel &label)
{
  const VpfPoint p = label.getPoint();
  cout << p.x << ' ' << p.y << ' ' << label.getText() << endl;
}

static void
dump_label (const VpfLabel &label, const VpfRectangle &bounds)
{
  if (inside(label.getPoint(), bounds))
    dump_label(label);
}

int
main (int ac, char ** av)
{
  VpfRectangle bounds;
  bounds.minX = -180.0;
  bounds.minY = -90.0;
  bounds.maxX = 180.0;
  bounds.maxY = 90.0;

  int args_start = 1;

  for (int i = 1; i < ac; i++) {
    string opt = av[i];
    if (opt.find("--minx=") == 0) {
      bounds.minX = atoi(opt.substr(7).c_str());
    } else if (opt.find("--miny=") == 0) {
      bounds.minY = atoi(opt.substr(7).c_str());
    } else if (opt.find("--maxx=") == 0) {
      bounds.maxX = atoi(opt.substr(7).c_str());
    } else if (opt.find("--maxy=") == 0) {
      bounds.maxY = atoi(opt.substr(7).c_str());
    } else if (opt.find("--") == 0) {
      cerr << "Unrecognized option: " << opt << endl;
      return 2;
    } else {
      break;
    }
    args_start++;
  }
  
  if (ac - args_start != 4) {
    cerr << "Usage: " << av[0]
	 << " <database dir> <library> <coverage> <feature>" << endl;
    return 2;
  }

  cerr << "Bounds: " << bounds << endl;

  try {

    VpfDataBase db(av[args_start]);
    VpfLibrary library = db.getLibrary(av[args_start+1]);
    VpfFeature feature = library.getCoverage(av[args_start+2])
      .getFeature(av[args_start+3]);


    const VpfRectangle rect = library.getBoundingRectangle();
    if (!overlap(rect, bounds)) {
      cerr << "Library coverage does not overlap with area" << endl;
      cerr << "Library: " << rect << endl;
      cerr << "Requested: " << bounds << endl;
      return 1;
    }

    int nTopologies = feature.getTopologyCount();
    int type = feature.getTopologyType();
    cerr << "Searching through " << nTopologies << " topologies" << endl;
    for (int i = 0; i < nTopologies; i++) {
      if ((i % 1000) == 0)
	cerr << i << "..." << endl;
      if (feature.isTiled()) {
	VpfRectangle rect = feature.getTile(i).getBoundingRectangle();
	if (!overlap(rect, bounds))
	  continue;
      }
	  
      switch (type) {
      case VpfFeature::POINT:
	dump_point(feature.getPoint(i), bounds);
	break;
      case VpfFeature::LINE:
	dump_line(feature.getLine(i), bounds);
	break;
      case VpfFeature::POLYGON:
	dump_polygon(feature.getPolygon(i), bounds);
	break;
      case VpfFeature::LABEL:
	dump_label(feature.getLabel(i), bounds);
	break;
      default:
	throw VpfException("Unsupported topology type");
      }
    }
  } catch (const VpfException &e) {
    cerr << "Died with exception: " << e.getMessage() << endl;
    return 1;
  }

  return 0;
}

// end of vpf-topology.cxx
