// vpf-summary.cxx - print a summary of a VPF database's structure.
// This file is released into the Public Domain, and comes with NO WARRANTY!

////////////////////////////////////////////////////////////////////////
// Summarize the structure of a VPF database.
//
// Usage: 
//
// vpf-summary <database root> [library [coverage [feature [property]]]]
//
// This utility can drill down fairly far into a VPF database.  It can
// display anything from the schema for a whole database to the
// schema for a single feature property, depending on what appears
// on the command line.  Here's an example to dump the whole schema
// for all libraries on the vmap0 North America CD:
//
//   vpf-summary /cdrom/vmaplv0
//
// Here's a second example to show just the allowed values for the
// f_code feature property on the roadline feature in the transportation
// coverage of the North America library:
//
//   vpf-summary /cdrom/vmaplv0 noamer trans roadl f_code
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>

using std::string;

using std::cout;
using std::cerr;
using std::endl;

#include "vpf.hxx"

/**
 * Get a printable name for a value type.
 */
static const char *
get_value_type_name (VpfValue::Type type)
{
  switch (type) {
  case VpfValue::EMPTY:
    return "empty";
  case VpfValue::TEXT:
    return "text";
  case VpfValue::INT:
    return "integer";
  case VpfValue::REAL:
    return "real number";
  case VpfValue::POINTS:
    return "coordinate array";
  case VpfValue::DATE:
    return "date and time";
  case VpfValue::CROSSREF:
    return "cross-tile reference";
  default:
    throw VpfException("Unknown value type");
  }
}


/**
 * Get a printable name for a topology type.
 */
static const char *
get_topology_name (VpfFeature::TopologyType type)
{
  switch (type) {
  case VpfFeature::UNKNOWN:
    return "unknown";
  case VpfFeature::POINT:
    return "point";
  case VpfFeature::LINE:
    return "line";
  case VpfFeature::POLYGON:
    return "polygon";
  case VpfFeature::LABEL:
    return "text";
  default:
    return "unknown";
  }
}


/**
 * Print an indentation.
 */
static string
indent (int level)
{
  string result;
  for (int i = 0; i < level; i++)
    result += ' ';
  return result;
}


/**
 * Print a summary of a property declaration.
 */
static void
dump_property_decl (const VpfPropertyDecl &decl, int level)
{
  cout << indent(level) << "Name: " << decl.getName() << endl;
  cout << indent(level) << "Description: " << decl.getDescription() << endl;
  cout << indent(level) << "Type: "
       << get_value_type_name(decl.getValueType()) << endl;
  if (decl.getValueCount() > 0) {
    cout << indent(level) << "Number of allowed values: "
	 << decl.getValueCount() << endl;
    for (int i = 0; i < decl.getValueCount(); i++) {
      cout << indent(level) << "Allowed Value " << i << ':' << endl;
      cout << indent(level+2) << "Value: " << decl.getValue(i) << endl;
      cout << indent(level+2) << "Description: "
	   << decl.getValueDescription(i) << endl;
    }
  } else {
    cout << indent(level) << "No value restrictions." << endl;
  }
}


/**
 * Print a summary of a feature.
 */
static void
dump_feature (const VpfFeature &feature, int level)
{
  cout << indent(level) << "Name: " << feature.getName() << endl;
  cout << indent(level) << "Description: " << feature.getDescription() << endl;
  cout << indent(level) << "Topology type: "
       << get_topology_name(feature.getTopologyType()) << endl;
  cout << indent(level) << "Number of topologies: "
       << feature.getTopologyCount() << endl;
  if (feature.getTopologyType() == VpfFeature::POLYGON &&
      feature.getTopologyCount() > 0) {
    cout << indent(level) << "Bounding rectangle of first polygon: ";
    VpfRectangle bounds = feature.getPolygon(0).getBoundingRectangle();
    cout << bounds.minX << ','
	 << bounds.minY << ','
	 << bounds.maxX << ','
	 << bounds.maxY << endl;
  }
  cout << indent(level) << "Number of feature properties: "
       << feature.getPropertyDeclCount() << endl;
  for (int i = 0; i < feature.getPropertyDeclCount(); i++) {
    cout << indent(level) << "Feature property " << i << ':' << endl;
    dump_property_decl(feature.getPropertyDecl(i), level+2);
  }
}


/**
 * Print a summary of a coverage.
 */
static void
dump_coverage (const VpfCoverage &cov, int level)
{
  cout << indent(level) << "Coverage name: " << cov.getName() << endl;
  cout << indent(level) << "Coverage description: "
       << cov.getDescription() << endl;
  cout << indent(level) << "Coverage path: " << cov.getPath() << endl;
  cout << indent(level) << "Topological level: " << cov.getLevel() << endl;
  cout << indent(level) << "Number of features: "
       << cov.getFeatureCount() << endl;
  for (int i = 0; i < cov.getFeatureCount(); i++) {
    cout << indent(level) << "Feature " << i << ':' << endl;
    dump_feature(cov.getFeature(i), level+2);
  }
}


/**
 * Print a summary of a library.
 */
static void
dump_library (const VpfLibrary &lib, int level)
{
  cout << indent(level) << "Library name: " << lib.getName() << endl;
  cout << indent(level) << "Library description: "
       << lib.getDescription() << endl;
  cout << indent(level) << "Library path: " << lib.getPath() << endl;
  cout << indent(level) << "Minimum bounding rectangle: ";
  VpfRectangle bounds = lib.getBoundingRectangle();
  cout << bounds.minX << ','
       << bounds.minY << ','
       << bounds.maxX << ','
       << bounds.maxY << endl;
  cout << indent(level) << "Number of coverages: "
       << lib.getCoverageCount() << endl;
  for (int i = 0; i < lib.getCoverageCount(); i++) {
    cout << indent(level) << "Coverage " << i << ':' << endl;
    dump_coverage(lib.getCoverage(i), level+2);
  }
}


/**
 * Print a summary of a database.
 */
static void
dump_database (const VpfDataBase &db, int level)
{
  cout << indent(level) << "Database name: " << db.getName() << endl;
  cout << indent(level) << "Database description: "
       << db.getDescription() << endl;
  cout << indent(level) << "Database path: " << db.getPath() << endl;
  cout << indent(level) << "Number of libraries: "
       << db.getLibraryCount() << endl;
  for (int i = 0; i < db.getLibraryCount(); i++) {
    cout << "Library " << (i+1) << ':' << endl;
    dump_library(db.getLibrary(i), level+2);
  }
}

int
main (int ac, char ** av)
{
  try {
    switch (ac) {
    case 2:
      cout << "*** Database: " << av[1] << " ***" << endl << endl;
      dump_database(VpfDataBase(av[1]), 0);
      return 0;
    case 3:
      cout << "*** Library: " << av[2] << " ***" << endl << endl;
      dump_library(VpfDataBase(av[1]).getLibrary(av[2]), 0);
      return 0;
    case 4:
      cout << "*** Coverage: " << av[3] << " ***" << endl << endl;
      dump_coverage(VpfDataBase(av[1])
		    .getLibrary(av[2]).getCoverage(av[3]), 0);
      return 0;
    case 5:
      cout << "*** Feature: " << av[4] << " ***" << endl << endl;;
      dump_feature(VpfDataBase(av[1]).getLibrary(av[2]).getCoverage(av[3])
		   .getFeature(av[4]), 0);
      return 0;
    case 6:
      cout << "*** Feature Property: " << av[5] << " ***" << endl << endl;
      dump_property_decl(VpfDataBase(av[1]).getLibrary(av[2])
			 .getCoverage(av[3]).getFeature(av[4])
			 .getPropertyDecl(av[5]), 0);
      return 0;
    default:
      cerr << "Usage: " << av[0] << "<database> [library [coverage [feature]]]"
	   << endl;
      return 2;
    }
  } catch (VpfException &e) {
    cerr << e.getMessage() << endl;
    return 1;
  }
}

// end of vpf-summary.cxx
