// teste00.cxx - test the E00 parsing routines and dump some results.

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/misc/sgstream.hxx>
#include "e00.hxx"

SG_USING_STD(cerr);
SG_USING_STD(cout);
SG_USING_STD(endl);

int main (int ac, const char ** av)
{
  int i, j, k;

  for (i = 1; i < ac; i++) {
    cerr << "Reading " << av[i] << endl;
    sg_gzifstream input(av[i]);
    E00 data;
    try {
      data.readE00(input);
    } catch (E00Exception &e) {
      cerr << "Reading " << av[i] << " failed with exception "
	   << e.getMessage() << endl;
      exit(1);
    }
    cout << "Read " << av[i] << " successfully" << endl;
    cout << "Read " << data.nPoints() << " point(s)" << endl;
    cout << "Read " << data.nLines() << " line segment(s)" << endl;
    cout << "Read " << data.nPolygons() << " polygon(s)" << endl;
    cout << " (including enclosing polygon)" << endl;

//     for (j = 1; j <= data.nInfoFiles(); j++) {
//       const E00::IFO &ifo = data.getIFO(j);
//       cout << "IFO file: " << ifo.fileName << endl;
//       for (k = 0; k < ifo.numItems; k++) {
// 	cout << "  " << ifo.defs[k].itemName << endl;
//       }
//     }


				// Go over the polygons
    cout << "Looking for largest polygon..." << endl;
    int maxPolySize = 0;
    for (j = 2; j <= data.nPolygons(); j++) {
      int size = 0;
      const E00::PAL &pal = data.getPAL(j);
      for (k = 0; k < pal.numArcs; k++) {
	int arcNum = pal.arcs[k].arcNum;
	if (arcNum == 0) {
	  // contour boundary; do nothing
	} else if (arcNum < 0) {
	  const E00::ARC &arc = data.getARC(0-arcNum);
	  size += arc.numberOfCoordinates;
	} else {
	  const E00::ARC &arc = data.getARC(arcNum);
	  size += arc.numberOfCoordinates;
	}
      }
      if (size > maxPolySize)
	maxPolySize = size;
      if (size > 1000) {
	cout << "** Large polygon " << j << ": " << size << " points" << endl;
      }
    }

    cout << "Largest polygon (excluding enclosing polygon) has "
	 << maxPolySize << " points" << endl;

				// Go over the line segments
    cout << "Looking for longest line segment..." << endl;
    int maxLineSize = 0;
    for (j = 1; j <= data.nLines(); j++) {
      const E00::ARC &arc = data.getARC(j);
      if (arc.numberOfCoordinates > maxLineSize)
	maxLineSize = arc.numberOfCoordinates;
    }

    cout << "Largest line segment (outside of polygon) has "
	 << maxLineSize << " points" << endl;
  } 

  return 0;
}

// end of teste00.cxx
