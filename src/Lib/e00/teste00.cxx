// teste00.cxx - test the E00 parsing routines and dump some results.

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/misc/sgstream.hxx>
#include "e00.hxx"

int main (int ac, const char ** av)
{
  for (int i = 1; i < ac; i++) {
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
    cerr << "Read " << av[i] << " successfully" << endl;
    cerr << "Read " << data.nPoints() << " point(s)" << endl;
    cerr << "Read " << data.nLines() << " line segment(s)" << endl;
    cerr << "Read " << data.nPolygons() << " polygon(s)" << endl;
    cerr << " (including enclosing polygon)" << endl;

    for (int i = 1; i <= data.nInfoFiles(); i++) {
      const E00::IFO &ifo = data.getIFO(i);
      cout << "IFO file: " << ifo.fileName << endl;
      for (int j = 0; j < ifo.numItems; j++) {
	cout << "  " << ifo.defs[j].itemName << endl;
      }
    }

//     for (int i = 2; i <= data.nPolygons(); i++) {
//       const E00::PAL &pal = data.getPAL(i);
//       for (int j = 0; j < pal.numArcs; j++) {
// 	int arcNum = pal.arcs[j].arcNum;
// 	if (arcNum == 0) {
// 	  cout << endl;
// 	} else if (arcNum < 0) {
// 	  const E00::ARC &arc = data.getARC(0-arcNum);
// 	  for (int k = arc.numberOfCoordinates - 1; k >= 0; k--) {
// 	    cout << arc.coordinates[k].x << '\t'
// 		 << arc.coordinates[k].y << endl;
// 	  }
// 	} else {
// 	  const E00::ARC &arc = data.getARC(arcNum);
// 	  for (int k = 0; k < arc.numberOfCoordinates; k++) {
// 	    cout << arc.coordinates[k].x << '\t'
// 		 << arc.coordinates[k].y << endl;
// 	  }
// 	}
// 	cout << endl;
//       }
//     }
  } 

  return 0;
}

// end of teste00.cxx
