
#include <fstream>
#include "e00.hxx"

using std::ifstream;

int main (int ac, const char ** av)
{
  for (int i = 1; i < ac; i++) {
    cerr << "Reading " << av[i] << endl;
    ifstream input(av[i]);
    E00 data;
    try {
      data.readE00(input);
    } catch (E00Exception &e) {
      cerr << "Reading " << av[i] << " failed with exception "
	   << e.message << endl;
      exit(1);
    }
    cerr << "Read " << av[i] << " successfully" << endl;
    cerr << "Read " << data.nPoints() << " point(s)" << endl;
    cerr << "Read " << data.nLines() << " line segment(s)" << endl;
    cerr << "Read " << data.nPolygons() << " polygon(s)" << endl;
    cerr << " (including enclosing polygon)" << endl;

    data.write(cout);
  } 

  return 0;
}
