#include <vector>
#include <map>
#include <string>
#include <iostream>

using std::vector;
using std::map;
using std::string;
using std::istream;

struct E00Exception
{
  E00Exception (const char * msg) : message(msg) {}
  string message;
};


// A coordinate in two-dimensional space.

struct e00Coord
{
  double x;
  double y;
};


// Arc Coordinates and Topology

struct e00ARC
{
  bool inPolygon;
  int coverageNum;
  int coverageId;
  int fromNode;
  int toNode;
  int leftPolygon;
  int rightPolygon;
  int numberOfCoordinates;
  vector<e00Coord> coordinates;
};


// Polygon Centroid Coordinates

struct e00CNT
{
  int numLabels;
  e00Coord centroid;
  vector<int> labels;
};


// Label Point Coordinates and Topology

struct e00LAB
{
  int coverageId;
  int enclosingPolygon;
  e00Coord coord;
  e00Coord box1;		// obsolete
  e00Coord box2;		// obsolete
};


// Coverage History

struct e00LOG
{
  vector<string> lines;
};


// Polygon Topology

struct e00PAL
{
  struct ARC
  {
    int arcNum;
    int nodeNum;
    int polygonNum;
  };
  int numArcs;
  e00Coord min;
  e00Coord max;
  vector<ARC> arcs;
};


// Projection Parameters

struct e00PRJ
{
  vector<string> lines;
};


// Tolerance Type

struct e00TOL
{
  int type;
  int status;
  double value;
};


// Info Files

struct e00IFO
{
  struct ItemDef 
  {
    string itemName;
    int itemWidth;		// followed by -1
    int itemStartPos;		// followed by 4-1
    int itemOutputFormat[2];
    string itemType;
    // -1
    // -1-1
    string seqId;
  };
  typedef vector<string> Entry;
  string fileName;
  string isArcInfo;
  int numItems;
  int altNumItems;
  int dataRecordLength;
  int numDataRecords;
  vector<ItemDef> defs;
  vector<Entry> entries;
};


// ARCInfo file

class E00 {
public:

  E00 ();
  virtual ~E00 ();

  virtual void readE00 (istream &input);

  virtual string getPathName () const { return pathName; }

  virtual int nPoints () const;
  virtual int nLines () const;
  virtual int nPolygons () const;

  virtual const e00ARC * getPoint (int i) const { return pointArcs[i]; }
  virtual const e00ARC * getLine (int i) const { return lineArcs[i]; }

  virtual void write (ostream &output) const;

private:

  vector<const e00ARC *> pointArcs;
  vector<const e00ARC *> lineArcs;

  string pathName;
  vector<e00ARC> arc_section;
  vector<e00CNT> cnt_section;
  vector<e00LAB> lab_section;
  vector<e00LOG> log_section;
  vector<e00PAL> pal_section;
  vector<e00PRJ> prj_section;
  vector<e00TOL> tol_section;
  vector<e00IFO> ifo_section;

  istream * _input;

  void expect (int i);
  void expect (double f);
  void expect (const char * s);

  void postProcess ();

  void readHeader ();
  void readARC ();
  void readCNT ();
  void readLAB ();
  void readLOG ();
  void readPAL ();
  void readPRJ ();
  void readSIN ();
  void readTOL ();
  void readIFO ();
  void readUnknown ();
};
