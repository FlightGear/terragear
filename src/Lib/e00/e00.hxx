// e00.hxx - declarations for E00 file processing.

#ifndef __E00_HXX
#define __E00_HXX 1

#include <simgear/compiler.h>

#include <vector>
#include STL_STRING
#include STL_IOSTREAM

SG_USING_STD(vector);
SG_USING_STD(string);
SG_USING_STD(istream);


// An exception reading an E00 file.

class E00Exception
{
public:
  E00Exception (const string &message) : _message(message) {}
  virtual const string &getMessage () const { return _message; }
private:
  string _message;
};



// ARCInfo file

class E00 {

public:


  //////////////////////////////////////////////////////////////////////
  // Data structures for internal use.
  //////////////////////////////////////////////////////////////////////


  // A coordinate in two-dimensional space.

  struct Coord
  {
    virtual ~Coord () {}
    double x;
    double y;
  };


  // Arc co-ordinates and topology.

  struct ARC
  {
    virtual ~ARC () {}
    int coverageNum;
    int coverageId;
    int fromNode;
    int toNode;
    int leftPolygon;
    int rightPolygon;
    int numberOfCoordinates;
    vector<Coord> coordinates;
    bool in_polygon;
  };


  // Polygon Centroid Coordinates

  struct CNT
  {
    virtual ~CNT () {}
    int numLabels;
    Coord centroid;
    vector<int> labels;
  };


  // Label Point Coordinates and Topology

  struct LAB
  {
    virtual ~LAB () {}
    int coverageId;
    int enclosingPolygon;
    Coord coord;
    Coord box1;		// obsolete
    Coord box2;		// obsolete
  };


  // Coverage History

  struct LOG
  {
    virtual ~LOG () {}
    vector<string> lines;
  };


  // Polygon Topology

  struct PAL
  {
    virtual ~PAL () {}
    struct ARCref
    {
      int arcNum;
      int nodeNum;
      int polygonNum;
    };
    int numArcs;
    Coord min;
    Coord max;
    vector<ARCref> arcs;
  };


  // Projection Parameters

  struct PRJ
  {
    virtual ~PRJ () {}
    vector<string> lines;
  };


  // Tolerance Type

  struct TOL
  {
    virtual ~TOL () {}
    int type;
    int status;
    double value;
  };


  // Info Files

  struct IFO
  {
    virtual ~IFO () {}
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


  E00 ();
  virtual ~E00 ();

  virtual void readE00 (istream &input);

  virtual string getPathName () const { return pathName; }

  virtual int nPoints () const { return lab_section.size(); }
  virtual int nLines () const { return arc_section.size(); }
  virtual int nPolygons () const { return pal_section.size(); }
  virtual int nInfoFiles () const { return ifo_section.size(); }

  virtual const ARC &getARC (int i) const { return arc_section[i-1]; }
  virtual const LAB &getLAB (int i) const { return lab_section[i-1]; }
  virtual const PAL &getPAL (int i) const { return pal_section[i-1]; }
  virtual const IFO &getIFO (int i) const { return ifo_section[i-1]; }
  virtual const IFO * getIFO (const string &name) const;
  virtual const string * getIFOItem (const string &fileName, int entry,
				     const string &itemName) const;
  virtual const string * getIFOItemType (const string &fileName,
					 const string &itemName) const;
    

private:

  virtual ARC &_getARC (int i) { return arc_section[i-1]; }

  string pathName;
  vector<ARC> arc_section;
  vector<CNT> cnt_section;
  vector<LAB> lab_section;
  vector<LOG> log_section;
  vector<PAL> pal_section;
  vector<PRJ> prj_section;
  vector<TOL> tol_section;
  vector<IFO> ifo_section;

  mutable istream * _input;

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
  void readTX6 ();
  void readTX7 ();
  void readRXP ();
  void readRPL ();
  void readIFO ();
  void readUnknown ();

};

#endif // __E00_HXX

// end of e00.hxx
