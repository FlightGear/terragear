// e00.hxx - declarations for E00 file processing.

#ifndef __E00_HXX
#define __E00_HXX 1

#include <simgear/compiler.h>

#include <vector>
#include <string>
#include <iostream>

// An exception reading an E00 file.

class E00Exception
{
public:
  E00Exception (const std::string &message) : _message(message) {}
  virtual const std::string &getMessage () const { return _message; }
private:
  std::string _message;
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
    std::vector<Coord> coordinates;
    bool in_polygon;
  };


  // Polygon Centroid Coordinates

  struct CNT
  {
    virtual ~CNT () {}
    int numLabels;
    Coord centroid;
    std::vector<int> labels;
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
    std::vector<std::string> lines;
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
    std::vector<ARCref> arcs;
  };


  // Projection Parameters

  struct PRJ
  {
    virtual ~PRJ () {}
    std::vector<std::string> lines;
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
      std::string itemName;
      int itemWidth;		// followed by -1
      int itemStartPos;		// followed by 4-1
      int itemOutputFormat[2];
      std::string itemType;
      // -1
      // -1-1
      std::string seqId;
    };
    typedef std::vector<std::string> Entry;
    std::string fileName;
    std::string isArcInfo;
    int numItems;
    int altNumItems;
    int dataRecordLength;
    int numDataRecords;
    std::vector<ItemDef> defs;
    std::vector<Entry> entries;
  };


  E00 ();
  virtual ~E00 ();

  virtual void readE00 (std::istream &input);

  virtual std::string getPathName () const { return pathName; }

  virtual int nPoints () const { return lab_section.size(); }
  virtual int nLines () const { return arc_section.size(); }
  virtual int nPolygons () const { return pal_section.size(); }
  virtual int nInfoFiles () const { return ifo_section.size(); }

  virtual const ARC &getARC (int i) const { return arc_section[i-1]; }
  virtual const LAB &getLAB (int i) const { return lab_section[i-1]; }
  virtual const PAL &getPAL (int i) const { return pal_section[i-1]; }
  virtual const IFO &getIFO (int i) const { return ifo_section[i-1]; }
  virtual const IFO * getIFO (const std::string &name) const;
  virtual const std::string * getIFOItem (const std::string &fileName, int entry,
				          const std::string &itemName) const;
  virtual const std::string * getIFOItemType (const std::string &fileName,
					      const std::string &itemName) const;
    

private:

  virtual ARC &_getARC (int i) { return arc_section[i-1]; }

  std::string pathName;
  std::vector<ARC> arc_section;
  std::vector<CNT> cnt_section;
  std::vector<LAB> lab_section;
  std::vector<LOG> log_section;
  std::vector<PAL> pal_section;
  std::vector<PRJ> prj_section;
  std::vector<TOL> tol_section;
  std::vector<IFO> ifo_section;

  mutable std::istream * _input;

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
