// e00.cxx: implementation of ArcInfo (e00) reader.

#include "e00.hxx"

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <cctype>

#include <stdio.h>

using std::vector;
using std::map;
using std::string;
using std::istream;
using std::cerr;
using std::endl;


////////////////////////////////////////////////////////////////////////
// Static helper functions.
////////////////////////////////////////////////////////////////////////

static void
append (string &s, int i)
{
  char buf[128];
  sprintf(buf, "%d", i);
  s += buf;
}

static void
append (string &s, double f)
{
  char buf[128];
  sprintf(buf, "%f", f);
  s += buf;
}

static void
skipWhitespace (istream &input)
{
  char c;
  input.get(c);
  while (isspace(c)) {
    input.get(c);
  }
  input.putback(c);
}

static void
readStringItem (istream &input, string &line, int width)
{
  char c;
  
  skipWhitespace(input);
  line.resize(0);
  
  for (int i = 0; i < width; i++) {
    input.get(c);
    if (c == '\n' || c == '\r')	// premature termination
      return;
    else
      line += c;
  }
}

static void
checkPrecision (istream &input)
{
  int i;

  input >> i;
  if (i != 2 && i != 3)
    throw E00Exception("Unknown precision");
}

static void
checkZeros (istream &input)
{
  int i, j;

  for (i = 0; i < 6; i++) {
    input >> j;
    if (j != 0)
      throw E00Exception("Expected -1 followed by six zeros");
  }
}



////////////////////////////////////////////////////////////////////////
// Implementation of E00
////////////////////////////////////////////////////////////////////////


E00::E00 ()
{
}


E00::~E00 ()
{
}


void
E00::expect (int i)
{
  int in;
  *_input >> in;
  if (in != i) {
    string message = "Expected ";
    append(message, i);
    throw E00Exception(message.c_str());
  }
}

void
E00::expect (double f)
{
  double in;
  *_input >> in;
  if (in != f) {
    string message = "Expected ";
    append(message, f);
    throw E00Exception(message.c_str());
  }
}

void
E00::expect (const char * s)
{
  string in;
  *_input >> in;
  if (in != s) {
    string message = "Expected ";
    message += s;
    throw E00Exception(message.c_str());
  }
}



////////////////////////////////////////////////////////////////////////
// XML output.
////////////////////////////////////////////////////////////////////////

static void
writeVertex (ostream &output, double x, double y)
{
  output << "<v x=\"" << x << "\" y=\"" << y << "\"/>" << endl;
}

void
E00::write (ostream &output) const
{
  output << "<?xml version=\"1.0\"?>" << endl << endl;
  output << "<GIS>" << endl << endl;

  if (arc_section.size() == 0) {
    for (int i = 0; i < (int)lab_section.size(); i++) {
      output << "<point>" << endl;
      writeVertex(output, lab_section[i].coord.x, lab_section[i].coord.y);
      output << "</point>" << endl << endl;
    }
  }

  for (int i = 0; i < (int)arc_section.size(); i++) {
    const e00ARC &arc = arc_section[i];
    if (!arc.inPolygon) {
      output << "<line>" << endl;
      for (int j = 0; j < (int)arc.coordinates.size(); j++) {
	writeVertex(output, arc.coordinates[j].x, arc.coordinates[j].y);
      }
      output << "</line>" << endl << endl;;
    }
  }

				// NB: skip enclosing poly
  for (int i = 1; i < (int)pal_section.size(); i++) {
    const e00PAL &pal = pal_section[i];
    output << "<polygon>" << endl;
    for (int j = 0; j < pal.numArcs; j++) {
      bool isReversed = false;
      int arcNum = pal.arcs[j].arcNum;
      if (arcNum < 0) {
	arcNum = 0 - arcNum;
	isReversed = true;
      }
      const e00ARC &arc = arc_section[arcNum];
      output << "<arc coverage=\"" << arc.coverageId << "\">" << endl;
      if (isReversed) {
	for (int k = arc.numberOfCoordinates - 1; k >= 0; k--) {
	  writeVertex(output, arc.coordinates[k].x, arc.coordinates[k].y);
	}
      } else {
	for (int k = 0; k < arc.numberOfCoordinates; k++) {
	  writeVertex(output, arc.coordinates[k].x, arc.coordinates[k].y);
	}
      }
      output << "</arc>" << endl;
    }
    output << "</polygon>" << endl << endl;
  }

  output << "</GIS>" << endl;
}





////////////////////////////////////////////////////////////////////////
// Public query methods.
////////////////////////////////////////////////////////////////////////

int
E00::nPoints () const
{
  return lab_section.size();
}

int
E00::nLines () const
{
  return lineArcs.size();
}

int
E00::nPolygons () const
{
  return pal_section.size();
}



////////////////////////////////////////////////////////////////////////
// Reader.
////////////////////////////////////////////////////////////////////////

void
E00::readE00 (istream &input)
{
  string token;

  _input = &input;

  readHeader();

  while (!_input->eof()) {

    *_input >> token;

    if (token == "ARC") {
      readARC();
    } else if (token == "CNT") {
      readCNT();
    } else if (token == "LAB") {
      readLAB();
    } else if (token == "LOG") {
      readLOG();
    } else if (token == "PAL") {
      readPAL();
    } else if (token == "PRJ") {
      readPRJ();
    } else if (token == "SIN") {
      readSIN();
    } else if (token == "TOL") {
      readTOL();
    } else if (token == "IFO") {
      readIFO();
    } else if (token == "EOS") {
      postProcess();
      return;
    } else {
      cerr << "Skipping unknown section type " << token << endl;
      readUnknown();
    }
  }

  throw E00Exception("File ended without EOS line");
}


/**
 * Read the header of an E00 file.
 */
void
E00::readHeader ()
{
  expect("EXP");
  expect(0);
  *_input >> pathName;
}


/**
 * Read the ARC section of an E00 file.
 */
void
E00::readARC ()
{
  e00ARC arc;
  e00Coord coord;

  checkPrecision(*_input);

  *_input >> arc.coverageNum;
  while (arc.coverageNum != -1) {
    arc.inPolygon = false;
    *_input >> arc.coverageId;
    *_input >> arc.fromNode;
    *_input >> arc.toNode;
    *_input >> arc.leftPolygon;
    *_input >> arc.rightPolygon;
    *_input >> arc.numberOfCoordinates;
    arc.coordinates.resize(0);
    for (int i = 0; i < arc.numberOfCoordinates; i++) {
      *_input >> coord.x;
      *_input >> coord.y;
      arc.coordinates.push_back(coord);
    }
    arc_section.push_back(arc);
    *_input >> arc.coverageNum;
  }

  checkZeros(*_input);
}


/**
 * Read the CNT section of an E00 file.
 */
void
E00::readCNT ()
{
  e00CNT cnt;
  int label;

  checkPrecision(*_input);

  *_input >> cnt.numLabels;

  while (cnt.numLabels != -1) {
    *_input >> cnt.centroid.x;
    *_input >> cnt.centroid.y;
    for (int i = 0; i < cnt.numLabels; i++) {
      *_input >> label;
      cnt.labels.push_back(label);
    }
    cnt_section.push_back(cnt);
    *_input >> cnt.numLabels;
  }

  checkZeros(*_input);
}


void
E00::readLAB ()
{
  e00LAB lab;

  checkPrecision(*_input);
  *_input >> lab.coverageId;
  *_input >> lab.enclosingPolygon;
  *_input >> lab.coord.x;
  *_input >> lab.coord.y;
  while (lab.coverageId != -1) {
    *_input >> lab.box1.x;	// obsolete
    *_input >> lab.box1.y;	// obsolete
    *_input >> lab.box2.x;	// obsolete
    *_input >> lab.box2.y;	// obsolete
    lab_section.push_back(lab);
    *_input >> lab.coverageId;
    *_input >> lab.enclosingPolygon;
    *_input >> lab.coord.x;
    *_input >> lab.coord.y;
  }
}


void
E00::readLOG ()
{
  e00LOG log;
  string line;

  checkPrecision(*_input);
  getline(*_input, line);
  while (line.find("EOL") != 0) {

    if (line[0] == '~') {
      log_section.push_back(log);
      log.lines.resize(0);
    } else {
      log.lines.push_back(line);
    }

    getline(*_input, line);
  }
}


void
E00::readPAL ()
{
  e00PAL pal;
  e00PAL::ARC arc;

  checkPrecision(*_input);
  *_input >> pal.numArcs;
  while (pal.numArcs != -1) {
    *_input >> pal.min.x;
    *_input >> pal.min.y;
    *_input >> pal.max.x;
    *_input >> pal.max.y;
    pal.arcs.resize(0);
    for (int i = 0; i < pal.numArcs; i++) {
      *_input >> arc.arcNum;
      *_input >> arc.nodeNum;
      *_input >> arc.polygonNum;
      pal.arcs.push_back(arc);
    }

    pal_section.push_back(pal);
    *_input >> pal.numArcs;
  }

  checkZeros(*_input);
}


void
E00::readPRJ ()
{
  e00PRJ prj;
  string line;

  checkPrecision(*_input);
  getline(*_input, line);
  while (line.find("EOP") != 0) {

    if (line[0] == '~') {
      prj_section.push_back(prj);
      prj.lines.resize(0);
    } else {
      prj.lines.push_back(line);
    }

    getline(*_input, line);
  }
}


void
E00::readSIN ()
{
  string line;
  checkPrecision(*_input);
  getline(*_input, line);
  while (line.find("EOX") != 0) {
    getline(*_input, line);
  }
}


void
E00::readTOL ()
{
  e00TOL tol;

  checkPrecision(*_input);
  *_input >> tol.type;
  while (tol.type != -1) {
    *_input >> tol.status;
    *_input >> tol.value;
    tol_section.push_back(tol);
    *_input >> tol.type;
  }

  checkZeros(*_input);
}


void
E00::readIFO ()
{
  e00IFO ifo;
  e00IFO::ItemDef def;
  e00IFO::Entry entry;
  string line;
  int intval;
  double realval;

  checkPrecision(*_input);

  *_input >> ifo.fileName;
  while (ifo.fileName.find("EOI") != 0) {

				// 'XX' may be absent
    *_input >> ifo.isArcInfo;
    if (ifo.isArcInfo == "XX") {
      *_input >> ifo.numItems;
    } else {
      ifo.numItems = atoi(ifo.isArcInfo.c_str());
      ifo.isArcInfo = "";
    }
    *_input >> ifo.altNumItems;
    *_input >> ifo.dataRecordLength;
    *_input >> ifo.numDataRecords;

				// Read the item definitions
    ifo.defs.resize(0);
    for (int i = 0; i < ifo.numItems; i++) {
      *_input >> def.itemName;
      *_input >> def.itemWidth;
      expect(-1);
      *_input >> def.itemStartPos;
      expect(-1);
      def.itemStartPos -= 4;
      def.itemStartPos /= 10;
      *_input >> def.itemOutputFormat[0];
      *_input >> def.itemOutputFormat[1];
      *_input >> def.itemType;
      expect(-1);
      expect(-1);
      expect(-1);
      *_input >> def.seqId;
      ifo.defs.push_back(def);
      getline(*_input, line);
    }

				// Read the data records
    ifo.entries.resize(0);
    for (int i = 0; i < ifo.numDataRecords; i++) {
      entry.resize(0);
      for (int j = 0; j < ifo.numItems; j++) {
	line.resize(0);
	string &type = ifo.defs[j].itemType;
	if (type == "20-1") {	// character field
	  readStringItem(*_input, line, ifo.defs[j].itemOutputFormat[0]);
	} else if (type == "50-1") { // integer
	  *_input >> intval;
	  append(line, intval);
	} else if (type == "60-1") { // real number
	  *_input >> realval;
	  append(line, realval);
	} else {		// assume integer
	  cerr << "Unknown IFO item type '30-1': assuming integer" << endl;
	  *_input >> intval;
	  append(line, intval);
	}
	entry.push_back(line);
      }
      ifo.entries.push_back(entry);
    }

    ifo_section.push_back(ifo);
    *_input >> ifo.fileName;
  }
}


void
E00::readUnknown ()
{
  string line;
  getline(*_input, line);
  while (line.find("EOX") != 0) {
    getline(*_input, line);
  }
}


void
E00::postProcess ()
{

				// Flag the arcs so that we know what is
				// and isn't part of a polygon.
  for (int i = 0; i < (int)pal_section.size(); i++) {
    e00PAL &pal = pal_section[i];
    for (int j = 0; j < (int)pal.arcs.size(); j++) {
      int arcNum = pal.arcs[j].arcNum;
      if (arcNum >= (int)arc_section.size()) {
	cerr << "Polygon includes non-existent arc " << arcNum << endl;
      } else {
	arc_section[arcNum].inPolygon = true;
      }
    }
  }

				// Now, check which arcs aren't flagged
				// and assign them to the appropriate
				// lists.
  for (int i = 0; i < (int)arc_section.size(); i++) {
    e00ARC &arc = arc_section[i];
    if (!arc.inPolygon) {
	lineArcs.push_back(&arc);
    }
  }

}

// end of e00.cxx
