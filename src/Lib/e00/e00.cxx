// e00.cxx: implementation of ArcInfo (e00) reader.

#include <simgear/compiler.h>

#include "e00.hxx"

#include <vector>
#include <map>
#include STL_STRING
#include STL_IOSTREAM
#include <cctype>

#include <stdio.h>

SG_USING_STD(vector);
SG_USING_STD(map);
SG_USING_STD(string);
SG_USING_STD(istream);
SG_USING_STD(cerr);
SG_USING_STD(endl);
SG_USING_STD(getline);


////////////////////////////////////////////////////////////////////////
// Static helper functions.
////////////////////////////////////////////////////////////////////////

/**
 * Append an integer to a string.
 */
static void
strAppend (string &s, int i)
{
  char buf[128];
  sprintf(buf, "%d", i);
  s += buf;
}


/**
 * Append a double-precision real to a string.
 */
static void
strAppend (string &s, double f)
{
  char buf[128];
  sprintf(buf, "%f", f);
  s += buf;
}


/**
 * Skip newlines.
 *
 * This function is used only by readIFO.  It has to track line
 * position because info records must be padded to 80 characters
 * (really!) -- another reason to hate E00 format.
 */
static void
skipNewlines (istream &input, int * line_pos)
{
  char c;
  input.get(c);
				// Doesn't seem to be needed.
//   while (isspace(c)) {
//     input.get(c);
//   }
  while (c == '\n' || c == '\r') {
    input.get(c);
    *line_pos = 0;
  }
  input.putback(c);
}


/**
 * Read a fixed-width item from the input stream.
 *
 * This function is used only by readIFO, where info records follow
 * declared widths.  It has to track line position and pad up to
 * 80 characters where necessary (actually, 79, since the newline
 * is excluded); this matters mainly when a string field crosses
 * record boundaries, as is common in DCW point coverages.
 */
static void
readItem (istream &input, string &line, int width, int * line_pos)
{
  char c;
  
  line.resize(0);
  
  for (int i = 0; i < width; i++) {
    input.get(c);
    (*line_pos)++;
    if (c == '\n' || c == '\r')	{ // premature termination
      (*line_pos)--;
      i--;
      while (*line_pos < 80 && i < width) {
	line += ' ';
	(*line_pos)++;
	i++;
      }
      if (*line_pos == 80)
	*line_pos = 0;
      else
	input.putback(c);
    } else {
      line += c;
    }
  }
}


/**
 * Check that a precision is known, or throw an exception.
 */
static void
checkPrecision (istream &input)
{
  int i;

  input >> i;
  if (i != 2 && i != 3)
    throw E00Exception("Unknown precision");
}


/**
 * Check that six zeros appear where expected, or throw an exception.
 */
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


/**
 * Check that the expected integer appears, or throw an exception.
 */
static void
expect (istream &input, int i)
{
  int in;
  input >> in;
  if (in != i) {
    string message = "Expected ";
    strAppend(message, i);
    throw E00Exception(message.c_str());
  }
}


/**
 * Check that the expected real number appears, or throw an exception.
 */
static void
expect (istream &input, double f)
{
  double in;
  input >> in;
  if (in != f) {
    string message = "Expected ";
    strAppend(message, f);
    throw E00Exception(message.c_str());
  }
}


/**
 * Check that the expected string appears, or throw an exception.
 */
static void
expect (istream &input, const char *s)
{
  string in;
  input >> in;
  if (in != string(s)) {
    string message = "Expected ";
    message += s;
    throw E00Exception(message.c_str());
  }
}



////////////////////////////////////////////////////////////////////////
// Implementation of E00
////////////////////////////////////////////////////////////////////////


E00::E00 ()
  : _input(0)
{
}


E00::~E00 ()
{
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

    cerr << "Reading " << token << " section" << endl;
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
    } else if (token == "TX6") {
      readTX6();
    } else if (token == "TX7") {
      readTX7();
    } else if (token == "RXP") {
      readRXP();
    } else if (token == "RPL") {
      readRPL();
    } else if (token == "EOS") {
      postProcess();
      return;
    } else {
      cerr << "Skipping unknown section type " << token << endl;
      readUnknown();
    }
  }

  throw E00Exception("File ended without EOS line");
  _input = 0;
}


/**
 * Read the header of an E00 file.
 */
void
E00::readHeader ()
{
  expect(*_input, "EXP");
  expect(*_input, 0);
  *_input >> pathName;
}


/**
 * Read the ARC section of an E00 file.
 */
void
E00::readARC ()
{
  ARC arc;
  Coord coord;

  checkPrecision(*_input);
  *_input >> arc.coverageNum;
  while (arc.coverageNum != -1) {
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
    arc.in_polygon = false;	// we'll check later
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
  int numLabels;
  int label;

  checkPrecision(*_input);

  *_input >> numLabels;

  while (numLabels != -1) {
    CNT cnt;
    cnt.numLabels = numLabels;
    *_input >> cnt.centroid.x;
    *_input >> cnt.centroid.y;
    for (int i = 0; i < cnt.numLabels; i++) {
      *_input >> label;
      cnt.labels.push_back(label);
    }
    cnt_section.push_back(cnt);
    *_input >> numLabels;
  }

  checkZeros(*_input);
}


void
E00::readLAB ()
{
  LAB lab;

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
  LOG log;
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
  PAL pal;
  PAL::ARCref arc;
  int count = 1;

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
      if (count > 1) {
	if (arc.arcNum > 0)
	  _getARC(arc.arcNum).in_polygon = true;
	else
	  _getARC(0-arc.arcNum).in_polygon = true;
      }
      int num = (arc.arcNum < 0 ? 0 - arc.arcNum : arc.arcNum);
      if (num != 0 &&
	  getARC(num).leftPolygon != count &&
	  getARC(num).rightPolygon != count) {
	cerr << "Polygon " << count << " includes arc " << num
	     << " which doesn't reference it" << endl;
      }
      *_input >> arc.nodeNum;
      *_input >> arc.polygonNum;
      pal.arcs.push_back(arc);
    }

    pal_section.push_back(pal);
    *_input >> pal.numArcs;
    count++;
  }

  checkZeros(*_input);
}


void
E00::readPRJ ()
{
  PRJ prj;
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
  TOL tol;

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
E00::readTX6 ()
{
  string dummy;
  *_input >> dummy;
				// FIXME: will fail if "JABBERWOCKY" appears
				// in the text annotation itself
  while (dummy != string("JABBERWOCKY"))
    *_input >> dummy;
}


void
E00::readTX7 ()
{
  string dummy;
  *_input >> dummy;
				// FIXME: will fail if "JABBERWOCKY" appears
				// in the text annotation itself
  while (dummy != string("JABBERWOCKY"))
    *_input >> dummy;
}

void
E00::readRXP ()
{
  string dummy;
  *_input >> dummy;
				// FIXME: will fail if "JABBERWOCKY" appears
				// in the text annotation itself
  while (dummy != string("JABBERWOCKY"))
    *_input >> dummy;
}

void
E00::readRPL ()
{
  string dummy;
  *_input >> dummy;
				// FIXME: will fail if "JABBERWOCKY" appears
				// in the text annotation itself
  while (dummy != string("JABBERWOCKY"))
    *_input >> dummy;
}


//
// This method relies heavily on the readItem and skipNewlines
// static functions defined above; it needs to be able to read
// fixed-width fields, padding line length up to 80 where necessary.
//
void
E00::readIFO ()
{
  int line_pos = 0;
  string line = "";
  int intval;
  double realval;

  checkPrecision(*_input);

  while (line == "")
    *_input >> line;

  while (line != string("EOI")) {
    int i;
				// Start of a new IFO file.
    IFO ifo;
    IFO::Entry entry;
    ifo.fileName = line;

//     cout << "Reading IFO file " << line << endl;

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
    for (i = 0; i < ifo.numItems; i++) {
      IFO::ItemDef def;

      *_input >> def.itemName;
      *_input >> def.itemWidth;
      expect(*_input, -1);
      *_input >> def.itemStartPos;
      expect(*_input, -1);
      def.itemStartPos -= 4;
      def.itemStartPos /= 10;
      *_input >> def.itemOutputFormat[0];
      *_input >> def.itemOutputFormat[1];
      *_input >> def.itemType;
      expect(*_input, -1);
      expect(*_input, -1);
      expect(*_input, -1);
      *_input >> def.seqId;
      ifo.defs.push_back(def);
      getline(*_input, line);
    }

				// Read the data records
    ifo.entries.resize(0);
    for (i = 0; i < ifo.numDataRecords; i++) {
//       cout << " Reading entry " << i << endl;
      entry.resize(0);
      line_pos = 0;
      skipNewlines(*_input, &line_pos);
      for (int j = 0; j < ifo.numItems; j++) {
	line.resize(0);
	string &type = ifo.defs[j].itemType;

	if (type == "10-1") {	// date
	  readItem(*_input, line, 8, &line_pos);
	}

	else if (type == "20-1") {	// character field
	  readItem(*_input, line, ifo.defs[j].itemOutputFormat[0], &line_pos);
	} 

	else if (type == "30-1") { // fixed-width integer
	  readItem(*_input, line, ifo.defs[j].itemOutputFormat[0], &line_pos);
	} 

	else if (type == "40-1") { // single-precision float
	  readItem(*_input, line, 14, &line_pos);
	}

	else if (type == "50-1") { // integer
	  if (ifo.defs[j].itemWidth == 2) {
	    readItem(*_input, line, 6, &line_pos);
	  } else if (ifo.defs[j].itemWidth == 4) {
	    readItem(*_input, line, 11, &line_pos);
	  } else {
	    cerr << "Unexpected width " << ifo.defs[j].itemWidth
		 << " for item of type 50-1" << endl;
	    exit(1);
	  }
	} 

	else if (type == "60-1") { // real number
	  if (ifo.defs[j].itemWidth == 4) {
	    readItem(*_input, line, 14, &line_pos);
	  } else if (ifo.defs[j].itemWidth == 8) {
	    readItem(*_input, line, 24, &line_pos);
	  } else {
	    cerr << "Unexpected width " << ifo.defs[j].itemWidth
		 << " for item of type 60-1" << endl;
	    exit(1);
	  }
	} 

	else {		// assume integer
	  cerr << "Unknown IFO item type " << type
	       << " assuming integer" << endl;
	  exit(1);
	}
// 	cout << "  Read item " << j << ": '" << line << '\'' << endl;
	entry.push_back(line);
      }
      ifo.entries.push_back(entry);
    }

    ifo_section.push_back(ifo);
    line = "";
    while (line == "")
      *_input >> line;
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
  // TODO
}



////////////////////////////////////////////////////////////////////////
// Other access methods.
////////////////////////////////////////////////////////////////////////

const E00::IFO *
E00::getIFO (const string &fileName) const
{
  for (int i = 0; i < ifo_section.size(); i++) {
    if (ifo_section[i].fileName == fileName)
      return &(ifo_section[i]);
  }
  return 0;
}

const string *
E00::getIFOItem (const string &fileName, int entry,
		 const string &itemName) const
{
  const IFO * ifo = getIFO(fileName);
  if (ifo == 0)
    return 0;

  int pos = -1;
  for (int i = 0; i < ifo->defs.size(); i++) {
    if (ifo->defs[i].itemName == itemName)
      pos = i;
  }

  if (pos == -1)
    return 0;

  return &(ifo->entries[entry-1][pos]);
}

const string *
E00::getIFOItemType (const string &fileName, const string &itemName) const
{
  const IFO * ifo = getIFO(fileName);
  if (ifo == 0)
    return 0;

  int pos = -1;
  for (int i = 0; i < ifo->defs.size(); i++) {
    if (ifo->defs[i].itemName == itemName)
      return &(ifo->defs[i].itemType);
  }

  return 0;
}

// end of e00.cxx
