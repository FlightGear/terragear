// line.cxx - implementation of VpfLine
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "vpfbase.hxx"
#include "line.hxx"
#include "table.hxx"
#include "value.hxx"

#include <string>
using std::string;


VpfLine::VpfLine (int lineId, const std::string &path,
		  VpfTableManager &tableManager)
  : VpfComponent(tableManager, path),
    _line_id(lineId),
    _ebr(0),
    _edg(0),
    _value(0)
{
}

VpfLine::VpfLine (const VpfLine &line)
  : VpfComponent(line.getTableManager(), line.getPath()),
    _line_id(line._line_id),
    _ebr(copyTable(line._ebr)),
    _edg(copyTable(line._edg)),
    _value(0)
{
}

VpfLine::~VpfLine ()
{
  freeTable(_ebr);
  freeTable(_edg);
}

int
VpfLine::getPointCount () const
{
  return getValue().getElementCount();
}

const VpfRectangle
VpfLine::getBoundingRectangle () const
{
  const VpfTable &ebr = getEBR();
  VpfRectangle rect;
  int row = ebr.findMatch("id", _line_id);
  rect.minX = ebr.getValue(row, "xmin").getReal();
  rect.minY = ebr.getValue(row, "ymin").getReal();
  rect.maxX = ebr.getValue(row, "xmax").getReal();
  rect.maxY = ebr.getValue(row, "ymax").getReal();
  return rect;
}

const VpfPoint
VpfLine::getPoint (int index) const
{
  return getValue().getPoint(index);
}

const VpfValue &
VpfLine::getValue () const
{
  if (_value == 0) {
    const VpfTable &edg = getEDG();
    int row = edg.findMatch("id", _line_id);
    if (row == -1)
      throw VpfException("Line does not exist");
    _value = &(edg.getValue(row, "coordinates"));
  }
  return *_value;
}

const VpfTable &
VpfLine::getEBR () const
{
  if (_ebr == 0)
    _ebr = getChildTable("ebr");
  return *_ebr;
}

const VpfTable &
VpfLine::getEDG () const
{
  if (_edg == 0)
    _edg = getChildTable("edg");
  return *_edg;
}

// end of line.cxx

