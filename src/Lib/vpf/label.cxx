// label.cxx - implementation of VpfLabel
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "label.hxx"

#include "vpfbase.hxx"
#include "table.hxx"
#include "value.hxx"

#include <string>
using std::string;

VpfLabel::VpfLabel (int labelId, const string &path,
		    VpfTableManager &tableManager)
  : VpfComponent(tableManager, path),
    _label_id(labelId),
    _txt(0)
{
}

VpfLabel::VpfLabel (const VpfLabel &table)
  : VpfComponent(table.getTableManager(), table.getPath()),
    _label_id(table._label_id),
    _txt(copyTable(table._txt))
{
}

VpfLabel::~VpfLabel ()
{
  freeTable(_txt);
}

const char *
VpfLabel::getText () const
{
  const VpfTable &txt = getTXT();
  int row = txt.findMatch("id", _label_id);
  return txt.getValue(row, "string").getText();
}

const VpfPoint
VpfLabel::getPoint () const
{
  const VpfTable &txt = getTXT();
  int row = txt.findMatch("id", _label_id);
  return txt.getValue(row, "shape_line").getPoint(0);
}

const VpfTable &
VpfLabel::getTXT () const
{
  if (_txt == 0)
    _txt = getChildTable("txt");
  return *_txt;
}

// end of label.cxx
