// contour.cxx - implementation of VpfContour

#include "contour.hxx"
#include "table.hxx"
#include "line.hxx"

#include <string>
#include <vector>

using std::string;
using std::vector;

VpfContour::VpfContour (int faceId, int startEdge, const string &path,
			VpfTableManager &tableManager)
  : VpfComponent(tableManager, path),
    _polygon_id(faceId),
    _start_edge(startEdge),
    _nPoints(0),
    _edg(0),
    _lines(0)
{
}

VpfContour::VpfContour (const VpfContour &contour)
  : VpfComponent(contour.getTableManager(), contour.getPath()),
    _polygon_id(contour._polygon_id),
    _start_edge(contour._start_edge),
    _nPoints(0),
    _edg(copyTable(contour._edg)),
    _lines(0)
{
}

VpfContour::~VpfContour ()
{
  freeTable(_edg);
  delete _lines;
}

int
VpfContour::getPointCount () const
{
  getLines();
  return _nPoints;
}

const VpfPoint
VpfContour::getPoint (int index) const
{
  const vector<line_info> &lines = getLines();
  int nLines = lines.size();
  for (int i = 0; i < nLines; i++) {
    if (index < (lines[i].offset + lines[i].size)) {
      VpfLine line(lines[i].id, getPath(), getTableManager());
      if (lines[i].isLR)
	return line.getPoint(index - lines[i].offset);
      else
	return line.getPoint(lines[i].offset + lines[i].size - index - 1);
    }
  }
  throw VpfException("contour point out of range");
}

const VpfTable &
VpfContour::getEDG () const
{
  if (_edg == 0)
    _edg = getChildTable("edg");
  return *_edg;
}

const vector<VpfContour::line_info> &
VpfContour::getLines () const
{
  if (_lines == 0) {
    _lines = new vector<line_info>;
    const VpfTable &edg = getEDG();
    int current_offset = 0;
    int current_edge = _start_edge;
    // std::cout << "start edge: " << _start_edge << std::endl;
    int previous_edge = -1;
    do {
      int row = edg.findMatch("id", current_edge);
      line_info info;
      info.id = current_edge;
      info.offset = current_offset;
      info.size = edg.getValue(row, "coordinates").getElementCount();
      current_offset += info.size;
      _nPoints += info.size;

      previous_edge = current_edge;
      if (edg.getValue(row, "right_face")
	  .getCrossRef().current_tile_key == _polygon_id) {
        info.isLR = true;
        current_edge = edg.getValue(row, "right_edge")
          .getCrossRef().current_tile_key;
        if (edg.getValue(row, "left_face")
            .getCrossRef().current_tile_key == _polygon_id)
          std::cout << "right face also left face" << std::endl;
      } else if (edg.getValue(row, "left_face")
          .getCrossRef().current_tile_key == _polygon_id) {
        info.isLR = false;
        current_edge = edg.getValue(row, "left_edge")
          .getCrossRef().current_tile_key;
      } else {
	throw VpfException("edge does not belong to face");
      }

      for ( vector<line_info>::reverse_iterator it = _lines->rbegin();
            it != _lines->rend(); it++ )
      {
        if ( it->id == info.id )
        {
          std::cout << "eeek!!! infinite loop..." << std::endl;          
          _nPoints -= info.size;
          return *_lines;
        }
      }

      _lines->push_back(info);
    } while (current_edge != _start_edge);
  }
  return *_lines;
}

// end of contour.cxx
