// polygon.cxx - implementation of VpfPolygon
// This file is released into the Public Domain, and comes with NO WARRANTY!

#include "vpfbase.hxx"
#include "table.hxx"
#include "contour.hxx"
#include "polygon.hxx"

#include <string>
using std::string;

VpfPolygon::VpfPolygon (int polygonId, const std::string &path,
			VpfTableManager &tableManager)
  : VpfComponent(tableManager, path),
    _polygon_id(polygonId),
    _fbr(0),
    _rng(0)
{
}

VpfPolygon::VpfPolygon (const VpfPolygon &polygon)
  : VpfComponent(polygon.getTableManager(), polygon.getPath()),
    _polygon_id(polygon._polygon_id),
    _fbr(copyTable(polygon._fbr)),
    _rng(copyTable(polygon._rng))
{
}

VpfPolygon::~VpfPolygon ()
{
  freeTable(_fbr);
  freeTable(_rng);
}

const VpfRectangle
VpfPolygon::getBoundingRectangle () const
{
  const VpfTable &fbr = getFBR();
  VpfRectangle rect;
  int row = fbr.findMatch("id", _polygon_id);
  rect.minX = fbr.getValue(row, "xmin").getReal();
  rect.minY = fbr.getValue(row, "ymin").getReal();
  rect.maxX = fbr.getValue(row, "xmax").getReal();
  rect.maxY = fbr.getValue(row, "ymax").getReal();
  return rect;
}

int
VpfPolygon::getContourCount () const
{
  return getRNG().countMatches("face_id", _polygon_id);
}

const VpfContour
VpfPolygon::getContour (int contour) const
{
  const VpfTable &rng = getRNG();
  int row = rng.findMatch("face_id", _polygon_id, contour);
  return VpfContour(_polygon_id,
		    rng.getValue(row, "start_edge").getInt(),
		    getPath(),
		    getTableManager());
}

const VpfTable &
VpfPolygon::getFBR () const
{
  if (_fbr == 0)
    _fbr = getChildTable("fbr");
  return *_fbr;
}

const VpfTable &
VpfPolygon::getRNG () const
{
  if (_rng == 0)
    _rng = getChildTable("rng");
  return *_rng;
}

// end of polygon.cxx

