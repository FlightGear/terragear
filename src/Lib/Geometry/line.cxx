// line.cxx - a simple multi-segment line class.
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.


#include "line.hxx"

namespace tg {

Line::Line ()
{
}

Line::Line (const Line &l)
{
    int nPoints = l.getPointCount();
    for (int i = 0; i < nPoints; i++)
        addPoint(l.getPoint(i));
}

Line::~Line ()
{
}

int
Line::getPointCount () const
{
  return _points.size();
}

const Point3D &
Line::getPoint (int index) const
{
  return _points[index];
}

Point3D &
Line::getPoint (int index)
{
  return _points[index];
}

void
Line::addPoint (const Point3D &point)
{
  _points.push_back(point);
}

Rectangle
Line::getBounds () const
{
  Point3D min;
  Point3D max;

  int nPoints = _points.size();
  for (int i = 0; i < nPoints; i++) {
    if (i == 0) {
      min = max = _points[i];
    } else {
      if (_points[i].x() < min.x())
	min.setx(_points[i].x());
      if (_points[i].x() > max.x())
	max.setx(_points[i].x());
      if (_points[i].y() < min.y())
	min.sety(_points[i].y());
      if (_points[i].y() > max.y())
	max.sety(_points[i].y());
    }
  }

  return Rectangle(min, max);
}

};

// end of line.cxx
