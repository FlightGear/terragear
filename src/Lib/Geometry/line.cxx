// line.cxx - a simple multi-segment line class.
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.


#include "line.hxx"

Line::Line ()
{
}

Line::Line (const Line &l)
{
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

// end of line.cxx
