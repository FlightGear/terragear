// rectangle.cxx - a simple rectangle class (for bounds, etc.)
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#include "rectangle.hxx"

Rectangle::Rectangle ()
{
}

Rectangle::Rectangle (const Rectangle &r)
  : _min(r.getMin()),
    _max(r.getMax())
{
}

Rectangle::~Rectangle ()
{
}

void
Rectangle::setMin (const Point3D &p)
{
  _min = p;
}

void
Rectangle::setMax (const Point3D &p)
{
  _max = p;
}

void
Rectangle::sanify ()
{
  double tmp;
  if (_min.x() > _max.x()) {
    tmp = _min.x();
    _min.setx(_max.x());
    _max.setx(tmp);
  }
  if (_min.y() > _max.y()) {
    tmp = _min.y();
    _min.sety(_max.y());
    _max.sety(tmp);
  }
}

bool
Rectangle::isInside (const Point3D &p) const
{
  return ((p.x() >= _min.x() && p.x() <= _max.x()) &&
	  (p.y() >= _min.y() && p.y() <= _max.y()));
}

// end of rectangle.cxx
