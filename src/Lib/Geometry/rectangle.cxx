// rectangle.cxx - a simple rectangle class (for bounds, etc.)
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#include "rectangle.hxx"

namespace tg {

Rectangle::Rectangle ()
{
}

Rectangle::Rectangle (const Rectangle &r)
  : _min(r.getMin()),
    _max(r.getMax())
{
}

Rectangle::Rectangle (const Point3D &min, const Point3D &max)
  : _min(min),
    _max(max)
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

bool
Rectangle::isOverlapping (const Rectangle &r) const
{
  const Point3D &min = r.getMin();
  const Point3D &max = r.getMax();
  return ((max.x() >= _min.x()) && (min.x() <= _max.x()) &&
	  (max.y() >= _min.y()) && (min.y() <= _max.y()));
}

const TGPolygon
Rectangle::toPoly () const
{
  TGPolygon poly;
  poly.add_node(0, _min);
  poly.add_node(0, Point3D(_max.x(), _min.y(), 0));
  poly.add_node(0, _max);
  poly.add_node(0, Point3D(_min.x(), _max.y(), 0));
  return poly;
}

};

// end of rectangle.cxx
