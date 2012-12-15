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

Rectangle::Rectangle (const SGGeod &min, const SGGeod &max)
  : _min(min),
    _max(max)
{
}

Rectangle::~Rectangle ()
{
}

void
Rectangle::setMin (const SGGeod &p)
{
  _min = p;
}

void
Rectangle::setMax (const SGGeod &p)
{
  _max = p;
}

void
Rectangle::sanify ()
{
  double tmp;
  if (_min.getLongitudeDeg() > _max.getLongitudeDeg()) {
    tmp = _min.getLongitudeDeg();
    _min.setLongitudeDeg(_max.getLongitudeDeg());
    _max.setLongitudeDeg(tmp);
  }
  if (_min.getLatitudeDeg() > _max.getLatitudeDeg()) {
    tmp = _min.getLatitudeDeg();
    _min.setLatitudeDeg(_max.getLatitudeDeg());
    _max.setLatitudeDeg(tmp);
  }
}

bool
Rectangle::isInside (const SGGeod &p) const
{
    return ((p.getLongitudeDeg() >= _min.getLongitudeDeg() && p.getLongitudeDeg() <= _max.getLongitudeDeg()) &&
    (p.getLatitudeDeg() >= _min.getLatitudeDeg() && p.getLatitudeDeg() <= _max.getLatitudeDeg()));
}

bool
Rectangle::intersects (const Rectangle &r) const
{
  const SGGeod &min = r.getMin();
  const SGGeod &max = r.getMax();
  return ((max.getLongitudeDeg() >= _min.getLongitudeDeg()) && (min.getLongitudeDeg() <= _max.getLongitudeDeg()) &&
  (max.getLatitudeDeg() >= _min.getLatitudeDeg()) && (min.getLatitudeDeg() <= _max.getLatitudeDeg()));
}

/*const TGPolygon
Rectangle::toPoly () const
{
  TGPolygon poly;
  poly.add_node(0, _min);
  poly.add_node(0, Point3D(_max.x(), _min.y(), 0));
  poly.add_node(0, _max);
  poly.add_node(0, Point3D(_min.x(), _max.y(), 0));
  return poly;
}*/

};

// end of rectangle.cxx
