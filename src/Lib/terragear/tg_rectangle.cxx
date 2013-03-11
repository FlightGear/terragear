// rectangle.cxx - a simple rectangle class (for bounds, etc.)
//
// Started by David Megginson, July 2002
//
// This file is in the Public Domain and comes with NO WARRANTY OF ANY KIND.

#include "tg_rectangle.hxx"

tgRectangle::tgRectangle()
{
}

tgRectangle::tgRectangle (const tgRectangle &r)
  : _min(r.getMin()),
    _max(r.getMax())
{
}

tgRectangle::tgRectangle (const SGGeod &min, const SGGeod &max)
  : _min(min),
    _max(max)
{
}

tgRectangle::~tgRectangle ()
{
}

void tgRectangle::setMin (const SGGeod &p)
{
    _min = p;
}

void tgRectangle::setMax (const SGGeod &p)
{
    _max = p;
}

void tgRectangle::expandBy(const tgRectangle& r)
{
    if ( r.getMin().getLongitudeDeg() < _min.getLongitudeDeg() ) {
        _min.setLongitudeDeg( r.getMin().getLongitudeDeg() );
    }
    if ( r.getMin().getLatitudeDeg() < _min.getLatitudeDeg() ) {
        _min.setLatitudeDeg( r.getMin().getLatitudeDeg() );
    }

    if ( r.getMax().getLongitudeDeg() > _max.getLongitudeDeg() ) {
        _max.setLongitudeDeg( r.getMax().getLongitudeDeg() );
    }
    if ( r.getMax().getLatitudeDeg() > _max.getLatitudeDeg() ) {
        _max.setLatitudeDeg( r.getMax().getLatitudeDeg() );
    }
}

void tgRectangle::sanify ()
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

bool tgRectangle::isInside (const SGGeod &p) const
{
    return ((p.getLongitudeDeg() >= _min.getLongitudeDeg() && p.getLongitudeDeg() <= _max.getLongitudeDeg()) &&
            (p.getLatitudeDeg() >= _min.getLatitudeDeg() && p.getLatitudeDeg() <= _max.getLatitudeDeg()));
}

bool tgRectangle::intersects (const tgRectangle &r) const
{
    const SGGeod &min = r.getMin();
    const SGGeod &max = r.getMax();

    return ((max.getLongitudeDeg() >= _min.getLongitudeDeg()) && (min.getLongitudeDeg() <= _max.getLongitudeDeg()) &&
            (max.getLatitudeDeg() >= _min.getLatitudeDeg()) && (min.getLatitudeDeg() <= _max.getLatitudeDeg()));
}

// end of rectangle.cxx
