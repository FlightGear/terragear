// vpfbase.cxx - basic support classes and structures.
// This file is released into the Public Domain, and comes with NO WARRANTY!


#include <string>

#include "vpfbase.hxx"

using std::string;



////////////////////////////////////////////////////////////////////////
// Point and rectangle functions.
////////////////////////////////////////////////////////////////////////

bool
inside (const VpfPoint &p, const VpfRectangle &r)
{
  return (p.x >= r.minX &&
	  p.x <= r.maxX &&
	  p.y >= r.minY &&
	  p.y <= r.maxY);
}

static inline bool
inside (double point, double min, double max)
{
  return (point >= min && point <= max);
}

static inline bool 
overlap (double min1, double max1, double min2, double max2)
{
  return (inside(min1, min2, max2) ||
	  inside(max1, min2, max2) ||
	  inside(min2, min1, max1) ||
	  inside(max2, min1, min2));
}

bool
overlap (const VpfRectangle &r1, const VpfRectangle &r2)
{
  return (overlap(r1.minX, r1.maxX, r2.minX, r2.maxX) &&
	  overlap(r1.minY, r1.maxY, r2.minY, r2.maxY));
}



////////////////////////////////////////////////////////////////////////
// Implementation of VpfException.
////////////////////////////////////////////////////////////////////////

VpfException::VpfException ()
  : _message("VPF error")
{
}

VpfException::VpfException (const std::string &message)
  : _message(message)
{
}

VpfException::~VpfException ()
{
}

const string &
VpfException::getMessage () const
{
  return _message;
}

// end of vpfbase.cxx
