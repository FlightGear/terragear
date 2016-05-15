// Copyright (c) 2000  
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved. 
//
// This file is part of CGAL (www.cgal.org); you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 3 of the License,
// or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
// 
//
// Author(s)     : Andreas Fabri, Herve Bronnimann

#ifndef TG_POINTC2_H
#define TG_POINTC2_H

#include <CGAL/Origin.h>
#include <CGAL/Bbox_2.h>

template <typename K>
class TgPointC2 {
  typedef typename K::FT         FT;

private:
  FT  vec[2];
  int col;

public:
  TgPointC2()
    : col(0)
  {
    *vec = 0;
    *(vec+1) = 0;
  }
  TgPointC2(const FT x, const FT y, int c = 0)
    : col(c)
  {
    *vec = x;
    *(vec+1) = y;
  }
  const FT& x() const  { return *vec; }
  const FT& y() const { return *(vec+1); }
  FT & x() { return *vec; }
  FT& y() { return *(vec+1); }
  int color() const { return col; }
  int& color() { return col; }
  bool operator==(const TgPointC2 &p) const
  {
    return ( *vec == *(p.vec) )  && ( *(vec+1) == *(p.vec + 1) && ( col == p.col) );
  }
  bool operator!=(const TgPointC2 &p) const
  {
      return !(*this == p);
  }
};

template <typename K, class ConstructBbox_2>
class TgConstruct_bbox_2 : public ConstructBbox_2 {
public:
  using ConstructBbox_2::operator();
  CGAL::Bbox_2 operator()(const TgPointC2<K>& p) const {
    return CGAL::Bbox_2(p.x(), p.y(), p.x(), p.y());
  }
};

template <typename K>
class TgConstruct_coord_iterator {
  typedef typename K::FT         FT;
public:
  typedef const FT*              result_type;

  const FT* operator()(const TgPointC2<K>& p) const
  {
    return &p.x();
  }
  const FT* operator()(const TgPointC2<K>& p, int)
  {
    const FT* pyptr = &p.y();
    pyptr++;
    return pyptr;
  }
};

template <typename K, typename OldK>
class TgConstruct_point_2
{
  typedef typename K::RT         RT;
  typedef typename K::Point_2    Point_2;
  typedef typename K::Line_2     Line_2;
  typedef typename Point_2::Rep  Rep;
public:
  typedef Point_2                result_type;
  // Note : the CGAL::Return_base_tag is really internal CGAL stuff.
  // Unfortunately it is needed for optimizing away copy-constructions,
  // due to current lack of delegating constructors in the C++ standard.
  Rep // Point_2
  operator()(CGAL::Return_base_tag, CGAL::Origin o) const
  { return Rep(o); }
  Rep // Point_2
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y) const
  { return Rep(x, y); }
  Rep // Point_2
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y, const RT& w) const
  { return Rep(x, y, w); }
  Point_2
  operator()(const CGAL::Origin&) const
  { return TgPointC2<K>(0, 0, 0); }
  Point_2
  operator()(const RT& x, const RT& y) const
  {
    return TgPointC2<K>(x, y, 0);
  }
  Point_2
  operator()(const Line_2& l) const
  {
    typename OldK::Construct_point_2 base_operator;
    Point_2 p = base_operator(l);
    return p;
  }
  Point_2
  operator()(const Line_2& l, int i) const
  {
    typename OldK::Construct_point_2 base_operator;
    return base_operator(l, i);
  }
  // We need this one, as such a functor is in the Filtered_kernel
  Point_2
  operator()(const RT& x, const RT& y, const RT& w) const
  {
    if(w != 1){
      return TgPointC2<K>(x/w, y/w, 0);
    } else {
      return TgPointC2<K>(x,y, 0);
    }
  }
};

#endif // TG_POINTC2_H
